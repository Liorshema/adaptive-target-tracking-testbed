"""Summarize optical-frame truth, robot-state agreement and callback ordering."""

import heapq
import json
from pathlib import Path

import numpy as np
from robot_bringup.factory.model_factory import create_robot_model
from robot_models.whole_body.kinematics import WholeBodyKinematics
from scipy.spatial.transform import Rotation

DATA = Path(__file__).resolve().parent


def matrix(pose):
    rotation, position = pose.get('orientation', {}), pose.get('position', {})
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(
        [rotation.get(key, 0.) for key in ['x', 'y', 'z']] + [rotation.get('w', 1.)]).as_matrix()
    result[:3, 3] = [position.get(key, 0.) for key in ['x', 'y', 'z']]
    return result


def main():
    events = json.loads((DATA / 'orientation_z_baseline/processing.json').read_text())
    summary = {}
    for source in ['wheel', 'joints', 'imu']:
        selected = [e for e in events if e['source'] == source and 5 <= e['stamp'] <= 35]
        rejected = [e for e in selected if e['rejected']]
        late = [e['previous'] - e['stamp'] for e in rejected
                if e['previous'] is not None and e['stamp'] < e['previous']]
        summary[source] = {
            'callbacks': len(selected), 'callback_hz': len(selected) / 30,
            'rejected': len(rejected),
            'within_source_regressions': int(np.sum(np.diff([e['stamp'] for e in selected]) < 0)),
            'lateness_to_filter_quantiles': np.quantile(late, [.5, .95, 1]).tolist() if late else [],
            'arrival_delay_quantiles': np.quantile(
                [e['now'] - e['stamp'] for e in selected], [.5, .95, 1]).tolist() if selected else []}
    for delay in [0., .01, .02, .03]:
        pending, last, rejected, kept = [], -1., 0, 0
        for index, event in enumerate(events):
            heapq.heappush(pending, (event['stamp'], index))
            while pending and pending[0][0] <= event['now'] - delay:
                timestamp, _ = heapq.heappop(pending)
                if timestamp < last:
                    rejected += 1
                else:
                    kept += 1
                    last = timestamp
        while pending:
            timestamp, _ = heapq.heappop(pending)
            if timestamp < last:
                rejected += 1
            else:
                kept += 1
                last = timestamp
        summary[f'offline_reorder_{delay}'] = {
            'accepted': kept, 'late_dropped': rejected,
            'note': 'Timestamp-order replay only; no live estimator or accuracy claim'}
    model = create_robot_model()
    for directory in ['orientation_z_baseline', 'orientation_x_corrected']:
        out = DATA / directory
        rows = json.loads((out / 'poses.json').read_text())
        streams = json.loads((out / 'streams.json').read_text())
        frames = {}
        for line in (out / 'gazebo_poses.jsonl').read_text().splitlines():
            if not line.startswith('{'):
                continue
            frame = json.loads(line)
            time = frame['header']['stamp']
            key = round(float(time.get('sec', 0)) + float(time.get('nsec', 0)) * 1e-9, 6)
            frames[key] = frame
        position_error, rotation_error = [], []
        for row in rows:
            if row['time'] < 10:
                continue
            frame = frames[round(row['time'], 6)]
            base = next(pose for pose in frame['pose'] if pose['name'] == 'robot')
            fk = WholeBodyKinematics.compute(model, matrix(base), np.array(row['q']))
            actual = np.asarray(row['physical_pose'])
            position_error.append(np.linalg.norm(fk[:3, 3] - actual[:3, 3]))
            rotation_error.append(np.degrees(Rotation.from_matrix(
                fk[:3, :3].T @ actual[:3, :3]).magnitude()))
        times = np.array([entry['time'] for entry in streams['estimate']])
        state = np.array([entry['mean'] for entry in streams['estimate']])
        odom_times = np.array([entry['time'] for entry in streams['odom']])
        odom = np.array([entry['base'] for entry in streams['odom']])
        joint_times = np.array([entry['time'] for entry in streams['joint']])
        joints = np.array([entry['q'] for entry in streams['joint']])
        mask = (times >= 10) & (times <= min(35, odom_times[-1], joint_times[-1]))
        odom_aligned = np.column_stack([np.interp(times, odom_times, odom[:, i]) for i in range(3)])
        joints_aligned = np.column_stack([
            np.interp(times, joint_times, joints[:, i]) for i in range(joints.shape[1])])
        summary[directory] = {
            'true_base_fk_position_mean': float(np.mean(position_error)),
            'true_base_fk_position_max': float(np.max(position_error)),
            'true_base_fk_rotation_degrees_max': float(np.max(rotation_error)),
            'estimate_vs_wheel_position_mean': float(np.mean(np.linalg.norm(
                state[mask, :2] - odom_aligned[mask, :2], axis=1))),
            'estimate_vs_wheel_yaw_mean_degrees': float(np.mean(np.abs(
                state[mask, 2] - odom_aligned[mask, 2])) * 180 / np.pi),
            'estimate_vs_joints_mean_rad': np.mean(np.abs(
                state[mask, 5:5+joints.shape[1]] - joints_aligned[mask]), axis=0).tolist()}
    (DATA / 'orientation_timing_analysis.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
