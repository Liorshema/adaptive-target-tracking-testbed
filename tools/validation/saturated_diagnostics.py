"""Read-only joint/PID, odometry and timestamp diagnostics from real captures."""

import argparse
import heapq
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def estimation(path):
    events = json.loads((path / 'processing.json').read_text())
    sources = {}
    for source in ['wheel', 'joints', 'imu']:
        rows = [r for r in events if r['source'] == source and 5. <= r['stamp'] <= 40.]
        if not rows:
            sources[source] = {'callbacks': 0}
            continue
        age = np.array([r['now'] - r['stamp'] for r in rows])
        lateness = [r['previous'] - r['stamp'] for r in rows
                    if r['rejected'] and r['previous'] is not None and r['stamp'] < r['previous']]
        sources[source] = dict(callbacks=len(rows), rejected=sum(r['rejected'] for r in rows),
            rate_hz=len(rows) / 35.,
            source_timestamp_regressions=sum(b['stamp'] < a['stamp'] for a, b in zip(rows, rows[1:])),
            arrival_age_median_ms=float(np.median(age) * 1000),
            arrival_age_p95_ms=float(np.quantile(age, .95) * 1000),
            watermark_lateness_p95_ms=float(np.quantile(lateness, .95) * 1000) if lateness else 0.)
    replay = {}
    for delay in [0., .01, .02, .03, .05]:
        pending = []
        watermark, accepted, late = -np.inf, 0, 0
        def commit(timestamp):
            nonlocal watermark, accepted, late
            if timestamp < watermark:
                late += 1
            else:
                watermark, accepted = timestamp, accepted + 1
        for i, event in enumerate(events):
            if not np.isfinite(event['stamp']) or event['stamp'] > event['now'] + .01:
                continue
            heapq.heappush(pending, (event['stamp'], i))
            while pending and pending[0][0] <= event['now'] - delay:
                commit(heapq.heappop(pending)[0])
        while pending:
            commit(heapq.heappop(pending)[0])
        replay[str(delay)] = dict(accepted=accepted, late=late)
    return {'sources': sources, 'timestamp_only_replay': replay}


def actuation(path):
    streams = json.loads((path / 'experiment_streams.json').read_text())
    result = json.loads((path / 'result.json').read_text())
    origin = result['target_initialization_time']
    rows = [row for row in streams['joints'] if 10. <= row[0] - origin <= result['thresholds']['duration']]
    times = np.array([r[0] for r in rows])
    q, dq = np.array([r[1] for r in rows]), np.array([r[2] for r in rows])
    delta = np.diff(times)
    good = delta > 0.
    finite_difference = np.diff(q, axis=0)[good] / delta[good, None]
    mean_velocity = ((dq[:-1] + dq[1:]) / 2)[good]
    trace = json.loads((path / 'actuation_trace.json').read_text())
    reference_rows = [e for e in trace if e['reference'] is not None and e['time'] >= origin + 10.]
    ordinary, resets = [], []
    for a, b in zip(reference_rows, reference_rows[1:]):
        dt = b['time'] - a['time']
        if dt > 0:
            rate = np.abs(np.array(b['reference']) - a['reference']) / dt
            (ordinary if a['active'] and b['active'] else resets).append(rate)
    peaks = []
    for joint in range(q.shape[1]):
        i = int(np.argmax(np.abs(dq[:, joint])))
        references = [e for e in reference_rows if e['time'] <= times[i]]
        event = references[-1] if references else reference_rows[0]
        peaks.append(dict(time=float(times[i]), position=float(q[i, joint]),
                          speed=float(dq[i, joint]), reference=event['reference'][joint],
                          reference_time=event['time'],
                          command=None if event['command'] is None else event['command'][joint + 2]))
    # Compare joint-speed response to timestamped command signals over positive delays.
    commands = [e for e in streams['commands'] if e[2] and e[0] >= origin + 10.]
    ct = np.array([e[0] for e in commands])
    cv = np.array([e[1][2:] for e in commands])
    correlations = {}
    for delay in [0., .02, .05, .1, .15, .2]:
        idx = np.searchsorted(ct, times - delay, side='right') - 1
        valid = idx >= 0
        values = []
        for joint in range(q.shape[1]):
            x, y = cv[idx[valid], joint], dq[valid, joint]
            values.append(float(np.corrcoef(x, y)[0, 1]) if np.std(x) > 1e-9 and np.std(y) > 1e-9 else 0.)
        correlations[str(delay)] = values
    return dict(received_joint_rate_hz=len(rows) / (times[-1] - times[0]),
                measured_speed_peaks=np.abs(dq).max(axis=0).tolist(),
                position_difference_speed_peaks=np.abs(finite_difference).max(axis=0).tolist(),
                finite_difference_vs_velocity_median_error=np.median(
                    np.abs(finite_difference - mean_velocity), axis=0).tolist(),
                ordinary_reference_rate_peaks=np.max(ordinary, axis=0).tolist() if ordinary else [],
                reset_reference_rate_peaks=np.max(resets, axis=0).tolist() if resets else [],
                speed_peak_snapshots=peaks, command_speed_correlations=correlations)


def odometry(path):
    streams = json.loads((path / 'experiment_streams.json').read_text())
    snapshot = yaml.safe_load((path / 'configuration_snapshot.yaml').read_text())
    robot_path = Path(__file__).resolve().parents[2] / 'src/robot_simulation/config/gazebo/robot.yaml'
    robot = yaml.safe_load(robot_path.read_text())['robot']
    pose = robot['pose']
    spawn = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    truth = streams['physical_base']
    times = np.array([e[0] for e in truth])
    errors = []
    for time, position, quaternion in streams['odometry']:
        i = int(np.argmin(np.abs(times - time)))
        if abs(times[i] - time) > .08:
            continue
        world_position = spawn @ position + pose[:3]
        world_rotation = spawn @ Rotation.from_quat(quaternion).as_matrix()
        actual_rotation = Rotation.from_quat(truth[i][2]).as_matrix()
        errors.append([time, np.linalg.norm(world_position - truth[i][1]),
                       np.degrees(Rotation.from_matrix(world_rotation.T @ actual_rotation).magnitude())])
    values = np.array(errors)
    wheels = snapshot['description']['wheels']
    return dict(samples=len(values), wheel_radius=wheels['radius'], track_width=2 * wheels['y_offset'],
                mean_position_error=float(values[:, 1].mean()), max_position_error=float(values[:, 1].max()),
                mean_orientation_error_degrees=float(values[:, 2].mean()),
                max_orientation_error_degrees=float(values[:, 2].max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[2] / 'results/validation'
    parser.add_argument('--stressed', type=Path, default=root / 'saturated_candidate_suggested')
    parser.add_argument('--gentle', type=Path, default=root / 'saturated_candidate_gentle')
    parser.add_argument('--selected', type=Path, default=root / 'saturated_final_120')
    parser.add_argument('--output', type=Path, default=root / 'saturated_diagnostics.json')
    args = parser.parse_args()
    result = {'estimation': estimation(args.stressed),
              'stressed_actuation': actuation(args.stressed),
              'gentle_actuation': actuation(args.gentle),
              'selected_odometry': odometry(args.selected)}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
