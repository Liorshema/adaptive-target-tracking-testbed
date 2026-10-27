"""Independent native-Gazebo posture and raw target/TF experiment measurements."""
from robot_control.configuration import load_control_parameters

import json
from pathlib import Path

import numpy as np

from scipy.spatial.transform import Rotation

import yaml


def pose_matrix(pose):
    """Decode a native Gazebo pose, including omitted zero-valued protobuf fields."""
    result = np.eye(4)
    position, rotation = pose.get('position', {}), pose.get('orientation', {})
    result[:3, 3] = [position.get(key, 0.) for key in ['x', 'y', 'z']]
    result[:3, :3] = Rotation.from_quat([
        rotation.get(key, 1. if key == 'w' else 0.)
        for key in ['x', 'y', 'z', 'w']]).as_matrix()
    return result


def experiment_metrics(output, streams, description, robot, origin, thresholds, control=None, center=None):
    """Measure every captured physical first-link pose, including startup."""
    link = description['manipulator']['link1']
    if control is None:
        config = Path(__file__).resolve().parents[2] / 'src/robot_control/config/control.yaml'
        control = load_control_parameters(config, 'parameters')['control_node']['ros__parameters']
    # The authoritative Xacro puts the link box along +X from its joint origin.
    corners = np.array([[x, y, z, 1.] for x in [0., link['length']]
                        for y in [-link['width'] / 2, link['width'] / 2]
                        for z in [-link['height'] / 2, link['height'] / 2]])
    physical = []
    tracking = []
    targets = np.array([entry[1] for entry in streams['targets']])
    target_times = np.array([entry[0] for entry in streams['targets']])
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler(
        'xyz', description['tracking_frame']['rotation_rpy']).as_matrix()
    mount[:3, 3] = description['tracking_frame']['translation']
    for line in (output / 'gazebo_poses.jsonl').read_text().splitlines():
        try:
            scene = json.loads(line)
        except json.JSONDecodeError:
            continue
        poses = {pose['name']: pose for pose in scene.get('pose', [])}
        if robot['entity_name'] not in poses or 'arm_link_1' not in poses:
            continue
        stamp = scene['header']['stamp']
        time = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) * 1e-9
        base = pose_matrix(poses[robot['entity_name']])
        # SceneBroadcaster link poses are model-relative, not world-relative.
        arm = pose_matrix(poses['arm_link_1'])
        axis = arm[:3, 0]
        physical.append({
            'time': time,
            'elevation': float(np.arcsin(np.clip(axis[2], -1., 1.))),
            'world_elevation': float(np.arcsin(np.clip(
                (base[:3, :3] @ axis)[2], -1., 1.))),
            'box_base_clearance': float(np.min((arm @ corners.T)[2])
                                        - description['body']['height'] / 2)})
        parent = description['tracking_frame']['parent']
        if (parent in poses and targets.size and
                thresholds['settling_time'] <= time - origin <= thresholds['duration']):
            index = int(np.argmin(np.abs(target_times - time)))
            if abs(target_times[index] - time) <= thresholds['max_feedback_skew']:
                camera = base @ pose_matrix(poses[parent]) @ mount
                relative = targets[index] - camera[:3, 3]
                distance = np.linalg.norm(relative)
                optical = camera[:3, :3] @ np.asarray(control['tracking_forward_axis'])
                pointing = np.degrees(np.arccos(np.clip(
                    optical @ relative / (np.linalg.norm(optical) * distance), -1., 1.)))
                tracking.append([time, float(distance - control['tracking_radius']),
                                 float(pointing)])
    if not physical or not tracking or not streams['joints'] or not streams['targets']:
        raise RuntimeError('Incomplete physical posture or target capture')
    selected = [sample for sample in streams['targets']
                if thresholds['settling_time'] <= sample[0] - origin <= thresholds['duration']]
    time, positions = np.array([s[0] for s in selected]), np.array([s[1] for s in selected])
    dt = np.diff(time)
    valid = dt > 0
    speeds = np.linalg.norm(np.diff(positions, axis=0)[valid] / dt[valid, None], axis=1)
    shoulder = list(description['manipulator']['joints']).index('shoulder')
    all_q = np.array([s[1] for s in streams['joints']])[:, shoulder]
    tf_positions = np.array([s[1] for s in streams['tracking_tf']])
    native = np.array([s['elevation'] for s in physical])
    result = dict(
        physical_samples=len(physical), joint_samples=len(all_q),
        minimum_shoulder=float(all_q.min()), maximum_shoulder=float(all_q.max()),
        minimum_link_elevation=float(native.min()),
        minimum_world_link_elevation=min(s['world_elevation'] for s in physical),
        minimum_first_link_box_base_clearance=min(s['box_base_clearance'] for s in physical),
        mean_target_speed=float(speeds.mean()), max_target_speed=float(speeds.max()),
        target_path_length=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
        target_max_displacement_from_initial=float(np.max(
            np.linalg.norm(positions - streams['targets'][0][1], axis=1))),
        target_net_displacement=float(np.linalg.norm(positions[-1] - positions[0])),
        xyz_ranges=np.ptp(positions, axis=0).tolist(), min_z=float(positions[:, 2].min()),
        max_z=float(positions[:, 2].max()),
        physical_tracking_samples=len(tracking),
        mean_physical_distance_error=float(np.mean([abs(s[1]) for s in tracking])),
        max_physical_distance_error=float(np.max([abs(s[1]) for s in tracking])),
        mean_physical_pointing_degrees=float(np.mean([s[2] for s in tracking])),
        max_physical_pointing_degrees=float(np.max([s[2] for s in tracking])),
        tracking_tf_samples=len(tf_positions),
        tracking_tf_excursion=(float(np.max(
            np.linalg.norm(tf_positions - tf_positions[0], axis=1)))
            if len(tf_positions) else 0.),
        joint_tolerance=thresholds['physical_numerical_tolerance'])
    if center is not None:
        all_positions = np.array([entry[1] for entry in streams['targets']])
        result['max_center_distance'] = float(np.linalg.norm(all_positions - center, axis=1).max())
    raw_velocity = [entry[2] for entry in streams['joints']
                    if len(entry) > 2 and thresholds['settling_time'] <=
                    entry[0] - origin <= thresholds['duration']]
    if raw_velocity:
        result['raw_joint_speed_peaks'] = np.abs(raw_velocity).max(axis=0).tolist()
    (output / 'physical_posture.json').write_text(json.dumps(physical))
    (output / 'physical_tracking.json').write_text(json.dumps(tracking))
    (output / 'experiment_streams.json').write_text(json.dumps(streams))
    return result
