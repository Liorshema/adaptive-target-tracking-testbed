"""Reuse physical metrics; audit periodic motion, all speed violations and reachability."""

import json
from pathlib import Path

import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import yaml

from analyze_reactive_v3 import extra_metrics, unwrapped_production_summary
from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import load_robot_model_parameters
from robot_models.whole_body.kinematics import WholeBodyKinematics

HERE = Path(__file__).resolve().parents[2] / 'results/validation'
CASES = ['aggressive_A_before', 'aggressive_A_gui_reference', 'aggressive_A',
         'aggressive_A_gui', 'aggressive_B_initial', 'aggressive_B', 'aggressive_C',
         'aggressive_B_gui']


def reachability_certificate(path, snapshot, positions):
    """Certify a straight-arm family plus free base XY placement on flat ground.

    For each target height between two valid shoulder postures, continuity gives
    a posture with matching virtual endpoint height p_C + R*f. Free horizontal
    base placement then matches target XY exactly. This is a static geometric
    certificate, not a claim about nonholonomic/rate-limited trajectory tracking.
    Samples outside this sufficient family are unresolved, not called unreachable.
    """
    description_path = path / 'reachability_description.yaml'
    description_path.write_text(yaml.safe_dump({'robot': snapshot['description']}, sort_keys=False))
    parameters = load_robot_model_parameters(description_path)
    model = create_robot_model(parameters)
    physical = list(snapshot['description']['manipulator']['joints'].values())
    axes = np.array([joint['axis'] for joint in physical])
    pitch = next(i for i, axis in enumerate(axes)
                 if np.linalg.norm(np.cross(axis, [0., 0., 1.])) > 1e-12)
    lower, upper = parameters.joint_limits.lower, parameters.joint_limits.upper
    if not (np.all(lower <= 0.) and np.all(upper >= 0.)):
        raise ValueError('Straight-arm certificate requires a valid zero posture')
    robot = snapshot.get('robot')
    if robot is None:  # Earlier captures predate inclusion of robot deployment in snapshots.
        robot = yaml.safe_load((HERE.parents[1] /
            'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
    base = np.eye(4)
    base[:3, :3] = Rotation.from_euler('xyz', robot['pose'][3:]).as_matrix()
    if not np.allclose(base[:3, :3], np.eye(3)):
        raise ValueError('Certificate is limited to the existing level deployment')
    base[:3, 3] = robot['pose'][:3]
    radius = snapshot['control']['tracking_radius']
    axis = np.array(snapshot['control']['tracking_forward_axis'])
    axis = axis / np.linalg.norm(axis)

    def virtual_endpoint(angle, transform=base):
        q = np.zeros(len(physical))
        q[pitch] = angle
        camera = WholeBodyKinematics.compute(model, transform, q)
        return camera[:3, 3] + radius * camera[:3, :3] @ axis

    heights = [virtual_endpoint(angle)[2] for angle in [lower[pitch], upper[pitch]]]
    inside = (positions[:, 2] >= min(heights)) & (positions[:, 2] <= max(heights))
    residuals = []
    for index in np.unique(np.r_[np.linspace(0, len(positions) - 1, 7, dtype=int),
                                 np.argmin(positions[:, 2]), np.argmax(positions[:, 2])]):
        if not inside[index]:
            continue
        target = positions[index]
        angle = brentq(lambda q: virtual_endpoint(q)[2] - target[2], lower[pitch], upper[pitch])
        placed = base.copy()
        placed[:2, 3] += target[:2] - virtual_endpoint(angle)[:2]
        residuals.append(float(np.linalg.norm(virtual_endpoint(angle, placed) - target)))
    return {'certified_samples': int(inside.sum()), 'total_samples': len(inside),
            'unresolved_samples': int((~inside).sum()),
            'observed_geometrically_unreachable_events': 0 if inside.all() else None,
            'certified_target_height_interval': sorted(heights),
            'checked_ik_max_residual': max(residuals, default=None),
            'dynamic_reachability_certified': False}


def summarize(path):
    """Keep startup and settled speed audits separate without weakening gates."""
    data, full, origin = unwrapped_production_summary(path)
    data = extra_metrics(path, data, full, origin)
    result = json.loads((path / 'result.json').read_text())
    snapshot = yaml.safe_load((path / 'configuration_snapshot.yaml').read_text())
    streams = json.loads((path / 'experiment_streams.json').read_text())
    settle, duration = result['thresholds']['settling_time'], data['duration']
    time = full[:, 0] - origin
    selected = full[time >= settle]
    differences = np.diff(selected, axis=0)
    positive = differences[:, 0] > 0.
    velocity = differences[positive, 1:4] / differences[positive, :1]
    reversals = []
    for index in [1, 2]:
        signs = np.sign(velocity[np.abs(velocity[:, index]) > .001, index])
        reversals.append(int(np.count_nonzero(np.diff(signs))))
    data.update(target_yz_reversals=reversals,
                minimum_target_z=float(full[:, 3].min()), maximum_target_z=float(full[:, 3].max()),
                reachability=reachability_certificate(path, snapshot, full[:, 1:4]),
                actuation_counters=result['counters'],
                watchdog_events_after_settling=result['watchdog_timeouts_after_settling'],
                saturation_cycles=result['saturation_cycles'],
                initial_simulation_time=origin,
                motion_weights=snapshot['strategy']['controller']['parameters']['motion_weights'],
                full_speed_violations={}, settled_speed_violations={})
    periodic = next(field['parameters'] for field in snapshot['environment']['environment']['acceleration_fields']
                    if field['type'] == 'sinusoidal')
    data['periodic_parameters'] = periodic
    data['initial_effective_phase_rad'] = np.mod(
        2 * np.pi * np.array(periodic['frequency']) * origin + periodic['phase'], 2 * np.pi).tolist()
    differences_full = np.diff(full, axis=0)
    positive_full = differences_full[:, 0] > 0.
    data['max_target_speed_full'] = float(np.linalg.norm(
        differences_full[positive_full, 1:4] / differences_full[positive_full, :1], axis=1).max())
    limits = np.array(data['physical_joint_speed_limits'])
    for label, start in [('full', -np.inf), ('settled', settle)]:
        rows = [row for row in streams['joints'] if start <= row[0] - origin <= duration]
        stamp = np.array([row[0] for row in rows])
        speed = np.abs([row[2] for row in rows])
        violation = speed > limits + data['numerical_tolerance']
        for index, name in enumerate(snapshot['joint_order']):
            mask = violation[:, index]
            data[label + '_speed_violations'][name] = {
                'limit': float(limits[index]), 'peak': float(speed[:, index].max()),
                'samples': int(mask.sum()),
                'episodes': int(np.count_nonzero(mask & ~np.r_[False, mask[:-1]])),
                'observed_seconds': float(mask[:-1] @ np.maximum(np.diff(stamp), 0.))}
    workspace = yaml.safe_load((Path(__file__).with_name('saturated_dynamics_cases.yaml')).read_text())
    data['workspace_bound'] = workspace['closed_loop_workspace_radius']
    data['workspace_check'] = data['max_center_displacement'] < data['workspace_bound']
    return data, full, origin


def main():
    """Produce a source-backed comparison and one inspectable XYZ/error plot."""
    results, traces = {}, {}
    for case in CASES:
        path = HERE / case
        if (path / 'result.json').exists():
            results[case], full, origin = summarize(path)
            traces[case] = (full, origin)
    if all(key in results for key in ['aggressive_B', 'aggressive_C']):
        before, after = results['aggressive_B'], results['aggressive_C']
        nonworse = {}
        for phase in ['full', 'settled']:
            nonworse[phase] = all(
                after[phase + '_speed_violations'][joint]['peak'] <=
                max(before[phase + '_speed_violations'][joint]['peak'], limit) +
                after['numerical_tolerance'] and
                after[phase + '_speed_violations'][joint]['observed_seconds'] <=
                before[phase + '_speed_violations'][joint]['observed_seconds']
                for joint, limit in zip(after[phase + '_speed_violations'],
                                        after['physical_joint_speed_limits']))
        results['allocation_comparison'] = {
            'no_worse_physical_speed': nonworse,
            'base_travel_ratio': after['base_travel'] / before['base_travel'],
            'joint_travel_ratio': (np.array(after['joint_travel']) / before['joint_travel']).tolist()}
    (HERE / 'aggressive_oscillatory_comparison.json').write_text(json.dumps(results, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for case in ['aggressive_A', 'aggressive_B', 'aggressive_C']:
        if case not in traces:
            continue
        full, origin = traces[case]
        for i in [0, 1, 2]:
            axes[i].plot(full[:, 0] - origin, full[:, i + 1], label=case[-1])
            axes[i].set_ylabel(f'World {"XYZ"[i]} [m]')
        tracking = np.array(json.loads((HERE / case / 'physical_tracking.json').read_text()))
        axes[3].plot(tracking[:, 0] - origin, tracking[:, 2], label=case[-1])
    axes[0].legend(title='Experiment')
    axes[3].set_ylabel('Pointing [deg]')
    axes[3].set_xlabel('Elapsed simulation time [s]')
    for axis in axes:
        axis.grid(alpha=.25)
    figure.tight_layout()
    figure.savefig(HERE / 'aggressive_oscillatory_tracking.png', dpi=140)
    plt.close(figure)
    print(json.dumps({case: {key: value for key, value in data.items() if key in [
        'mean_rtf', 'xyz_ranges_settled', 'mean_distance_error', 'mean_pointing_degrees',
        'base_travel', 'joint_travel', 'full_measured_joint_speed_peaks', 'target_yz_reversals',
        'reachability', 'checks']} for case, data in results.items()}, indent=2))


if __name__ == '__main__':
    main()
