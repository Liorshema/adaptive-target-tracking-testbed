"""Reuse native-Gazebo metrics and add matched v3 travel/speed comparisons."""

import json
from pathlib import Path

import numpy as np
import yaml

from analyze_saturated_milestone import summarize

HERE = Path(__file__).resolve().parent
CASES = ['v3_baseline_before', 'v3_baseline_after', 'v3_3d_original', 'v3_arm_4x_rejected', 'v3_3d_arm',
         'v3_selected_long', 'v3_baseline_gui']


def extra_metrics(path, data, full, origin):
    """Use all received physical joints and base poses over the settled window."""
    result = json.loads((path / 'result.json').read_text())
    snapshot = yaml.safe_load((path / 'configuration_snapshot.yaml').read_text())
    streams = json.loads((path / 'experiment_streams.json').read_text())
    settle, duration = result['thresholds']['settling_time'], data['duration']

    def selected(key):
        return [r for r in streams[key] if settle <= r[0] - origin <= duration]

    base = selected('physical_base')
    positions = np.array([r[1] for r in base])
    from scipy.spatial.transform import Rotation
    yaw = np.unwrap(Rotation.from_quat([r[2] for r in base]).as_euler('xyz')[:, 2])
    joints = selected('joints')
    time = np.array([r[0] for r in joints])
    q, dq = np.array([r[1] for r in joints]), np.array([r[2] for r in joints])
    physical = list(snapshot['description']['manipulator']['joints'].values())
    limits = np.array([r['velocity'] for r in physical])
    tolerance = result['thresholds']['physical_numerical_tolerance']
    violation = np.abs(dq) > limits + tolerance
    bounds_low, bounds_high = np.array([r['lower'] for r in physical]), np.array([r['upper'] for r in physical])
    all_q = np.array([r[1] for r in streams['joints']])
    data.update(
        base_travel=float(np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1).sum()),
        base_net_displacement=float(np.linalg.norm(positions[-1, :2] - positions[0, :2])),
        base_yaw_travel=float(np.abs(np.diff(yaw)).sum()),
        joint_travel=np.abs(np.diff(q, axis=0)).sum(axis=0).tolist(),
        joint_position_check=bool(np.all(q >= bounds_low - tolerance) and
                                  np.all(q <= bounds_high + tolerance)),
        full_joint_position_check=bool(np.all(all_q >= bounds_low - tolerance) and
                                       np.all(all_q <= bounds_high + tolerance)),
        full_measured_joint_speed_peaks=np.abs([r[2] for r in streams['joints']]).max(axis=0).tolist(),
        speed_violation_sample_fraction=violation.mean(axis=0).tolist(),
        speed_violation_seconds=(violation[:-1] * np.maximum(np.diff(time), 0.)[:, None]).sum(axis=0).tolist(),
        measured_joint_samples=len(joints),
        max_displacement_from_initial=float(np.linalg.norm(
            full[:, 1:4] - snapshot['simulation']['target']['initial_position'], axis=1).max()),
        target_path_length_settled=result['experiment']['target_path_length'],
        physical_joint_speed_limits=limits.tolist(), numerical_tolerance=tolerance,
        mean_compute_ms=result['mean_compute_ms'], p95_compute_ms=result['p95_compute_ms'],
        process_cpu=result['mean_process_cpu_percent'],
        approximate_observed_topic_sim_hz=result['observed_topic_rates_sim_hz'],
        checks=result['checks'], xyz_ranges_by_10s={})
    for start in np.arange(0., duration, 10.):
        block = full[(full[:, 0] >= origin + start) & (full[:, 0] < origin + start + 10.)]
        if len(block) > 1:
            data['xyz_ranges_by_10s'][f'{start:g}-{start + 10:g}'] = np.ptp(block[:, 1:4], axis=0).tolist()
    parameters = snapshot['target']['target']['dynamics']['parameters']
    field = snapshot['environment']['environment']['acceleration_fields'][0]['parameters']
    data['interaction_bound'] = float(np.hypot(parameters['a_max'],
        parameters.get('tangential', {}).get('acceleration', 0.)))
    if full.shape[1] >= 13:
        environment = -(full[:, 1:4] - np.array(field['center'])) @ np.array(field['gain_matrix']).T
        interaction = full[:, 7:10] + parameters['damping'] * full[:, 4:7] - environment
        data['max_radial_acceleration'] = data.pop('max_interaction_acceleration')
        data['max_interaction_acceleration'] = float(np.linalg.norm(interaction, axis=1).max())
        data['bounded_interaction_check'] = data['max_interaction_acceleration'] <= data['interaction_bound'] + 1e-12
    profile_path = path / 'compute_cpu_wall.json'
    if profile_path.exists():
        profile = np.array(json.loads(profile_path.read_text()))
        profile = profile[profile[:, 2] == 0]
        data['unprofiled_pipeline_mean_wall_cpu_ms'] = (profile[:, :2].mean(axis=0) * 1000.).tolist()
    return data


def unwrapped_production_summary(path):
    """Use public target positions when the permanent launch has no trace prefix."""
    result = json.loads((path / 'result.json').read_text())
    streams = json.loads((path / 'experiment_streams.json').read_text())
    experiment = result['experiment']
    origin, duration = result['target_initialization_time'], result['thresholds']['duration']
    full = np.array([[r[0], *r[1]] for r in streams['targets']
                     if 0. <= r[0] - origin <= duration])
    data = {key: result[key] for key in ['mean_rtf', 'duration'] if key in result}
    data.update(duration=duration, source=str(path),
                xyz_ranges_full=np.ptp(full[:, 1:4], axis=0).tolist(),
                xyz_ranges_settled=experiment['xyz_ranges'],
                mean_target_speed=experiment['mean_target_speed'],
                max_target_speed=experiment['max_target_speed'],
                target_speed_source='finite differences of public ground-truth positions',
                max_center_displacement=experiment['max_center_distance'],
                measured_joint_speed_peaks=result['joint_velocity_peaks'],
                mean_distance_error=experiment['mean_physical_distance_error'],
                max_distance_error=experiment['max_physical_distance_error'],
                mean_pointing_degrees=experiment['mean_physical_pointing_degrees'],
                max_pointing_degrees=experiment['max_physical_pointing_degrees'],
                minimum_shoulder=experiment['minimum_shoulder'],
                minimum_first_link_clearance=experiment['minimum_first_link_box_base_clearance'],
                finite=bool(np.isfinite(full).all()))
    return data, full, origin


def main():
    """Summarize available runs; plots share scales and use the actual optical axis."""
    metrics, traces = {}, {}
    for case in CASES:
        path = HERE / case
        if not (path / 'result.json').exists():
            continue
        data, full, origin = (summarize(path) if (path / 'target_trace.json').exists()
                              else unwrapped_production_summary(path))
        metrics[case] = extra_metrics(path, data, full, origin)
        traces[case] = (full, origin)
    if 'v3_3d_original' in metrics and 'v3_3d_arm' in metrics:
        before, after = metrics['v3_3d_original'], metrics['v3_3d_arm']
        metrics['allocation_comparison'] = dict(
            base_travel_ratio=after['base_travel'] / before['base_travel'],
            joint_travel_ratio=(np.array(after['joint_travel']) / before['joint_travel']).tolist(),
            no_increased_speed_violations=bool(np.all(
                np.maximum(np.array(after['measured_joint_speed_peaks']) -
                           np.array(after['physical_joint_speed_limits']), 0.) <=
                np.maximum(np.array(before['measured_joint_speed_peaks']) -
                           np.array(before['physical_joint_speed_limits']), 0.) +
                after['numerical_tolerance'])),
            no_increased_observed_violation_duration=bool(np.all(
                np.array(after['speed_violation_seconds']) <= before['speed_violation_seconds'])))
    (HERE / 'reactive_v3_comparison.json').write_text(json.dumps(metrics, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for case in ['v3_baseline_after', 'v3_3d_original', 'v3_3d_arm', 'v3_selected_long']:
        if case not in traces:
            continue
        trace, origin = traces[case]
        for index, axis in enumerate(axes):
            axis.plot(trace[:, 0] - origin, trace[:, index + 1], label=case, linewidth=1.)
            axis.set_ylabel(f'World {"XYZ"[index]} [m]')
            axis.grid(alpha=.25)
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel('Elapsed simulation time [s]')
    figure.tight_layout()
    figure.savefig(HERE / 'reactive_v3_xyz.png', dpi=140)
    plt.close(figure)
    print(json.dumps({key: {k: v for k, v in value.items() if k in [
        'mean_rtf', 'xyz_ranges_settled', 'mean_distance_error', 'mean_pointing_degrees',
        'base_travel', 'joint_travel', 'measured_joint_speed_peaks', 'checks']}
        for key, value in metrics.items()}, indent=2))


if __name__ == '__main__':
    main()
