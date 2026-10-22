"""Read-only physical tracking and source-epoch localization consistency audit."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.reporting import write_json
from joint_limit_recovery_analysis import control_continuity, stalled_windows, telemetry_summary
from tracking_loss_analysis import analyze, read, stats


def aligned_pose(rows, times):
    source = np.array([r['source_time'] for r in rows])
    position = np.array([r['data']['position'][:2] for r in rows])
    angles = Rotation.from_quat([r['data']['orientation'] for r in rows]).as_euler('xyz')[:, 2]
    angles = np.unwrap(angles)
    index = np.searchsorted(source, times)
    inside = (index > 0) & (index < len(source))
    index = np.clip(index, 1, len(source)-1)
    inside &= (source[index] - source[index-1] <= .15)
    return np.column_stack([np.interp(times, source, position[:, k]) for k in range(2)]
                           + [np.interp(times, source, angles)]), inside


def audit(directory, output):
    context = json.loads((directory/'resolved_configuration.json').read_text())['context']
    result = json.loads((directory/'result.json').read_text())
    historical = analyze(directory, load_yaml(Path(__file__).with_name('tracking_loss_experiments.yaml')),
                         output/'tracking')
    truth = read(directory/'telemetry/base_truth.jsonl')
    estimates = read(directory/'telemetry/robot_estimate.jsonl')
    times = np.array([r['source_time'] for r in estimates])
    actual, valid = aligned_pose(truth, times)
    means = np.array([r['data']['mean'] for r in estimates])
    rotation = Rotation.from_euler('xyz', context['robot']['pose'][3:]).as_matrix()[:2, :2]
    estimated = means[:, :3].copy()
    estimated[:, :2] = means[:, :2] @ rotation.T + np.array(context['robot']['pose'][:2])
    estimated[:, 2] += context['robot']['pose'][5]
    errors = estimated - actual
    errors[:, 2] = np.arctan2(np.sin(errors[:, 2]), np.cos(errors[:, 2]))
    covariance = np.array([r['data']['covariance'] for r in estimates]).reshape(-1, means.shape[1], means.shape[1])[:, :3, :3]
    J = np.eye(3); J[:2, :2] = rotation
    covariance = J @ covariance @ J.T
    selected = valid & (times >= 20.) & (times < 90.)
    err = errors[selected]; cov = covariance[selected]
    nees = np.einsum('ni,nij,nj->n', err, np.linalg.inv(cov), err)
    pose_metrics = {'samples': int(selected.sum()),
        'xy_error_m': stats(np.linalg.norm(err[:, :2], axis=1)),
        'absolute_yaw_deg': stats(np.abs(err[:, 2])*180/np.pi),
        'nees_3d': stats(nees), 'fraction_inside_95percent_3d_ellipsoid': float(np.mean(nees <= 7.8147279)),
        'marginal_95percent_coverage_xyz_yaw': np.mean(np.abs(err) <= 1.96*np.sqrt(cov.diagonal(axis1=1, axis2=2)), axis=0).tolist(),
        'min_covariance_eigenvalue': float(np.linalg.eigvalsh(covariance).min()),
        'std_xy_yaw_mean': np.mean(np.sqrt(cov.diagonal(axis1=1, axis2=2)), axis=0).tolist(),
        'error_xy_yaw_rmse': np.sqrt(np.mean(err**2, axis=0)).tolist(),
        'pose_increment_position_m': stats(np.linalg.norm(np.diff(estimated[:, :2], axis=0), axis=1)),
        'pose_increment_yaw_deg': stats(np.abs(np.arctan2(np.sin(np.diff(estimated[:, 2])),np.cos(np.diff(estimated[:, 2]))))*180/np.pi),
        'estimate_source_interval_s': stats(np.diff(times)),
        'largest_position_increment_source_interval_s': float(np.diff(times)[np.argmax(np.linalg.norm(np.diff(estimated[:, :2], axis=0), axis=1))]),
        'alignment': 'Offline linear XY / unwrapped-yaw interpolation at estimate source epoch; brackets <=0.15 s. Never fed to runtime.'}
    diagnostics = read(directory/'telemetry/diagnostics.jsonl')
    parsed = [r['data']['parsed'] for r in diagnostics if r['data'].get('parsed')]
    sensor_path = directory/'telemetry/localization_measurement.jsonl'
    measurements = read(sensor_path) if sensor_path.exists() else []
    sensor_metrics = {'samples': len(measurements)}
    if measurements:
        stamps = np.array([r['source_time'] for r in measurements])
        source, ok = aligned_pose(truth, stamps)
        observed = np.column_stack([np.array([r['data']['position'][:2] for r in measurements]),
            Rotation.from_quat([r['data']['orientation'] for r in measurements]).as_euler('xyz')[:, 2]])
        delta = observed[ok]-source[ok]
        delta[:, 2] = np.arctan2(np.sin(delta[:, 2]), np.cos(delta[:, 2]))
        sensor_metrics.update(error_mean_xy_yaw=np.mean(delta, axis=0).tolist(),
            error_std_xy_yaw=np.std(delta, axis=0).tolist(),
            recorded_clock_minus_source_s=stats([r['clock']-r['source_time'] for r in measurements if r['clock'] is not None]),
            arrival_clock_caveat='Recorder clock and callback order are asynchronous; estimator timing counters are the authoritative receive ages.',
            observed_rate_hz=float((len(stamps)-1)/(stamps[-1]-stamps[0])))
    control = control_continuity(directory/'telemetry/control.jsonl', 1.5, 90.)
    control['valid_sample_fraction'] = control['valid_samples']/control['samples']
    entire_control = control_continuity(directory/'telemetry/control.jsonl', 0., 90.)
    entire_control['valid_sample_fraction'] = entire_control['valid_samples']/entire_control['samples']
    summary = {'directory': str(directory), 'result': result, 'localization_20_90': pose_metrics,
        'physical_0_90': historical['stages']['0-90'],
        'control_1p5_90': control, 'control_0_90': entire_control, 'sensor': sensor_metrics,
        'final_estimation_diagnostics': parsed[-1] if parsed else None,
        'max_localization_source_processing_age_s': max((r.get('timing',{}).get('sources',{}).get('localization',{}).get('processing_age_max_s',0.) for r in parsed),default=None),
        'actuation': telemetry_summary(directory/'telemetry',1.5,90.),
        'stall': stalled_windows(directory/'physics/actuator.csv')}
    write_json(output/'summary.json', summary)
    print(directory.name, pose_metrics['xy_error_m']['mean'], pose_metrics['absolute_yaw_deg']['mean'], flush=True)
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--study', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--names', nargs='+', help='Analyze only named completed cases; retain other analyses.')
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    summaries = {}
    for directory in sorted(a.study.iterdir()):
        if a.names and directory.name not in a.names:
            continue
        if (directory/'result.json').exists() and (directory/'telemetry/robot_estimate.jsonl').exists():
            result = json.loads((directory/'result.json').read_text())
            if result.get('error') or 'window' not in result:
                summaries[directory.name] = {'excluded_incomplete_attempt': result}
                continue
            destination = a.output/directory.name; destination.mkdir()
            summaries[directory.name] = audit(directory, destination)
    write_json(a.output/'summary.json', summaries)


if __name__ == '__main__':
    main()
