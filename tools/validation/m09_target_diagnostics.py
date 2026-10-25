#!/usr/bin/env python3
"""Recompute M09 target evidence from preserved normalized telemetry (no ROS runs).

Run after sourcing install/setup.bash, or put workspace src package roots on
PYTHONPATH. Output is additive: choose a new --output directory for a new audit.
"""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml

from robot_experiments.metrics.target_diagnostics import (
    bounded_interpolate, common_grid_roughness,
    error_metrics, replay_constant_velocity, sampling_summary, turn_lags,
)


plt.switch_backend('Agg')


STREAMS = ('target_truth', 'target_measurement', 'target_estimate', 'prediction',
           'control', 'command', 'actuation', 'joints')


def load_stream(path):
    records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    summary = sampling_summary([r['source_time'] for r in records])
    summary['frames'] = sorted({r.get('frame_id', '') for r in records})
    if summary['nonfinite_times'] or summary['backward_steps']:
        raise ValueError(f'{path}: split invalid/reset epochs before analysis')
    if summary['duplicates']:
        # Before /clock starts, inactive control/command heartbeats share epoch 0.
        inactive_startup = path.stem in ('control', 'command') and all(
            not r['data'].get('valid', r['data'].get('active', True))
            for r in records if r['source_time'] == 0.)
        active_times = [r['source_time'] for r in records if r['source_time'] != 0.]
        if not inactive_startup or len(set(active_times)) != len(active_times):
            raise ValueError(f'{path}: ambiguous duplicate epochs')
    return records, summary


def values(records, key):
    return np.asarray([r['data'][key] for r in records], dtype=float)


def stamps(records):
    return np.asarray([r['source_time'] for r in records], dtype=float)


def summarize(array):
    array = np.asarray(array, dtype=float)
    array = array[np.isfinite(array)]
    return {'n': int(array.size), 'mean': float(np.mean(array)) if array.size else None,
            'median': float(np.median(array)) if array.size else None,
            'p95': float(np.percentile(array, 95)) if array.size else None,
            'max': float(np.max(array)) if array.size else None}


def correlation(a, b):
    a, b = np.asarray(a), np.asarray(b)
    valid = np.isfinite(a) & np.isfinite(b)
    return {'n': int(valid.sum()), 'pearson': float(np.corrcoef(a[valid], b[valid])[0, 1])
            if valid.sum() > 2 and np.std(a[valid]) > 1e-12
            and np.std(b[valid]) > 1e-12 else None}


def parameters(run):
    # Both M08 and M09 archive the effective merged parameters in estimation.yaml.
    result = {}
    for name in ['estimation.yaml', 'target_estimation.yaml']:
        path = run / 'inputs' / name
        if path.exists():
            result.update(yaml.safe_load(path.read_text())['estimation_node']['ros__parameters'])
    required = ['target_initial_state', 'target_initial_covariance_diagonal',
                'target_position_variance', 'target_acceleration_noise_std']
    if not all(name in result for name in required):
        raise ValueError(f'{run}: effective target configuration unavailable')
    return result


def coupling(records, replay, mask, joint_names):
    """Bounded temporal associations; absent input lineage precludes causality."""
    targets = records['target_estimate']
    forecasts = records['prediction']
    ft = stamps(forecasts)
    controls = [r for r in records['control'] if r['data'].get('valid')]
    ct = stamps(controls)
    joints = records['joints']
    jt = stamps(joints)
    wrist = 2 + joint_names.index('wrist_joint')
    rows = []
    for i in np.flatnonzero(mask):
        time = targets[i]['source_time']
        f = int(np.searchsorted(ft, time - 1e-9))
        if f == 0 or f >= len(ft) or abs(ft[f] - time) > 1e-8:
            continue
        forecast = forecasts[f]['data']
        prior = forecasts[f - 1]['data']
        evaluation_time = forecast['timestamps'][0]
        old = bounded_interpolate(prior['timestamps'], prior['states'],
                                  [evaluation_time], .101)[0, :3]
        if not np.isfinite(old).all():
            continue
        prediction_revision = np.asarray(forecast['states'][0][:3]) - old
        c = int(np.searchsorted(ct, time))
        if c == 0 or c >= len(ct) or ct[c] - time > .12 or ct[c] - ct[c - 1] > .12:
            continue
        current, previous = controls[c]['data'], controls[c - 1]['data']
        left, right = np.searchsorted(jt, [ct[c], ct[c] + .12])
        observed = []
        for sample in joints[left:right]:
            names = sample['data']['names']
            if 'wrist_joint' in names:
                observed.append(abs(sample['data']['velocity'][names.index('wrist_joint')]))
        rows.append({
            'estimate_time_s': float(time), 'control_time_s': float(ct[c]),
            'source_time_offset_s': float(ct[c] - time),
            'position_correction_m': float(np.linalg.norm(replay['correction'][i, :3])),
            'prediction_revision_m': float(np.linalg.norm(prediction_revision)),
            'raw_wrist_rad_s': float(current['raw_velocity'][wrist]),
            'safe_wrist_rad_s': float(current['safe_velocity'][wrist]),
            'raw_wrist_change_rad_s': float(abs(
                current['raw_velocity'][wrist] - previous['raw_velocity'][wrist])),
            'safe_wrist_change_rad_s': float(abs(
                current['safe_velocity'][wrist] - previous['safe_velocity'][wrist])),
            'physical_wrist_peak_next_120ms_rad_s': max(observed) if observed else None,
            'feedback_samples': len(observed),
        })
    comparisons = {}
    for a, b in [('position_correction_m', 'prediction_revision_m'),
                 ('prediction_revision_m', 'raw_wrist_change_rad_s'),
                 ('position_correction_m', 'raw_wrist_change_rad_s'),
                 ('raw_wrist_change_rad_s', 'safe_wrist_change_rad_s'),
                 ('safe_wrist_change_rad_s', 'physical_wrist_peak_next_120ms_rad_s')]:
        comparisons[f'{a}__{b}'] = correlation(
            [r[a] for r in rows], [r[b] if r[b] is not None else np.nan for r in rows])
    largest = sorted(rows, key=lambda row: row['position_correction_m'], reverse=True)[:5]
    return {'max_control_offset_s': .12, 'physical_following_window_s': .12,
            'comparison': comparisons, 'largest_correction_events': largest,
            'rows': rows,
            'limitations': 'Control telemetry has no consumed prediction epoch or input '
                           'sequence. These source-time neighbors are associations, not '
                           'identified propagation or causal estimates. Overlapping '
                           'physical windows and correlated samples are not independent.'}


def plot_run(run_name, output, times, truth, measurement, estimate, replay,
             coupling_result, covariance, records, wrist_index):
    colors = ['#2a9d42', '#888888', '#e07a16']
    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True,
                                constrained_layout=True)
    for i, axis in enumerate(axes[:3]):
        for array, color, label in zip([truth, measurement, estimate], colors,
                                       ['Truth', 'Measurement', 'Estimate']):
            axis.plot(times, array[:, i], color=color, lw=1.3 if label != 'Measurement'
                      else .7, alpha=.9 if label != 'Measurement' else .6, label=label)
        axis.set_ylabel('xyz'[i] + ' (m)')
        axis.grid(alpha=.2)
    axes[0].legend(loc='upper right', ncol=3)
    axes[0].set_title(run_name + ': source-time aligned target; raw posterior samples')
    axes[3].plot(times, np.linalg.norm(measurement - truth, axis=1), color=colors[1],
                 label='Measurement error norm')
    axes[3].plot(times, np.linalg.norm(estimate - truth, axis=1), color=colors[2],
                 label='Estimate error norm')
    axes[3].plot(times, np.sqrt(np.trace(covariance[:, :3, :3], axis1=1, axis2=2)),
                 color='#3466b2', label='sqrt(trace(P_position))')
    axes[3].set_ylabel('m')
    axes[3].set_xlabel('Simulation source time (s)')
    axes[3].grid(alpha=.2)
    axes[3].legend(ncol=3, fontsize=8)
    figure.savefig(output / f'{run_name}_target_timeseries.png', dpi=150)
    plt.close(figure)
    figure, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                                constrained_layout=True)
    axes[0].plot(times, np.linalg.norm(replay['innovation'], axis=1), label='Pre-update innovation')
    axes[0].plot(times, np.linalg.norm(replay['correction'][:, :3], axis=1),
                 label='Position correction')
    axes[0].set_ylabel('m')
    axes[0].legend(ncol=2)
    rows = coupling_result['rows']
    if rows:
        event_times = [r['estimate_time_s'] for r in rows]
        axes[1].plot(event_times, [r['prediction_revision_m'] for r in rows],
                     label='Forecast revision, common future epoch')
        axes[1].set_ylabel('m')
        axes[1].legend()
    controls = [r for r in records['control']
                if times[0] <= r['source_time'] <= times[-1]]
    for key, color in [('raw_velocity', '#9c51a0'), ('safe_velocity', '#2459aa')]:
        valid = [r for r in controls if r['data'].get('valid')]
        axes[2].plot(stamps(valid), [r['data'][key][wrist_index] for r in valid], color=color,
                     lw=1., label=key + ' wrist')
    joints = [r for r in records['joints']
              if times[0] <= r['source_time'] <= times[-1]]
    joint_velocity = [r['data']['velocity'][r['data']['names'].index('wrist_joint')]
                      for r in joints]
    axes[2].plot(stamps(joints), joint_velocity, '.', color='#777777',
                 markersize=1.6, alpha=.55, label='Measured wrist samples')
    axes[2].set_ylabel('rad/s')
    axes[2].set_xlabel('Simulation source time (s)')
    axes[2].legend(ncol=3, fontsize=8)
    for axis in axes:
        axis.set_xlim(times[0], times[-1])
        axis.grid(alpha=.2)
    axes[0].set_title(run_name + ': temporal coupling (input lineage unavailable)')
    figure.savefig(output / f'{run_name}_coupling_timeseries.png', dpi=150)
    plt.close(figure)


def analyze(run, output, run_name):
    records, summaries = {}, {}
    for name in STREAMS:
        records[name], summaries[name] = load_stream(run / 'telemetry' / f'{name}.jsonl')
    for name in STREAMS[:4]:
        if summaries[name]['frames'] != ['world']:
            raise ValueError(f'{run}/{name}: compatible world frame required')
    observation_times = stamps(records['target_measurement'])
    estimate_times = stamps(records['target_estimate'])
    if (observation_times.shape != estimate_times.shape
            or not np.allclose(observation_times, estimate_times, atol=2e-9, rtol=0.)):
        raise ValueError('1:1 posterior/measurement epoch contract not verified')
    p = parameters(run)
    measurement = values(records['target_measurement'], 'position')
    estimate = values(records['target_estimate'], 'mean')
    covariance = values(records['target_estimate'], 'covariance').reshape((-1, 6, 6))
    replay = replay_constant_velocity(
        observation_times, measurement, p['target_initial_state'],
        np.diag(p['target_initial_covariance_diagonal']), p['target_position_variance'],
        p['target_acceleration_noise_std'])
    fidelity = {'max_mean_abs_error': float(np.max(abs(replay['mean'] - estimate))),
                'max_covariance_abs_error': float(np.max(abs(replay['covariance'] - covariance)))}
    if max(fidelity.values()) > 1e-8:
        raise ValueError(f'{run}: replay does not reproduce live posterior: {fidelity}')
    truth = bounded_interpolate(stamps(records['target_truth']),
                                values(records['target_truth'], 'position'),
                                observation_times, max_gap=.06)
    # A common +1s startup exclusion, with bounded truth support, across all captures.
    mask = ((observation_times >= observation_times[0] + 1.)
            & np.isfinite(truth).all(axis=1))
    times = observation_times[mask]
    truth, measurement, estimate = truth[mask], measurement[mask], estimate[mask]
    covariance = covariance[mask]
    error = estimate[:, :3] - truth
    nees = np.einsum('ni,nij,nj->n', error, np.linalg.inv(covariance[:, :3, :3]), error)
    selected_replay = {key: value[mask] for key, value in replay.items()}
    innovations = selected_replay['innovation']
    sensitivity = []
    for sigma in [.25, .5, 1.]:
        for r_scale in [.5, 1., 2.]:
            offline = replay_constant_velocity(
                observation_times, values(records['target_measurement'], 'position'),
                p['target_initial_state'], np.diag(p['target_initial_covariance_diagonal']),
                np.asarray(p['target_position_variance']) * r_scale, sigma)
            offline_estimate = offline['mean'][mask, :3]
            roughness = common_grid_roughness(times, [truth, offline_estimate])
            sensitivity.append({'sigma_a': sigma, 'r_scale': r_scale,
                                'error': error_metrics(offline_estimate - truth),
                                'roughness': roughness['series'][1]})
    coupling_result = coupling(records, replay, mask, p['joint_names'])
    result = {
        'run': str(run), 'window_s': [float(times[0]), float(times[-1])],
        'startup_excluded_s': 1., 'paired_samples': int(mask.sum()),
        'missing_truth_samples_total': int(np.sum(~np.isfinite(
            bounded_interpolate(stamps(records['target_truth']),
                                values(records['target_truth'], 'position'),
                                observation_times, .06)).all(axis=1))),
        'sampling': summaries, 'effective_target_parameters': {
            key: p[key] for key in p if key.startswith('target_')},
        'replay_fidelity': fidelity,
        'measurement_error': error_metrics(measurement - truth),
        'estimation_error': error_metrics(error),
        'roughness_100ms': common_grid_roughness(times, [truth, measurement, estimate[:, :3]]),
        'roughness_200ms': common_grid_roughness(
            times, [truth, measurement, estimate[:, :3]], step=.2),
        'turn_lags': turn_lags(times, truth, estimate[:, :3], step=.05, radius=.5,
                               minimum_excursion=.01),
        'covariance': {
            'full_capture_position_trace_first_last_m2': [
                float(np.trace(replay['covariance'][0, :3, :3])),
                float(np.trace(replay['covariance'][-1, :3, :3]))],
            'min_eigenvalue': float(np.min(np.linalg.eigvalsh(covariance))),
            'max_asymmetry': float(np.max(abs(covariance - covariance.transpose(0, 2, 1)))),
            'position_trace_m2': summarize(np.trace(covariance[:, :3, :3], axis1=1, axis2=2)),
            'position_trace_first_last_m2': [
                float(np.trace(covariance[0, :3, :3])),
                float(np.trace(covariance[-1, :3, :3]))]},
        'innovation': {'norm_m': summarize(np.linalg.norm(innovations, axis=1)),
                       'bias_axis_m': np.mean(innovations, axis=0).tolist(),
                       'lag1_correlation_axis': [
                           correlation(innovations[:-1, j], innovations[1:, j])
                           for j in range(3)],
                       'nis_3d': summarize(selected_replay['nis']),
                       'nis_below_3d_95pct_fraction': float(np.mean(selected_replay['nis'] <= 7.8147279)),
                       'position_gain_diagonal': [summarize(selected_replay['gain'][:, j, j])
                                                  for j in range(3)],
                       'correction_norm_m': summarize(np.linalg.norm(selected_replay['correction'][:, :3], axis=1))},
        'position_nees_3d': summarize(nees),
        'position_nees_below_3d_95pct_fraction': float(np.mean(nees <= 7.8147279)),
        'offline_sensitivity': sensitivity, 'coupling': coupling_result,
        'source_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in [*(run / 'telemetry' / (name + '.jsonl') for name in STREAMS),
                                       run / 'inputs' / 'estimation.yaml',
                                       run / 'inputs' / 'target_estimation.yaml',
                                       run / 'inputs' / 'measurements.yaml']},
    }
    plot_run(run_name, output, times, truth, measurement, estimate[:, :3],
             selected_replay, coupling_result, covariance, records,
             2 + p['joint_names'].index('wrist_joint'))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.cwd())
    parser.add_argument('--run', type=Path, action='append', help='Run with telemetry/ and inputs/')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('Refusing to overwrite existing evidence: use a fresh --output directory')
    runs = args.run or sorted(args.workspace.glob(
        'results/experiments/m08_verified_final/paired_prediction_*/runs/*')) + sorted(
        args.workspace.glob('results/experiments/m08_final_seed917/paired_seed917_*/runs/*')) + sorted(
        args.workspace.glob('results/m09_headless_verified/*'))
    args.output.mkdir(parents=True)
    results = {}
    for run in runs:
        name = 'm09_headless' if 'm09_headless_verified' in str(run) else run.name
        results[name] = analyze(run, args.output, name)
    payload = {'schema_version': 1, 'description': 'M09 immutable-recording target diagnostic audit',
               'alignment': {'frame': 'world', 'truth_max_bracket_gap_s': .06,
                             'measurement_estimate': 'one-to-one source epochs within 2ns (one-ns stamp serialization loss)',
                             'roughness_grid_s': [.1, .2], 'roughness_max_bracket_gap_s': .12,
                             'time_basis': 'simulation source time, never recorder wall time'},
               'runs': results}
    (args.output / 'm09_target_diagnostic_evidence.json').write_text(
        json.dumps(payload, indent=2, allow_nan=False) + '\n')
    overview = {key: {'window': value['window_s'],
                      'measurement_rmse_m': value['measurement_error']['rmse_norm_m'],
                      'estimate_rmse_m': value['estimation_error']['rmse_norm_m'],
                      'replay': value['replay_fidelity']}
                for key, value in results.items()}
    print(json.dumps(overview, indent=2))


if __name__ == '__main__':
    main()
