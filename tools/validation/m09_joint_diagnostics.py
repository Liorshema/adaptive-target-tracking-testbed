#!/usr/bin/env python3
"""Reproduce gap-aware M09 joint diagnostics from preserved M08 JSONL only.

Usage: source install/setup.bash; python3 tools/validation/m09_joint_diagnostics.py
Optional --output-prefix writes a separate set of derived evidence artifacts.
"""

import argparse
import hashlib
from collections import Counter
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for package in (ROOT / 'src').iterdir():
    if package.is_dir():
        sys.path.insert(0, str(package))
from robot_experiments.metrics.joint_diagnostics import (  # noqa: E402
    body_to_world, causal_indices, named_joint_series, sampled_motion,
)

GAP = .01  # Five nominal 500 Hz feedback intervals; no long-gap interpolation.
COMMAND_AGE = .15  # Analysis cutoff: three nominal controller cycles; never future align.
ACTUATION_AGE = .15  # 10 Hz snapshots; no claim that reference is continuously observed.
STREAMS = ('joints', 'base_truth', 'target_truth', 'odometry', 'control', 'command', 'actuation', 'target_measurement',
           'target_estimate', 'prediction', 'robot_estimate', 'diagnostics')


def read_records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def selected(records, start, end):
    return [r for r in records if r.get('source_time') is not None and
            start <= r['source_time'] <= end]


def series(records, field, length, predicate=lambda r: True):
    valid = [r for r in records if r.get('source_time') is not None and predicate(r) and
             len(r['data'].get(field, [])) == length]
    # Duplicate publications at one source time cannot represent a positive interval.
    unique = {r['source_time']: r for r in valid}
    valid = [unique[t] for t in sorted(unique)]
    return (np.array([r['source_time'] for r in valid]),
            np.array([r['data'][field] for r in valid]).reshape((-1, length)), valid)


def errors(t, measured, commands_t, commands, age):
    indices = causal_indices(commands_t, t, age)
    valid = indices >= 0
    if not np.any(valid):
        return {'available': False}
    difference = measured[valid] - commands[indices[valid]]
    dt = np.diff(t)
    weights = np.where((dt <= GAP + 1e-12) & valid[:-1] & valid[1:], dt, 0.)
    full = np.full(len(t), np.nan)
    full[valid] = difference
    eligible = weights > 0
    observed = float(weights.sum())
    integ = np.sum(weights[eligible] *
                   (full[:-1][eligible]**2 + full[1:][eligible]**2)/2)
    return {'available': True, 'aligned_samples': int(valid.sum()),
            'observed_duration_s': observed,
            'mean_error_rad_s': float(np.mean(difference)),
            'sample_rmse_rad_s': float(np.sqrt(np.mean(difference**2))),
            'observed_time_rms_error_rad_s': float(np.sqrt(integ/observed))
            if observed else None,
            'peak_abs_error_rad_s': float(np.max(np.abs(difference))),
            'max_command_age_s': float(np.max(t[valid]-commands_t[indices[valid]]))}


def counters(records, start, end):
    r = selected(records, start, end)
    fields = ('accepted_commands', 'rejected_commands', 'timeout_events',
              'reference_limit_events', 'inactive_commands', 'feedback_rejections')
    if not r:
        return {'available': False}
    return {'available': True, 'first_snapshot_time': r[0]['source_time'],
            'last_snapshot_time': r[-1]['source_time'],
            'first': {f: r[0]['data'][f] for f in fields},
            'last': {f: r[-1]['data'][f] for f in fields},
            'observed_deltas': {f: r[-1]['data'][f]-r[0]['data'][f] for f in fields},
            'reasons': dict(Counter(x['data']['reason'] for x in r))}


def event_evidence(streams, time, names):
    start, end = time-.2, time+.2
    event = {'peak_time_s': time, 'inspection_window_s': [start, end], 'signals': {}}
    for stream in STREAMS:
        records = streams[stream]
        # Estimator diagnostic String envelopes intentionally have null source_time;
        # parsed.time is the publisher's source-time field, never substitute arrival time.
        if stream == 'diagnostics':
            records = [dict(r, source_time=r['data'].get('parsed', {}).get('time'))
                       for r in records]
        subset = selected(records, start, end)
        event['signals'][stream] = [
            {'source_time': r['source_time'], 'arrival_monotonic': r['arrival_monotonic'],
             'frame_id': r['frame_id'], 'data': r['data']} for r in subset]
    t, q, v, _ = named_joint_series(streams['joints'], 'wrist_joint')
    at = int(np.argmin(abs(t-time)))
    event['physical_peak_velocity_rad_s'] = float(v[at])
    event['peak_position_rad'] = float(q[at])
    if at > 0:
        dt = float(t[at]-t[at-1])
        event['preceding_feedback_interval'] = {
            'start_s': float(t[at-1]), 'end_s': float(t[at]), 'duration_s': dt,
            'position_increment_rad': float(q[at]-q[at-1]),
            'finite_difference_velocity_rad_s': float((q[at]-q[at-1])/dt),
            'contiguous_physics_steps': bool(abs(dt-.002)<1e-10),
        }
    event['position_increment_checks'] = []
    for halfwidth in [.002, .01, .02, .05]:
        lo = int(np.searchsorted(t, time-halfwidth))
        hi = int(np.searchsorted(t, time+halfwidth, side='right')-1)
        if hi <= lo:
            continue
        dt = np.diff(t[lo:hi+1])
        event['position_increment_checks'].append({
            'window_s': [float(t[lo]), float(t[hi])], 'sample_count': hi-lo+1,
            'largest_gap_s': float(np.max(dt)),
            'position_increment_rad': float(q[hi]-q[lo]),
            'finite_difference_velocity_rad_s': float((q[hi]-q[lo])/(t[hi]-t[lo])),
            'trapezoid_velocity_integral_rad': float(np.sum(
                dt*(v[lo:hi]+v[lo+1:hi+1])/2)),
        })
    event['actuation_counters'] = counters(streams['actuation'], start, end)
    ct, cv, cr = series(streams['command'], 'velocity', len(names)+2)
    ci = causal_indices(ct, [time], COMMAND_AGE)[0]
    if ci >= 0:
        r = cr[ci]
        event['preceding_command'] = {'source_time': float(ct[ci]), 'age_s': time-ct[ci],
                                      'data': r['data']}
    at, av, ar = series(streams['actuation'], 'position_reference', len(names))
    ai = causal_indices(at, [time], ACTUATION_AGE)[0]
    if ai >= 0:
        event['preceding_actuation_snapshot'] = {
            'source_time': float(at[ai]), 'age_s': time-at[ai], 'data': ar[ai]['data']}
    return event


def summarize(run):
    metadata = json.loads((run/'metadata.json').read_text())
    context = json.loads((run/'resolved_configuration.json').read_text())['context']
    names = context['joint_names']
    definitions = context['description']['robot']['manipulator']['joints']
    template = context['robot']['joint_name_template']
    limits = {template.format(key=key): value for key, value in definitions.items()}
    wheel_names = sum([context['robot']['differential_drive'][side+'_joints']
                       for side in ['left', 'right']], [])
    streams = {s: read_records(run/'telemetry'/f'{s}.jsonl') for s in STREAMS}
    start, end = (metadata['window'][x] for x in ['window_start', 'window_end'])
    result = {'dataset': str(run.relative_to(ROOT)), 'metadata_window': metadata['window'],
              'arm_name_order_from_resolved_configuration': names, 'arm': {}, 'wheels': {},
              'capture_arm': {}, 'alignment': {'feedback_max_gap_s': GAP,
              'command_max_age_s': COMMAND_AGE, 'actuation_snapshot_max_age_s': ACTUATION_AGE,
              'command_rule': 'causal left hold at source time, no future nearest neighbor'},
              'frames': {s: sorted(set(r['frame_id'] for r in records))
                         for s, records in streams.items()}}
    capture_end = end  # include startup, exclude post-window collector shutdown tail
    for name in names + wheel_names:
        t, q, v, rejected = named_joint_series(streams['joints'], name)
        limit = limits.get(name, {})
        kwargs = {'speed_limit': limit.get('velocity'),
                  'position_limits': (limit['lower'], limit['upper']) if limit else None,
                  'tolerance': context['physical_numerical_tolerance']}
        metrics = sampled_motion(t, q, v, start, end, GAP, **kwargs)
        metrics['rejected_samples'] = rejected
        (result['arm'] if name in names else result['wheels'])[name] = metrics
        if name in names:
            result['capture_arm'][name] = sampled_motion(t, q, v, t[0], capture_end, GAP,
                                                       **kwargs)
    raw_t, raw, _ = series(streams['control'], 'raw_velocity', len(names)+2)
    safe_t, safe, _ = series(streams['control'], 'safe_velocity', len(names)+2)
    cmd_t, cmd, cmd_records = series(streams['command'], 'velocity', len(names)+2)
    # Resolve SafeVelocityCommand's own arm ordering by name before any comparison.
    cmd = np.asarray([list(v[:2]) + [v[2+r['data']['joint_names'].index(n)] for n in names]
                      for v, r in zip(cmd, cmd_records)])
    act_t, act, _ = series(streams['actuation'], 'commanded_velocity', len(names)+2)
    result['input_hashes'] = {file: hashlib.sha256((run/file).read_bytes()).hexdigest()
                              for file in ['metadata.json', 'resolved_configuration.json']}
    result['stream_audit'] = {}
    for label, records in streams.items():
        times = [r['source_time'] for r in records if r.get('source_time') is not None]
        file = run/'telemetry'/f'{label}.jsonl'
        result['stream_audit'][label] = {
            'file': str(file.relative_to(ROOT)),
            'sha256': hashlib.sha256(file.read_bytes()).hexdigest(),
            'received_count': len(records), 'nonnull_source_time_count': len(times),
            'duplicate_source_times': len(times)-len(set(times)),
            'out_of_order_source_times': int(np.sum(np.diff(times)<0)),
            'evaluation_sample_count': len(selected(records, start, end)),
            'first_source_time': times[0] if times else None,
            'last_source_time': times[-1] if times else None,
        }
    result['command_comparison'] = {}
    for i, name in enumerate(names):
        t, q, v, _ = named_joint_series(selected(streams['joints'], start, end), name)
        result['command_comparison'][name] = {
            label: errors(t, v, ts, val[:, i+2], age) for label, ts, val, age in [
                ('raw_WLS', raw_t, raw, COMMAND_AGE),
                ('safe_controller', safe_t, safe, COMMAND_AGE),
                ('published_SafeVelocityCommand', cmd_t, cmd, COMMAND_AGE),
                ('accepted_command_snapshot', act_t, act, ACTUATION_AGE)]}
    mask = (raw_t >= start) & (raw_t <= end)
    result['raw_vs_safe'] = {'sample_count': int(mask.sum()),
                            'peak_abs_difference_by_control_axis':
                            np.max(abs(raw[mask]-safe[mask]), axis=0).tolist()}
    aligned = causal_indices(safe_t, cmd_t, .001)
    good = ((cmd_t>=start)&(cmd_t<=end)&(aligned>=0))
    result['safe_vs_published'] = {
        'same_source_cycle_samples': int(good.sum()),
        'peak_abs_difference_by_control_axis': np.max(
            abs(cmd[good]-safe[aligned[good]]), axis=0).tolist(),
        'command_joint_name_sets': sorted(set(tuple(r['data']['joint_names'])
                                              for r in cmd_records)),
    }
    result['actuation_counters_evaluation'] = counters(streams['actuation'], start, end)
    result['actuation_counters_capture'] = counters(streams['actuation'], 0., end)
    result['control_invalid_reasons_evaluation'] = dict(Counter(
        r['data']['reason'] for r in selected(streams['control'], start, end)
        if not r['data']['valid']))
    result['command_publication'] = {
        'largest_gap_evaluation_s': float(np.max(np.diff(cmd_t[(cmd_t>=start)&(cmd_t<=end)]))),
        'median_gap_evaluation_s': float(np.median(np.diff(cmd_t[(cmd_t>=start)&(cmd_t<=end)])))}
    result['base'] = base_summary(streams, start, end, raw_t, raw, safe_t, safe, cmd_t, cmd)
    t, q, v, _ = named_joint_series(streams['joints'], 'wrist_joint')
    candidates = np.flatnonzero((t>=start)&(t<=end))
    peaks = []
    for index in candidates[np.argsort(-abs(v[candidates]))]:
        if all(abs(t[index]-p) >= .2 for p in peaks):
            peaks.append(float(t[index]))
        if len(peaks) == 3:
            break
    # Include full-capture maximum too; important startup events lack commands.
    capture_mask = np.flatnonzero(t<=end)
    capture_peak = float(t[capture_mask[np.argmax(abs(v[capture_mask]))]])
    if all(abs(capture_peak-p) >= .2 for p in peaks):
        peaks.append(capture_peak)
    result['wrist_events'] = [event_evidence(streams, p, names) for p in peaks]
    effort = [r['data']['effort'] for r in streams['joints']]
    result['effort_observability'] = {
        'all_reported_joint_effort_zero': all(all(x == 0. for x in a) for a in effort),
        'PID_torque': 'unavailable; zero-valued JointState effort is not measured PID torque',
        'reference_topic_publications': 'unavailable; only 10 Hz ActuationTelemetry snapshots'}
    result['coupling'] = coupling_summary(streams, start, end, names)
    return result, streams


def base_summary(streams, start, end, raw_t, raw, safe_t, safe, cmd_t, cmd):
    records = selected(streams['base_truth'], start, end)
    if any(r['frame_id'] != 'world' or r['data']['child_frame_id'] != 'base_link'
           for r in records):
        raise ValueError('unexpected ground-truth frame contract')
    t = np.array([r['source_time'] for r in records])
    p = np.array([r['data']['position'] for r in records])
    body_v = np.array([r['data']['linear_velocity'] for r in records])
    world_v = np.array([body_to_world(r['data']['orientation'], v)
                        for r, v in zip(records, body_v)])
    world_w = np.array([body_to_world(r['data']['orientation'], r['data']['angular_velocity'])
                       for r in records])
    w = np.array([r['data']['angular_velocity'][2] for r in records])
    d = np.diff(p, axis=0)/np.diff(t)[:, None]
    result = {'sample_count': len(t), 'largest_gap_s': float(np.max(np.diff(t))),
              'twist_contract': 'Odometry body/child twist; rotate xyzw orientation to world',
              'world_position_difference_vs_rotated_twist_RMSE_m_s':
              float(np.sqrt(np.mean((d-world_v[1:])**2))),
              'world_position_difference_vs_unrotated_twist_RMSE_m_s':
              float(np.sqrt(np.mean((d-body_v[1:])**2))),
              'sampled_horizontal_path_m': float(np.sum(np.linalg.norm(np.diff(p[:,:2],axis=0),
                                                                         axis=1))),
              'horizontal_excursion_from_first_sample_m':
              float(np.max(np.linalg.norm(p[:,:2]-p[0,:2],axis=1))),
              'world_vertical_angular_velocity_peak_rad_s': float(np.max(abs(world_w[:,2])))}
    for i, label in enumerate(['forward_v', 'yaw_rate']):
        actual = body_v[:,0] if i == 0 else w
        dt = np.diff(t)
        observed = dt <= .11  # 20 Hz source ground truth; never join a long loss.
        integral = np.sum(dt[observed]*(actual[:-1][observed]**2+actual[1:][observed]**2)/2)
        val = {'physical_peak_abs': float(np.max(abs(actual))),
               'physical_observed_RMS': float(np.sqrt(integral/dt[observed].sum())),
               'physical_observed_duration_s': float(dt[observed].sum()),
               'physical_coverage_fraction': float(dt[observed].sum()/(end-start))}
        for lab, ts, values in [('raw', raw_t, raw), ('safe', safe_t, safe),
                                ('published', cmd_t, cmd)]:
            mask = (ts>=start)&(ts<=end)
            val[lab+'_peak_abs'] = float(np.max(abs(values[mask,i])))
            val[lab+'_sample_RMS'] = float(np.sqrt(np.mean(values[mask,i]**2)))
        index = causal_indices(cmd_t, t, COMMAND_AGE)
        valid = index >= 0
        val['physical_minus_causal_published_sample_RMSE'] = float(np.sqrt(np.mean(
            (actual[valid]-cmd[index[valid],i])**2)))
        result[label] = val
    return result


def coupling_summary(streams, start, end, names):
    # Descriptive first-difference relationship on one controller source-time grid.
    ct, raw, _ = series(selected(streams['control'], start, end), 'raw_velocity', len(names)+2)
    et, est, _ = series(streams['target_estimate'], 'mean', 6)
    indices = causal_indices(et, ct, .15)
    valid = indices >= 0
    ct, raw, indices = ct[valid], raw[valid], indices[valid]
    est_step = np.linalg.norm(np.diff(est[indices,:3], axis=0), axis=1)
    command_step = abs(np.diff(raw[:,2+names.index('wrist_joint')]))
    result = {'method': 'controller-grid changes using latest nonfuture target estimate <=0.15s; '
              'step norm is NOT a Kalman correction or innovation',
              'sample_count': len(est_step),
              'estimate_position_step_vs_raw_wrist_step_Pearson':
              float(np.corrcoef(est_step, command_step)[0,1])
              if np.std(est_step)>0 and np.std(command_step)>0 else None,
              'causality_limit': 'correlation cannot separate robot state feedback, target dynamics, '
              'reference geometry, safety, or low-level plant effects'}
    return result


def plots(result, streams, prefix):
    names = result['arm_name_order_from_resolved_configuration']
    start, end = result['metadata_window']['window_start'], result['metadata_window']['window_end']
    fig, axes = plt.subplots(4, 2, figsize=(13, 9), sharex=True)
    for i, name in enumerate(names):
        t,q,v,_ = named_joint_series(selected(streams['joints'], start, end), name)
        q = q.copy(); v = v.copy()
        gaps = np.flatnonzero(np.diff(t)>GAP+1e-12)+1
        q[gaps] = np.nan; v[gaps] = np.nan
        axes[i,0].plot(t,q,lw=.8); axes[i,0].set_ylabel(name+'\nq (rad)')
        axes[i,1].plot(t,v,lw=.6);axes[i,1].set_ylabel('dq (rad/s)')
        lim=result['arm'][name]['speed_limit']['description_rad_s']
        for bound in [-lim,lim]:axes[i,1].axhline(bound,color='r',ls=':',lw=.7)
    axes[-1,0].set_xlabel('source time (s)');axes[-1,1].set_xlabel('source time (s)')
    fig.suptitle(Path(result['dataset']).name+' — physical feedback; red description limits')
    fig.tight_layout(); path=str(prefix)+'_joint_activity.png';fig.savefig(path,dpi=150);plt.close(fig)
    paths=[path]
    event=max((r for r in result['wrist_events'] if start<=r['peak_time_s']<=end),
              key=lambda r:abs(r['physical_peak_velocity_rad_s']))
    a,b=event['inspection_window_s'];fig,axes=plt.subplots(5,1,figsize=(13,12),sharex=True)
    t,q,v,_=named_joint_series(selected(streams['joints'],a,b),'wrist_joint')
    q = q.copy(); v = v.copy()
    gaps = np.flatnonzero(np.diff(t)>GAP+1e-12)+1
    q[gaps] = np.nan; v[gaps] = np.nan
    axes[0].plot(t,v,'.-',label='physical dq',color='black',markersize=2)
    for stream,field,label in [('control','raw_velocity','raw WLS'),
                               ('control','safe_velocity','safe'),('command','velocity','published')]:
        ts,vs,_=series(selected(streams[stream],a-.15,b),field,6)
        axes[0].step(ts,vs[:,5],where='post',label=label,alpha=.75)
    for bound in [-1.501,1.501]:axes[0].axhline(bound,color='r',ls=':',lw=.7)
    axes[0].legend(ncol=4);axes[0].set_ylabel('wrist rad/s')
    axes[1].plot(t,q,'.-',label='physical q',markersize=2)
    ts,vs,_=series(selected(streams['actuation'],a-.15,b),'position_reference',4)
    axes[1].scatter(ts,vs[:,3],label='reference snapshots',color='r',s=18)
    axes[1].legend();axes[1].set_ylabel('wrist rad')
    for stream,field,label in [('target_truth','position','truth'),
                               ('target_measurement','position','measurement'),
                               ('target_estimate','mean','estimate')]:
        subset=selected(streams[stream],a-.15,b)
        ts=np.array([r['source_time'] for r in subset]);vs=np.array([r['data'][field][:3] for r in subset])
        if len(subset):
            axes[2].plot(ts,vs[:,2],'.-',label=label+' z')
    if axes[2].lines:
        axes[2].legend()
    axes[2].set_ylabel('world z (m)')
    subset=selected(streams['prediction'],a-.15,b)
    axes[3].plot([r['source_time'] for r in subset],[r['data']['states'][0][2] for r in subset],
                 '.-',label='first predicted point z')
    axes[3].legend();axes[3].set_ylabel('world z (m)')
    subset=selected(streams['actuation'],a-.15,b)
    for field in ['feedback_rejections','timeout_events','reference_limit_events']:
        axes[4].step([r['source_time'] for r in subset],[r['data'][field] for r in subset],
                     where='post',label=field)
    axes[4].legend();axes[4].set_ylabel('counters');axes[4].set_xlabel('source time (s)')
    for ax in axes:ax.axvline(event['peak_time_s'],color='grey',ls='--');ax.set_xlim(a,b);ax.grid(alpha=.2)
    fig.suptitle(Path(result['dataset']).name+' — largest recorded wrist event; snapshots are sparse')
    fig.tight_layout();path=str(prefix)+'_wrist_event.png';fig.savefig(path,dpi=150);plt.close(fig)
    paths.append(path)
    fig,axes=plt.subplots(2,1,figsize=(12,6),sharex=True)
    records=selected(streams['base_truth'],start,end)
    times=[r['source_time'] for r in records]
    for axis,field,index,label in [(axes[0],'linear_velocity',0,'forward v (m/s)'),
                                  (axes[1],'angular_velocity',2,'yaw rate (rad/s)')]:
        axis.plot(times,[r['data'][field][index] for r in records],label='physical body twist')
        ts,values,_=series(selected(streams['command'],start,end),'velocity',6)
        component=0 if index==0 else 1
        axis.step(ts,values[:,component],where='post',label='published command',alpha=.7)
        axis.set_ylabel(label);axis.legend();axis.grid(alpha=.2)
    axes[-1].set_xlabel('source time (s)')
    fig.suptitle(Path(result['dataset']).name+' — base allocation and physical motion')
    fig.tight_layout();path=str(prefix)+'_base_allocation.png'
    fig.savefig(path,dpi=150);plt.close(fig);paths.append(path)
    return [str(Path(p).relative_to(ROOT)) if Path(p).is_relative_to(ROOT)
            else str(Path(p)) for p in paths]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-prefix',default='results/m09_joint_diagnostics/diagnostics')
    args=parser.parse_args()
    prefix=ROOT/args.output_prefix
    if prefix.with_suffix('.json').exists() or list(prefix.parent.glob(prefix.name+'_*')):
        parser.error('Output prefix already exists; choose a fresh prefix to preserve evidence')
    prefix.parent.mkdir(parents=True,exist_ok=True)
    runs=sorted(ROOT.glob('results/experiments/m08_verified_final/*/runs/*'))+sorted(
        ROOT.glob('results/experiments/m08_final_seed917/*/runs/*'))
    output={'policy': {'feedback_integration': 'trapezoid v^2 over adjacent intervals <=0.01s; '
                       'no interpolation/zero fill across gaps or window edges',
                       'violation': '|dq| > description limit + unchanged 0.001 rad/s; '
                       'duration is left-held on eligible intervals only',
                       'safety_scope': 'all startup samples additionally reported in capture_arm; '
                       'evaluation metrics use each recorded six-second metadata window'},
            'runs': []}
    for run in runs:
        result,streams=summarize(run)
        result['plots']=plots(result,streams,Path(str(prefix)+'_'+run.name))
        output['runs'].append(result)
        print(run.name,'wrist',result['arm']['wrist_joint']['peak_abs_velocity_rad_s'],
              'coverage',result['arm']['wrist_joint']['coverage_fraction'])
    path=Path(str(prefix)+'.json')
    path.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    print(path)


if __name__ == '__main__':
    main()
