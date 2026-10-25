"""Timestamp-aligned estimator/localization audits; ground truth is observation only."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def planar(samples, spawn=None):
    """Express physical world poses in the configured initial odom frame."""
    times = np.array([s[0] for s in samples])
    positions = np.array([s[1] for s in samples])
    rotations = Rotation.from_quat([s[2] for s in samples])
    if spawn is not None:
        initial = Rotation.from_euler('xyz', spawn[3:])
        positions = initial.inv().apply(positions - spawn[:3])
        rotations = initial.inv() * rotations
    yaw = np.unwrap(rotations.as_euler('xyz')[:, 2])
    return times, np.column_stack([positions[:, :2], yaw])


def interpolate(times, values, at):
    """Offline interpolation is restricted to bracketed timestamps by callers."""
    return np.column_stack([np.interp(at, times, values[:, i]) for i in range(values.shape[1])])


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def estimate_errors(estimates, streams, snapshot, start, end):
    """Compare errors and covariance at state timestamps, not publication receipt."""
    tg, truth = planar(streams['physical_base'], np.array(snapshot['robot']['pose']))
    to, odom = planar(streams['odometry'])
    tj = np.array([s[0] for s in streams['joints']])
    qj = np.array([s[1] for s in streams['joints']])
    n = qj.shape[1]
    selected = [e for e in estimates if max(start, tg[0], to[0], tj[0]) <= e[0] <=
                min(end, tg[-1], to[-1], tj[-1])]
    if not selected:
        raise ValueError('No bracketed estimates to compare')
    t = np.array([e[0] for e in selected])
    mean = np.array([e[1] for e in selected])
    actual = interpolate(tg, truth, t)
    wheel = interpolate(to, odom, t)
    joints = interpolate(tj, qj, t)
    base_error = mean[:, :3] - actual
    base_error[:, 2] = wrap(base_error[:, 2])
    wheel_error = wheel - actual
    wheel_error[:, 2] = wrap(wheel_error[:, 2])
    q_error = mean[:, 5:5+n] - joints
    physical = list(snapshot['description']['manipulator']['joints'].values())
    lower = np.array([j['lower'] for j in physical])
    upper = np.array([j['upper'] for j in physical])
    excess = np.maximum(np.maximum(lower - mean[:, 5:5+n], mean[:, 5:5+n] - upper), 0.)
    result = dict(samples=len(selected),
        mean_base_position_error_m=float(np.linalg.norm(base_error[:, :2], axis=1).mean()),
        max_base_position_error_m=float(np.linalg.norm(base_error[:, :2], axis=1).max()),
        mean_abs_yaw_error_deg=float(np.degrees(np.abs(base_error[:, 2])).mean()),
        max_abs_yaw_error_deg=float(np.degrees(np.abs(base_error[:, 2])).max()),
        mean_wheel_position_error_m=float(np.linalg.norm(wheel_error[:, :2], axis=1).mean()),
        mean_wheel_yaw_error_deg=float(np.degrees(np.abs(wheel_error[:, 2])).mean()),
        joint_rmse_rad=np.sqrt(np.mean(q_error**2, axis=0)).tolist(),
        joint_max_error_rad=np.abs(q_error).max(axis=0).tolist(),
        estimated_max_joint_excess_rad=excess.max(axis=0).tolist(),
        estimated_outside_samples=int(np.any(excess > 0, axis=1).sum()),
        mean_estimate_vs_wheel_position_m=float(np.linalg.norm(mean[:, :2]-wheel[:, :2], axis=1).mean()),
        mean_estimate_vs_wheel_yaw_deg=float(np.degrees(np.abs(wrap(mean[:, 2]-wheel[:, 2]))).mean()),
        max_state_stamp_gap_s=float(np.diff(t).max()) if len(t)>1 else None)
    if len(selected[0]) > 2:
        p = np.array([e[2] for e in selected]).reshape(len(selected), mean.shape[1], mean.shape[1])
        nees_base = np.array([e @ np.linalg.solve(c[:3, :3], e)
                              for e, c in zip(base_error, p)])
        nees_q = np.array([e @ np.linalg.solve(c[5:5+n, 5:5+n], e)
                           for e, c in zip(q_error, p)])
        result['covariance'] = dict(
            min_eigenvalue=float(np.linalg.eigvalsh(p).min()),
            max_asymmetry=float(np.abs(p-p.transpose(0,2,1)).max()),
            mean_base_nees=float(nees_base.mean()), mean_joint_nees=float(nees_q.mean()),
            fraction_base_nees_below_chi2_95=float(np.mean(nees_base <= 7.8147279)),
            fraction_joint_nees_below_chi2_95=float(np.mean(nees_q <= 9.4877290)))
    return result


def timing(path, streams):
    """Separate transport arrival, buffering and actual causal fusion commits."""
    events = json.loads((path/'processing.json').read_text())
    commits = json.loads((path/'processing_commits.json').read_text())
    final_path = path/'estimator_final_diagnostics.json'
    final = json.loads(final_path.read_text()) if final_path.exists() else None
    diagnostics = streams.get('estimation_diagnostics', [])
    queue = final.get('timing') if final else diagnostics[-1]['timing'] if diagnostics else None
    result = {}
    for source in ['wheel','joints','imu']:
        rows = [e for e in events if e['source']==source]
        if not rows:
            result[source] = dict(received=0)
            continue
        stamps = np.array([e['stamp'] for e in rows])
        age = np.array([e['now']-e['stamp'] for e in rows])
        positive = np.diff(np.unique(stamps)); positive = positive[positive>0]
        if queue is None:
            received, rejected = len(rows), sum(e['rejected'] for e in rows)
            counters = dict(received=received, accepted=received-rejected,
                            rejected=rejected, pending=0)
        else:
            counters = dict(queue['sources'].get(source, {}))
            counters['rejected'] = sum(counters.get(k,0) for k in
                ['late','stale','future','duplicate','invalid','overflow'])
            counters['pending'] = counters.get('received',0)-counters.get('accepted',0)-counters['rejected']
        counters.update(rejection_fraction=counters['rejected']/counters['received'],
            observed_callback_hz=len(rows)/max(stamps.max()-stamps.min(),1e-9),
            median_source_period_s=float(np.median(positive)) if len(positive) else None,
            arrival_age_p50_s=float(np.median(age)), arrival_age_p95_s=float(np.quantile(age,.95)),
            arrival_age_max_s=float(age.max()),
            source_stamp_regressions=int(np.sum(np.diff(stamps)<0)))
        result[source] = counters
    accepted = [c for c in commits if c['accepted']]
    latencies = [c['now']-c['stamp'] for c in accepted]
    result['commits'] = dict(samples=len(accepted),
        backward_steps=sum(c['previous'] is not None and c['stamp']<c['previous'] for c in accepted),
        state_age_p50_s=float(np.median(latencies)), state_age_p95_s=float(np.quantile(latencies,.95)),
        state_age_max_s=float(np.max(latencies)))
    result['final_diagnostics'] = final or (diagnostics[-1] if diagnostics else None)
    return result


def drive_phases(streams, snapshot, origin):
    """Physical/odometric increment errors identify skid without tuning geometry."""
    tg, ground = planar(streams['physical_base'], np.array(snapshot['robot']['pose']))
    to, wheel = planar(streams['odometry'])
    result, elapsed = {}, 0.
    for segment in snapshot['drive_fixture']['segments']:
        start, end = origin+elapsed, origin+elapsed+segment['seconds']
        elapsed += segment['seconds']
        mask = (tg>=start+.1)&(tg<=end-.1)&(tg>=to[0])&(tg<=to[-1])
        t,g = tg[mask],ground[mask]
        o=interpolate(to,wheel,t)
        relative_g,relative_o=g-g[0],o-o[0]
        # Compare phase increments in the common odom orientation, retaining yaw bias separately.
        result[segment['name']]=dict(
            physical_displacement_m=float(np.linalg.norm(relative_g[-1,:2])),
            odom_displacement_m=float(np.linalg.norm(relative_o[-1,:2])),
            physical_yaw_change_rad=float(relative_g[-1,2]),
            odom_yaw_change_rad=float(relative_o[-1,2]),
            endpoint_increment_position_error_m=float(np.linalg.norm(relative_g[-1,:2]-relative_o[-1,:2])),
            endpoint_yaw_increment_error_deg=float(np.degrees(abs(wrap(relative_g[-1,2]-relative_o[-1,2])))))
        steady = t >= start + snapshot['drive_fixture']['ramp_seconds'] + .1
        sg, so = g[steady], o[steady]
        dg, do = sg[-1, :2]-sg[0, :2], so[-1, :2]-so[0, :2]
        def local(vector, yaw):
            return Rotation.from_euler('z', -yaw).apply([*vector, 0.])[:2]
        result[segment['name']]['after_ramp'] = dict(
            physical_displacement_m=float(np.linalg.norm(dg)),
            odom_displacement_m=float(np.linalg.norm(do)),
            physical_yaw_change_rad=float(sg[-1, 2]-sg[0, 2]),
            odom_yaw_change_rad=float(so[-1, 2]-so[0, 2]),
            local_endpoint_position_error_m=float(np.linalg.norm(
                local(dg, sg[0, 2])-local(do, so[0, 2]))))
    return result


def analyze(path):
    streams=json.loads((path/'experiment_streams.json').read_text())
    snapshot=yaml.safe_load((path/'configuration_snapshot.yaml').read_text())
    base=json.loads((path/'result.json').read_text())
    start=base['target_initialization_time']+base['thresholds']['settling_time']
    end=base['target_initialization_time']+base['thresholds']['duration']
    result=dict(origin=base['target_initialization_time'],mean_rtf=base['mean_rtf'],
        mean_cpu=base['mean_process_cpu_percent'],
        errors=estimate_errors(streams['robot_estimates'],streams,snapshot,start,end),
        timing=timing(path,streams), physical_tracking=base.get('experiment'),
        observer_checks=base['checks'],observed_topic_rates=base['observed_topic_rates_sim_hz'])
    raw=np.array([s[1] for s in streams['joints']])
    limits=list(snapshot['description']['manipulator']['joints'].values())
    excess=np.maximum(np.maximum(np.array([j['lower'] for j in limits])-raw,
        raw-np.array([j['upper'] for j in limits])),0.)
    result['raw_max_joint_excess_rad']=excess.max(axis=0).tolist()
    result['raw_physical_excursion_samples']=int(np.any(excess>1e-9,axis=1).sum())
    if 'drive_fixture' in snapshot:
        result['drive_phases']=drive_phases(streams,snapshot,base['target_initialization_time'])
    (path/'m03_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('outputs',nargs='+',type=Path)
    args=parser.parse_args()
    for path in args.outputs:
        result=analyze(path)
        print(path,json.dumps(dict(errors=result['errors'],rtf=result['mean_rtf'])))
