"""Summarize preserved M03 production captures without changing or running the robot."""

from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from m02_metrics import audit_joints


HERE = Path(__file__).resolve().parent


def inactive_intervals(samples, start, end):
    """Estimate inactive intervals by holding each telemetry flag until the next stamp."""
    runs = []
    for index, row in enumerate(samples):
        left = max(start, row[0])
        right = min(end, samples[index+1][0] if index+1 < len(samples) else row[0])
        if row[1] or right <= left:
            continue
        if runs and abs(runs[-1][1]-left) < 1e-9:
            runs[-1][1] = right
        else:
            runs.append([left, right])
    return dict(intervals_s=runs, duration_s=sum(b-a for a, b in runs),
                longest_s=max((b-a for a, b in runs), default=0.))


def summarize(name):
    """Keep all physical tracking samples separate from valid-control-only timing."""
    directory = HERE / name
    result = json.loads((directory / 'result.json').read_text())
    streams = json.loads((directory / 'experiment_streams.json').read_text())
    snapshot = yaml.safe_load((directory / 'configuration_snapshot.yaml').read_text())
    estimator = json.loads((directory / 'm03_metrics.json').read_text())
    origin = result['target_initialization_time']
    start = origin + result['thresholds']['settling_time']
    end = origin + result['thresholds']['duration']
    rows = [json.loads(line) for line in (directory / 'samples.jsonl').read_text().splitlines()]
    rows = [r for r in rows if start <= r['time'] <= end]
    milliseconds = np.array([r['compute_time']*1000 for r in rows])
    limits = snapshot['description']['manipulator']['joints']
    physical = audit_joints([j for j in streams['joints'] if start <= j[0] <= end],
                            limits, result['thresholds']['physical_numerical_tolerance'])
    sdf = ET.parse(directory / 'generated_world.sdf')
    joints = {j.get('name'): j for j in sdf.findall('.//joint')}
    for name, limit in zip(snapshot['joint_order'], limits.values()):
        for field in ['lower', 'upper', 'velocity', 'effort']:
            assert float(joints[name].findtext('axis/limit/'+field)) == limit[field]
        np.testing.assert_array_equal(
            [float(v) for v in joints[name].findtext('axis/xyz').split()], limit['axis'])
    plugins = {p.findtext('joint_name'): p for p in sdf.findall('.//plugin')
               if p.get('name') == 'gz::sim::systems::JointPositionController'}
    for key, name in zip(limits, snapshot['joint_order']):
        plugin = plugins[name]
        assert plugin.findtext('use_velocity_commands') == 'false'
        assert float(plugin.findtext('cmd_min')) == -limits[key]['effort']
        assert float(plugin.findtext('cmd_max')) == limits[key]['effort']
        pid = dict(snapshot['holding']['pid'], **snapshot['holding'].get(
            'joint_overrides', {}).get(key, {}).get('pid', {}))
        assert all(float(plugin.findtext(k)) == v for k, v in pid.items())
    control = [c for c in streams['control'] if start <= c[0] <= end]
    actuation = [a for a in streams['actuation'] if start <= a[0] <= end]
    period_ms = 1000 / snapshot['control']['validation_rate']
    evidence_files = ['result.json', 'configuration_snapshot.yaml', 'samples.jsonl',
                      'experiment_streams.json', 'generated_world.sdf', 'm03_metrics.json']
    return dict(
        evidence=str(directory.relative_to(HERE.parent.parent)),
        input_sha256={f: hashlib.sha256((directory/f).read_bytes()).hexdigest()
                      for f in evidence_files},
        window_sim_s=[start, end], mean_rtf=result['mean_rtf'],
        control_compute_ms=dict(mean=float(milliseconds.mean()),
            p95=float(np.quantile(milliseconds, .95)), maximum=float(milliseconds.max()),
            samples=len(rows), nominal_period_ms=period_ms,
            samples_above_wall_period=int((milliseconds > period_ms).sum())),
        physical_tracking={k: result['experiment'][k] for k in [
            'physical_tracking_samples', 'mean_physical_distance_error',
            'max_physical_distance_error', 'mean_physical_pointing_degrees',
            'max_physical_pointing_degrees', 'mean_target_speed', 'max_target_speed']},
        physical_joint_audit=physical,
        configured_joint_velocity_limits_rad_s={k: v['velocity'] for k, v in limits.items()},
        control_validity=dict(samples=len(control), invalid=sum(not c[1] for c in control),
            reasons=dict(Counter(c[2] for c in control if not c[1])),
            inactive=inactive_intervals(streams['control'], start, end)),
        actuation_validity=dict(samples=len(actuation), inactive_samples=sum(not a[3] for a in actuation),
            inactive=inactive_intervals([[a[0], a[3]] for a in streams['actuation']], start, end),
            observed_recovery_counter_max=max([int(n) for a in streams['actuation']
                for n in re.findall(r'events=(\d+)', a[4])] or [0]),
            final_active=streams['actuation'][-1][3]),
        watchdog_timeouts_after_settling=result['watchdog_timeouts_after_settling'],
        actuator_counters_final=result['counters'], observer_checks=result['checks'],
        source_time_estimation_errors=estimator['errors'], estimation_timing=estimator['timing'],
        mean_process_cpu_percent=result['mean_process_cpu_percent'],
        memory_usage=None,
        memory_note='RSS/PSS was not captured. No peak or memory-trend claim is available.',
        generated_physical_limits_match_description=True,
        generated_effort_pid_matches_holding_configuration=True,
        launch=result['launch'])


if __name__ == '__main__':
    output = {name: summarize(name) for name in ['m03_nominal_after', 'm03_wide_final']}
    path = HERE / 'm04_m05_baseline.json'
    path.write_text(json.dumps(output, indent=2)+'\n')
    for name, measurements in output.items():
        print(name, json.dumps({key: measurements[key] for key in [
            'mean_rtf', 'control_compute_ms', 'physical_tracking',
            'control_validity', 'actuation_validity']}))
