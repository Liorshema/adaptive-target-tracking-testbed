"""Audit raw physical safety, including inactive intervals and startup samples."""

import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from shoulder_experiment_metrics import experiment_metrics


def audit_joints(samples, limits, tolerance=.001):
    """Use every delivered sample; durations use left-held measured flags."""
    times = np.array([s[0] for s in samples])
    q, dq = np.array([s[1] for s in samples]), np.array([s[2] for s in samples])
    dt = np.maximum(np.diff(times, append=times[-1]), 0.)
    gap = float(dt.max())
    result = {}
    for i, (name, limit) in enumerate(limits.items()):
        excess = np.maximum(np.maximum(limit['lower'] - q[:, i], q[:, i] - limit['upper']), 0.)
        speed_bad = np.abs(dq[:, i]) > limit['velocity'] + tolerance
        strict_bad = excess > 0.
        accepted_bad = excess > tolerance
        result[name] = dict(
            minimum=float(q[:, i].min()), maximum=float(q[:, i].max()),
            max_position_excess_rad=float(excess.max()),
            position_excess_time_estimate_s=float(dt[strict_bad].sum()),
            position_outside_acceptance_time_estimate_s=float(dt[accepted_bad].sum()),
            peak_speed_rad_s=float(np.abs(dq[:, i]).max()),
            speed_violation_samples=int(speed_bad.sum()),
            speed_violation_time_estimate_s=float(dt[speed_bad].sum()),
            first_position_violation_time=(float(times[np.flatnonzero(strict_bad)[0]])
                                           if strict_bad.any() else None),
            finite_difference_peak_rad_s=float(np.max(np.abs(np.diff(q[:, i])) /
                                              np.maximum(np.diff(times), 1e-15))))
    return dict(joints=result, samples=len(samples), max_sample_gap_s=gap)


def analyze(path):
    """Compute physical errors even after Control becomes permanently inactive."""
    snapshot = yaml.safe_load((path / 'configuration_snapshot.yaml').read_text())
    streams = json.loads((path / 'experiment_streams.json').read_text())
    thresholds = yaml.safe_load(Path(__file__).with_name('v2_thresholds.yaml').read_text())
    launch_text = (path / 'observed.launch.py').read_text() if (path / 'observed.launch.py').exists() else ''
    match = re.search(r'Simulation initialized at t=([\d.]+)s', (path / 'runtime.log').read_text())
    origin = float(match.group(1))
    thresholds['duration'] = streams['joints'][-1][0] - origin
    limits = snapshot['description']['manipulator']['joints']
    result = audit_joints(streams['joints'], limits, thresholds['physical_numerical_tolerance'])
    result['settled'] = audit_joints([s for s in streams['joints'] if s[0] >= origin +
                                     thresholds['settling_time']], limits,
                                    thresholds['physical_numerical_tolerance'])
    result['origin'] = origin
    result['last_joint_time'] = streams['joints'][-1][0]
    result['physical'] = experiment_metrics(path, streams, snapshot['description'],
        snapshot['robot'], origin, thresholds, snapshot['control'])
    control = streams.get('control', [])
    invalid = [s for s in control if not s[1]]
    position_invalid = [s for s in invalid if s[2].startswith('Current joint positions violate limits')]
    result['control'] = dict(samples=len(control), invalid_samples=len(invalid),
        position_invalid_samples=len(position_invalid),
        first_position_invalid=position_invalid[0] if position_invalid else None,
        last_position_invalid=position_invalid[-1] if position_invalid else None,
        final=control[-1] if control else None)
    rtf = [v for phase, v in json.loads((path / 'rtf.json').read_text()) if phase >= 10]
    result['mean_rtf'] = float(np.mean(rtf))
    trace = json.loads((path / 'actuation_trace.json').read_text())
    references = [s for s in trace if s['reference'] is not None]
    refs = np.array([s['reference'] for s in references])
    times = np.array([s['time'] for s in references])
    good_dt = np.diff(times) > 0
    slopes = np.abs(np.diff(refs, axis=0)[good_dt] / np.diff(times)[good_dt, None])
    result['reference'] = dict(minimum=refs.min(axis=0).tolist(), maximum=refs.max(axis=0).tolist(),
        peak_slope_rad_s=slopes.max(axis=0).tolist(),
        max_measured_lead_rad=np.max([np.abs(np.array(s['reference']) - s['measured'])
            for s in references if s['measured'] is not None], axis=0).tolist())
    repeated_time = np.diff(times) == 0
    result['reference']['max_zero_dt_step_rad'] = (
        np.abs(np.diff(refs, axis=0)[repeated_time]).max(axis=0).tolist()
        if repeated_time.any() else [0.] * len(limits))
    holding = snapshot.get('holding')
    if holding is None:
        holding = yaml.safe_load((Path(__file__).resolve().parents[2] /
            'src/robot_simulation/config/gazebo/joint_position_hold.yaml').read_text())[
                'joint_position_hold']
    initial_reference = [holding['positions'][name] for name in limits]
    result['reference']['initial_handoff_step_rad'] = (
        refs[0] - initial_reference).tolist()
    active = [s for s in trace if s['active'] and s['joint_time'] is not None and
              s['command_time'] is not None]
    result['freshness'] = dict(
        max_active_feedback_age_s=max((s['time'] - s['joint_time'] for s in active), default=None),
        max_active_command_age_s=max((s['time'] - s['command_time'] for s in active), default=None))
    result['recovery_events'] = max(s.get('recovery_events', 0) for s in trace)
    trace_times = np.array([s['time'] for s in trace])
    joint_times = np.array([s[0] for s in streams['joints']])
    previous = np.searchsorted(trace_times, joint_times, side='right') - 1
    recovery_samples = [sample for index, sample in zip(previous, streams['joints'])
                        if index >= 0 and trace[index].get('position_mode') in
                        ['numerical_recovery', 'recovery']]
    result['recovery_peak_measured_speed_rad_s'] = (
        np.abs([s[2] for s in recovery_samples]).max(axis=0).tolist()
        if recovery_samples else [])
    faults = [s for s in trace if s.get('position_mode') == 'fault']
    result['fault'] = faults[0] if faults else None
    result['actuation_active_fraction'] = float(np.mean([s['active'] for s in trace]))
    result['after_last_position_invalid_active_samples'] = sum(
        s['active'] and s['time'] > position_invalid[-1][0] for s in trace) if position_invalid else None
    result['before_m02_adapter'] = 'M02_BASELINE_ACTUATION' in launch_text or any(
        s.get('position_mode') is None for s in trace)
    result['physical_speed_pass'] = all(j['speed_violation_samples'] == 0
                                        for j in result['joints'].values())
    generated = path / 'generated_world.sdf'
    if not generated.exists():
        match = re.search(r'(/tmp/robot_simulation_[^ /]+/world\.sdf)',
                          (path / 'runtime.log').read_text())
        if match and Path(match.group(1)).is_file():
            generated.write_text(Path(match.group(1)).read_text())
    if generated.exists():
        world = ET.parse(generated).getroot()
        joints = {j.get('name'): j for j in world.findall('.//joint')}
        audited = {}
        for name, physical in zip(snapshot['joint_order'], limits.values()):
            actual = joints[name]
            audited[name] = {field: float(actual.findtext('axis/limit/' + field))
                             for field in ['lower', 'upper', 'velocity', 'effort']}
            assert all(audited[name][field] == physical[field] for field in audited[name])
            assert np.array_equal([float(x) for x in actual.findtext('axis/xyz').split()],
                                  physical['axis'])
        controllers = {p.findtext('joint_name'): p for p in world.findall('.//plugin')
                       if p.get('name') == 'gz::sim::systems::JointPositionController'}
        for key, name in zip(limits, snapshot['joint_order']):
            plugin = controllers[name]
            physical = limits[key]
            assert plugin.findtext('use_velocity_commands') == 'false'
            assert float(plugin.findtext('cmd_min')) == -physical['effort']
            assert float(plugin.findtext('cmd_max')) == physical['effort']
            pid = dict(holding['pid'], **holding.get('joint_overrides', {}).get(
                key, {}).get('pid', {}))
            for parameter, value in pid.items():
                assert float(plugin.findtext(parameter)) == value
        result['generated_limits'] = audited
        result['generated_pid_and_effort_configuration_pass'] = True
        result['physics_step_s'] = float(world.findtext('world/physics/max_step_size'))
    (path / 'm02_metrics.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('outputs', nargs='+', type=Path)
    args = parser.parse_args()
    for output in args.outputs:
        result = analyze(output)
        print(output, json.dumps({k: result[k] for k in ['mean_rtf', 'recovery_events',
            'control', 'reference', 'physical_speed_pass']}))
