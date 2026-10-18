"""Pure source-time experiment metrics, version 1.

Errors use equally weighted valid samples, rather than resampling trajectories.
Safety uses unsmoothed raw joint samples; durations use right holds bounded by
max_skew and the evaluation end. Missing intervals remain explicitly uncovered.
"""

import math

import numpy as np

from robot_bringup.factory.model_factory import create_robot_model

from robot_description.model_parameters import robot_model_parameters_from_description

from robot_models.spatial.rotations import rpy_to_rotation
from robot_models.whole_body.kinematics import WholeBodyKinematics

from .alignment import SourceIndex, finite_vector, prepare, statistics


_STREAMS = ('target_truth', 'target_measurement', 'joints', 'base_truth', 'odometry',
            'robot_estimate', 'target_estimate', 'prediction', 'control', 'command',
            'actuation', 'clock')
_INVALID = (ValueError, TypeError, KeyError, IndexError, OverflowError)


def _transform(data):
    position = finite_vector(data['position'], 3)
    quaternion = finite_vector(data['orientation'], 4)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12 or abs(norm - 1.) > .01:
        raise ValueError('invalid orientation quaternion norm')
    x, y, z, w = quaternion / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = position
    return result


def _ordered(data, names, field):
    actual = data['names']
    if len(actual) != len(set(actual)):
        raise ValueError('duplicate joint names')
    values = finite_vector(data[field], len(actual))
    return values[[actual.index(name) for name in names]]


def _append(series, name, time, value, **extra):
    series[name]['samples'].append({'time': float(time), 'value': float(value), **extra})


def _series_metrics(series, name,
                    reason='missing, stale, nonfinite, or incompatible physical truth'):
    entry = series[name]
    return statistics([sample['value'] for sample in entry['samples']], entry['unit'], reason)


def _safety(records, names, parameters, start, end, skew, series, tolerance):
    metrics = {}
    window_duration = end - start
    before = [record for record in records if record['source_time'] < start]
    candidates = [record for record in records if start <= record['source_time'] <= end]
    if before:
        last_time = before[-1]['source_time']
        if last_time + skew > start:
            candidates = [record for record in before if record['source_time'] == last_time] + (
                candidates)
    for field, name, unit in [('position', 'joint_position_violation', 'rad'),
                              ('velocity', 'joint_speed_violation', 'rad/s')]:
        samples = []
        for record in candidates:
            try:
                values = _ordered(record['data'], names, field)
                if field == 'position':
                    overshoot = np.maximum(0., np.maximum(parameters.lower - values,
                                                          values - parameters.upper))
                else:
                    overshoot = np.maximum(0., np.abs(values) - parameters.velocity)
                samples.append((record['source_time'], float(np.max(overshoot))))
                _append(series, name, max(start, record['source_time']), float(np.max(overshoot)),
                        source_time=record['source_time'],
                        per_joint=overshoot.tolist(), measured=values.tolist())
            except _INVALID:
                continue
        metric = statistics([sample[1] for sample in samples], unit,
                            f'no valid physical joint {field} samples in authoritative name order')
        durations = [
            max(0., min(time + skew, end,
                samples[i + 1][0] if i + 1 < len(samples) else end) - max(start, time))
            for i, (time, _) in enumerate(samples)]
        observed_duration = min(window_duration, sum(durations))
        gaps = np.diff([start] + [max(start, time) for time, _ in samples] + [end])
        coverage_complete = window_duration - observed_duration <= max(
            1e-9, 1e-9 * window_duration)
        metric.update(violation_samples=sum(value > tolerance for _, value in samples),
                      violation_duration=sum(dt for dt, (_, value) in zip(durations, samples)
                                             if value > tolerance),
                      observed_duration=observed_duration, duration_unit='s',
                      window_duration=window_duration,
                      coverage_fraction=observed_duration / window_duration,
                      max_gap=float(np.max(gaps)), coverage_complete=coverage_complete,
                      duration_method='right hold capped by max_skew; no gap extrapolation',
                      tolerance=tolerance, tolerance_unit=unit,
                      max_overshoot=max((value for _, value in samples), default=None),
                      rejected_samples=len(candidates) - len(samples))
        metrics[name] = metric
    available = all(metric['available'] for metric in metrics.values())
    failed = [name for name, metric in metrics.items() if metric['violation_samples']]
    complete = all(metric['coverage_complete'] for metric in metrics.values())
    observations_accepted = False if failed else (True if available else None)
    accepted = False if failed else (True if available and complete else None)
    coverage_fields = ['observed_duration', 'window_duration', 'coverage_fraction',
                       'max_gap', 'coverage_complete', 'duration_unit']
    return metrics, {'accepted': accepted,
                     'observations_accepted': observations_accepted,
                     'coverage_complete': complete,
                     'coverage': {name: {key: metric[key] for key in coverage_fields}
                                  for name, metric in metrics.items()},
                     'failed_checks': failed,
                     'unavailable_checks': [name for name, metric in metrics.items()
                                            if not metric['available']],
                     'policy': ('physical position and speed gates; '
                                'missing or incompletely covered evidence is unknown')}


def _predictions(records, truth, end, skew, series):
    horizons, rejected = {}, 0
    for record in records:
        try:
            times, states = record['data']['timestamps'], record['data']['states']
            if record.get('frame_id') != truth.frame:
                raise ValueError('incompatible prediction frame')
            if len(times) != len(states) or not times:
                raise ValueError('incompatible prediction layout')
        except _INVALID:
            rejected += 1
            continue
        previous = record['source_time']
        for time, state in zip(times, states):
            try:
                time = float(time)
                if not math.isfinite(time) or time <= previous or time > end:
                    raise ValueError('prediction must increase after issuance within window')
                previous = time
                position = finite_vector(state, 6)[:3]
                actual = truth.interpolate_position(time, skew)
                error = float(np.linalg.norm(position - actual))
                # Header epochs are quantized to nanoseconds while future
                # stamps remain doubles. Group nominal horizons at 0.1 us
                # reporting resolution without changing alignment timestamps.
                horizon = round(time - record['source_time'], 7)
                horizons.setdefault(str(horizon), []).append(error)
                _append(series, 'prediction_error', time, error, horizon=horizon,
                        source_time=record['source_time'])
            except _INVALID:
                rejected += 1
    metric = _series_metrics(series, 'prediction_error')
    metric.update(horizons={key: statistics(values, 'm') for key, values in sorted(
        horizons.items(), key=lambda item: float(item[0]))}, rejected_samples=rejected,
        horizon_unit='s', horizon_reporting_resolution=1e-7,
        alignment='absolute future timestamp; bounded bracket interpolation')
    return metric


def _counter(records, name, start, end):
    samples = []
    for record in records:
        value = record['data'].get(name)
        if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
            samples.append((record['source_time'], value))
    before = [sample for sample in samples if sample[0] <= start]
    within = [sample for sample in samples if start < sample[0] <= end]
    relevant = (before[-1:] if before else within[:1]) + (within if before else within[1:])
    # The first observed cumulative count is a baseline, never a window event.
    count, resets = 0, 0
    for (_, first), (_, last) in zip(relevant, relevant[1:]):
        if last < first:
            resets += 1
        else:
            count += last - first
    available = len(relevant) >= 2 and not resets
    return {'available': available, 'unit': 'count', 'sample_count': len(relevant),
            'count': count if available else None, 'counter_resets': resets,
            'reason': None if available else 'insufficient counter baseline or counter reset',
            'method': 'cumulative deltas between observed baseline and final sample'}


def evaluate(streams: dict[str, list[dict]], context: dict, start: float, end: float,
             max_skew: float = .08) -> dict:
    """Evaluate a closed simulation-source-time window using physical truth.

    ``robot_estimate`` uses the documented [x,y,yaw,v,w,q...,dq...] layout.
    Physical base pose is transformed into the configured initial-pose odometry
    frame before robot errors are calculated. Target state/prediction positions
    require exact world frame IDs; no implicit frame conversion is performed.
    """
    if not all(math.isfinite(value) for value in [start, end, max_skew]) or (
            start >= end or max_skew <= 0):
        raise ValueError('finite increasing evaluation window and positive max_skew required')
    all_records, selected, stream_stats = {}, {}, {}
    for name in sorted(set(_STREAMS) | set(streams)):
        all_records[name], selected[name], stream_stats[name] = prepare(
            streams.get(name, []), start, end, max_skew, preserve_duplicates=(name == 'joints'))
    units = {'tracking_distance': 'm', 'pointing_error': 'rad', 'prediction_error': 'm',
             'target_position_error': 'm', 'robot_position_error': 'm',
             'robot_yaw_error': 'rad', 'robot_joint_error': 'rad',
             'joint_position_violation': 'rad', 'joint_speed_violation': 'rad/s',
             'control_compute_time': 's', 'realtime_factor': '1'}
    series = {name: {'unit': unit, 'time_unit': 's', 'time_basis': 'simulation source time',
                     'samples': []} for name, unit in units.items()}
    metrics = {}
    names = list(context['joint_names'])
    parameters = robot_model_parameters_from_description(context['description'])
    model = create_robot_model(parameters=parameters)
    joint_index = SourceIndex(selected['joints'])
    base_index = SourceIndex(selected['base_truth'], context['world_frame'])
    truth = SourceIndex(selected['target_truth'], context['world_frame'])
    control = context['control']
    axis = finite_vector(control['tracking_forward_axis'], 3)
    if np.linalg.norm(axis) < 1e-12:
        raise ValueError('tracking forward axis must be nonzero')
    radius = float(control['tracking_radius'])
    if not math.isfinite(radius) or radius < 0:
        raise ValueError('tracking radius must be finite and nonnegative')
    for record in truth.records:
        time = record['source_time']
        try:
            joint = joint_index.nearest(time, max_skew)
            base = base_index.nearest(time, max_skew)
            q = _ordered(joint['data'], names, 'position')
            pose = WholeBodyKinematics.compute(model, _transform(base['data']), q)
            relative = finite_vector(record['data']['position'], 3) - pose[:3, 3]
            distance = float(np.linalg.norm(relative))
            _append(series, 'tracking_distance', time, abs(distance - radius))
            if distance > 1e-12:
                optical = pose[:3, :3] @ axis
                angle = float(np.arccos(np.clip(optical @ relative / (
                    np.linalg.norm(optical) * distance), -1., 1.)))
                _append(series, 'pointing_error', time, angle)
        except _INVALID:
            continue
    pose = finite_vector(context['robot']['pose'], 6)
    initial = np.eye(4)
    initial[:3, :3] = rpy_to_rotation(pose[3:])
    initial[:3, 3] = pose[:3]
    for record in selected['robot_estimate']:
        try:
            frame = context['estimator_frame']
            if not frame or record.get('frame_id') != frame:
                raise ValueError('incompatible robot estimate frame')
            state = finite_vector(record['data']['mean'], 5 + 2 * len(names))
            time = record['source_time']
        except _INVALID:
            continue
        try:
            physical_base = _transform(base_index.nearest(time, max_skew)['data'])
            actual = np.linalg.inv(initial) @ physical_base
            _append(series, 'robot_position_error', time,
                    np.linalg.norm(state[:2] - actual[:2, 3]))
            yaw = math.atan2(actual[1, 0], actual[0, 0])
            _append(series, 'robot_yaw_error', time, abs(math.atan2(
                math.sin(state[2] - yaw), math.cos(state[2] - yaw))))
        except _INVALID:
            pass
        try:
            joint = joint_index.nearest(time, max_skew)
            delta = state[5:5 + len(names)] - _ordered(joint['data'], names, 'position')
            _append(series, 'robot_joint_error', time, np.linalg.norm(delta))
        except _INVALID:
            continue
    for record in selected['target_estimate']:
        try:
            if record.get('frame_id') != context['world_frame']:
                raise ValueError('incompatible target estimate frame')
            state = finite_vector(record['data']['mean'], 6)
            target = truth.nearest(record['source_time'], max_skew)
            error = np.linalg.norm(state[:3] - finite_vector(target['data']['position'], 3))
            _append(series, 'target_position_error', record['source_time'], error)
        except _INVALID:
            continue
    for record in selected['control']:
        try:
            value = float(record['data']['computation_time'])
            if math.isfinite(value) and value >= 0:
                _append(series, 'control_compute_time', record['source_time'], value)
        except _INVALID:
            continue
    clock_samples = selected['clock']
    total_sim, total_wall, bin_sim, bin_wall = 0., 0., 0., 0.
    for previous, current in zip(clock_samples, clock_samples[1:]):
        dt = current['arrival_monotonic'] - previous['arrival_monotonic']
        ds = current['source_time'] - previous['source_time']
        if dt > 0 and ds > 0:
            total_sim += ds
            total_wall += dt
            bin_sim += ds
            bin_wall += dt
            if bin_wall >= .5:
                _append(series, 'realtime_factor', current['source_time'], bin_sim / bin_wall,
                        wall_duration=bin_wall, simulation_duration=bin_sim)
                bin_sim, bin_wall = 0., 0.
    # Small batched delivery intervals are absorbed into half-second bins. A
    # short run still gets one bin; an undersized final tail affects only the
    # total elapsed-time ratio, never a misleading instantaneous peak.
    if bin_wall >= .1 or (bin_wall > 0 and not series['realtime_factor']['samples']):
        _append(series, 'realtime_factor', clock_samples[-1]['source_time'], bin_sim / bin_wall,
                wall_duration=bin_wall, simulation_duration=bin_sim)
    for name in ['tracking_distance', 'pointing_error', 'target_position_error',
                 'robot_position_error', 'robot_yaw_error', 'robot_joint_error',
                 'control_compute_time', 'realtime_factor']:
        metrics[name] = _series_metrics(series, name)
    if total_wall > 0:
        metrics['realtime_factor'].update(
            mean=total_sim / total_wall,
            overall_ratio=total_sim / total_wall, wall_duration=total_wall,
            simulation_duration=total_sim,
            weighting=('mean is total simulation delta / total monotonic delta; '
                       'other stats use >=0.5s bins with >=0.1s final tail'))
    metrics['prediction_rmse'] = _predictions(selected['prediction'], truth, end, max_skew, series)
    tolerance = float(context.get('physical_numerical_tolerance', .001))
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError('physical numerical tolerance must be finite and nonnegative')
    safety_metrics, safety = _safety(all_records['joints'], names, parameters.joint_limits,
                                     start, end, max_skew, series, tolerance)
    metrics.update(safety_metrics)
    valid_flags = [record['data']['valid'] for record in selected['control']
                   if isinstance(record['data'].get('valid'), bool)]
    metrics['invalid_commands'] = {
        'available': bool(valid_flags), 'unit': 'count',
        'sample_count': len(valid_flags), 'count': sum(not flag for flag in valid_flags)
        if valid_flags else None, 'reason': None if valid_flags else 'no control validity samples'}
    for metric, counter in [('watchdog_events', 'timeout_events'),
                            ('rejected_commands', 'rejected_commands'),
                            ('feedback_rejections', 'feedback_rejections'),
                            ('reference_limit_events', 'reference_limit_events')]:
        metrics[metric] = _counter(all_records['actuation'], counter, start, end)
    for name in ['nees', 'nis']:
        metrics[name] = {
            'available': False, 'unit': '1', 'sample_count': 0,
            'reason': 'unsupported: no matched full truth/innovation and covariance contract'}
    for name, fields in [('command', ['velocity', 'active']),
                         ('control', ['raw_velocity', 'safe_velocity', 'valid', 'reason']),
                         ('actuation', ['active', 'reason'])]:
        # Numerical vectors and event markers retain source epochs for read-only consumers.
        samples = []
        for record in selected[name]:
            values = {key: record['data'][key] for key in fields if key in record['data']}
            try:
                for value in values.values():
                    if isinstance(value, list):
                        if not np.isfinite(np.asarray(value, dtype=float)).all():
                            raise ValueError('nonfinite telemetry vector')
                samples.append({'time': record['source_time'], **values})
            except _INVALID:
                continue
        series[name] = {'time_basis': 'simulation source time', 'time_unit': 's',
                        'units': {'velocity': 'm/s,rad/s; base v,w then joints',
                                  'raw_velocity': 'm/s,rad/s; base v,w then joints',
                                  'safe_velocity': 'm/s,rad/s; base v,w then joints'},
                        'samples': samples}
    return {'schema_version': 1, 'evaluation_window': {'start': start, 'end': end, 'unit': 's'},
            'contract': {
                'time_basis': 'simulation source epoch', 'max_skew': max_skew,
                'alignment': ('nearest for tracking/estimation; '
                              'bracket interpolation only for prediction truth'),
                'statistics': 'equal sample weights; p95 linear percentile; no smoothing',
                'world_frame': context['world_frame'],
                'estimator_frame': context['estimator_frame'],
                'warmup': 'excluded by caller-supplied evaluation window',
                'safety_tolerance': tolerance},
            'metrics': metrics, 'safety': safety, 'streams': stream_stats, 'time_series': series}
