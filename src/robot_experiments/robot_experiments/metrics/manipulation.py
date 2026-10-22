"""Pure physical manipulation metrics on explicit source-time phase windows.

Alignment is causal: the latest physical base/joint sample no older than max_skew
is used at each target epoch. No interpolation, estimated pose, commands or future
feedback supply physical truth. Integrals and angle unwraps exclude intervals
with missing endpoints or gaps above max_gap. Angular travel assumes less than pi
of actual angular motion between eligible samples; it cannot establish hidden turns.
"""

import math

import numpy as np

from robot_bringup.factory.model_factory import create_robot_model

from robot_description.model_parameters import robot_model_parameters_from_description

from robot_models.whole_body.kinematics import WholeBodyKinematics

from . import _ordered, _transform
from .alignment import finite_vector, statistics
from .joint_diagnostics import causal_indices, named_joint_series, sampled_motion


_INVALID = (KeyError, TypeError, ValueError, IndexError, OverflowError)


def _records(records):
    """Reject malformed/duplicate epochs and refuse to merge source-clock resets."""
    output, rejected, previous = [], 0, -np.inf
    for record in records:
        try:
            time = float(record['source_time'])
            if not np.isfinite(time):
                raise ValueError('nonfinite source time')
        except _INVALID:
            rejected += 1
            continue
        if time < previous:
            raise ValueError('backwards source time: split clock epochs before analysis')
        if time == previous:
            rejected += 1
            continue
        previous = time
        output.append(record)
    return output, rejected


def _segments(rows, field, start, end, max_gap):
    """Keep missing observations as segment boundaries rather than bridging them."""
    segments, current = [], []
    for row in rows:
        if not start <= row['time'] <= end:
            continue
        if row.get(field) is None:
            if current:
                segments.append(current)
            current = []
            continue
        if current and row['time'] - current[-1]['time'] > max_gap + 1e-12:
            segments.append(current)
            current = []
        current.append(row)
    if current:
        segments.append(current)
    return segments


def _pairs(rows, fields, start, end, max_gap):
    """Yield only adjacent, fully observed intervals wholly inside this window."""
    selected = [row for row in rows if start <= row['time'] <= end]
    for left, right in zip(selected, selected[1:]):
        dt = right['time'] - left['time']
        if (0 < dt <= max_gap + 1e-12 and
                all(left.get(field) is not None and right.get(field) is not None
                    for field in fields)):
            yield left, right, dt


def _scalar(rows, field, unit, start, end, max_gap):
    segments = _segments(rows, field, start, end, max_gap)
    values = [row[field] for segment in segments for row in segment]
    result = statistics(values, unit)
    if values:
        result.update(min=float(min(values)), range=float(max(values) - min(values)),
                      segment_end_minus_start=[
                          float(segment[-1][field] - segment[0][field])
                          for segment in segments if len(segment) > 1])
    return result


def _arc_coverage(arcs):
    """Measure the union of observed shortest angular arcs on one circle."""
    intervals = []
    period = 2 * math.pi
    for first, second in arcs:
        low, high = sorted((first, second))
        length = high - low
        low %= period
        if low + length <= period:
            intervals.append((low, low + length))
        else:
            intervals.extend([(low, period), (0., low + length - period)])
    covered, rightmost = 0., -np.inf
    for low, high in sorted(intervals):
        covered += max(0., high - max(low, rightmost))
        rightmost = max(rightmost, high)
    return float(min(period, covered))


def _angular(rows, field, start, end, max_gap):
    segments = _segments(rows, field, start, end, max_gap)
    if not segments:
        return {'available': False, 'sample_count': 0, 'reason': 'no compatible physical samples'}
    details, arcs = [], []
    for segment in segments:
        values = np.unwrap([row[field] for row in segment])
        arcs.extend(zip(values[:-1], values[1:]))
        details.append({
            'start': segment[0]['time'], 'end': segment[-1]['time'],
            'sample_count': len(segment), 'excursion_rad': float(np.ptp(values)),
            'net_change_rad': float(values[-1] - values[0]),
            'travel_rad': float(np.sum(np.abs(np.diff(values)))),
            'observed_duration_s': float(segment[-1]['time'] - segment[0]['time'])})
    return {'available': True, 'unit': 'rad',
            'sample_count': sum(item['sample_count'] for item in details),
            'segment_count': len(details), 'segments': details,
            'unwrapped_excursion_rad': max(item['excursion_rad'] for item in details),
            'observed_travel_rad': sum(item['travel_rad'] for item in details),
            'observed_duration_s': sum(item['observed_duration_s'] for item in details),
            'angular_coverage_rad': _arc_coverage(arcs),
            'unwrap_policy': 'independent contiguous segments; largest segment excursion'}


def _joint_metrics(series, names, limits, start, end, max_gap, tolerance, proximity):
    results = {}
    for index, name in enumerate(names):
        times, q, dq, rejected = series[name]
        low, high = limits.lower[index], limits.upper[index]
        speed = limits.velocity[index]
        result = sampled_motion(times, q, dq, start, end, max_gap,
                                speed_limit=speed, position_limits=(low, high),
                                tolerance=tolerance)
        result['rejected_samples'] = rejected
        selected = (times >= start) & (times <= end)
        t, angles, speeds = times[selected], q[selected], dq[selected]
        if len(angles):
            fractions = (angles - low) / (high - low)
            result['q_fraction_of_range'] = {
                'min': float(np.min(fractions)), 'max': float(np.max(fractions))}
            result['peak_speed_utilization'] = float(np.max(np.abs(speeds)) / speed)
            result['peak_position_midpoint_utilization'] = float(
                np.max(np.abs(2 * fractions - 1)))
            near = np.minimum(fractions, 1 - fractions) <= proximity
            dt = np.diff(t)
            eligible = dt <= max_gap + 1e-12
            result['near_limit_observed_duration_s'] = float(np.sum(dt * eligible * near[:-1]))
            result['near_limit_policy'] = 'left-held on eligible physical joint intervals'
        results[name] = result
    return results


def _target_metrics(rows, start, end, max_gap, region):
    result = {name: _scalar(rows, field, unit, start, end, max_gap)
              for name, field, unit in [('rho', 'rho_m', 'm'), ('height', 'height_m', 'm'),
                                        ('vertical_velocity', 'vertical_velocity_m_s', 'm/s')]}
    result['relative_azimuth'] = _angular(rows, 'azimuth_rad', start, end, max_gap)
    angular = [row for row in rows if start <= row['time'] <= end
               and row['azimuth_rad'] is not None]

    def rear(angle):
        return abs(angle) > math.pi / 2
    result.update(rear_hemisphere_observed=(
        any(rear(row['azimuth_rad']) for row in angular) if angular else None),
                  rear_entries=0 if angular else None, rear_exits=0 if angular else None,
                  aft_axis_crossings=0 if angular else None,
                  rear_dwell_observed_s=0. if angular else None)
    for left, right, dt in _pairs(rows, ['azimuth_rad'], start, end, max_gap):
        a, b = left['azimuth_rad'], right['azimuth_rad']
        result['rear_entries'] += int(not rear(a) and rear(b))
        result['rear_exits'] += int(rear(a) and not rear(b))
        result['rear_dwell_observed_s'] += dt * rear(a)
        # Unwrap only this observed short arc; odd pi is the physical aft axis.
        b = a + math.atan2(math.sin(b - a), math.cos(b - a))
        low, high = sorted((a, b))
        result['aft_axis_crossings'] += max(
            0, math.floor((high - math.pi) / (2 * math.pi))
            - math.floor((low - math.pi) / (2 * math.pi)))
    observed, overhead = 0., 0.
    for left, _, dt in _pairs(rows, ['rho_m', 'height_m'], start, end, max_gap):
        observed += dt
        inside = (region['rho'][0] <= left['rho_m'] <= region['rho'][1] and
                  region['height'][0] <= left['height_m'] <= region['height'][1])
        overhead += dt * inside
    times = [r['time'] for r in rows if start <= r['time'] <= end]
    gaps = np.diff(times)
    result.update(largest_target_gap_s=float(max(gaps, default=0.)),
                  excluded_target_gap_count=int(np.sum(gaps > max_gap)),
                  observed_geometry_duration_s=observed,
                  geometry_coverage_fraction=observed / (end - start),
                  overhead_dwell_observed_s=overhead if result['rho']['available'] else None,
                  dwell_policy='left-held on eligible adjacent physical geometry intervals')
    return result


def _health(streams, start, end, max_gap):
    """Observe source-stamped health; missing intervals are never active."""
    result = {}
    for stream, field in [('control', 'valid'), ('actuation', 'active')]:
        records, _ = _records(streams.get(stream, []))
        samples = [r for r in records if start <= r['source_time'] <= end]
        inactive, observed, transitions, recovery, fault = 0., 0., 0, 0., 0.
        for left, right in zip(samples, samples[1:]):
            dt = right['source_time'] - left['source_time']
            if dt > max_gap:
                continue
            observed += dt
            value = left['data'].get(field)
            inactive += dt * (value is False)
            transitions += int(value is True and right['data'].get(field) is False)
            reason = str(left['data'].get('reason', '')).lower()
            recovery += dt * ('recovery' in reason)
            fault += dt * ('fault' in reason or 'permanent' in reason or 'latched' in reason)
        counters = {}
        for key in ['rejected_commands', 'timeout_events', 'feedback_rejections',
                    'inactive_commands', 'reference_limit_events']:
            values = [r['data'][key] for r in samples
                      if isinstance(r['data'].get(key), (int, float))]
            counters[key] = (values[-1] - values[0] if len(values) > 1
                             and all(b >= a for a, b in zip(values, values[1:])) else None)
        result[stream] = {
            'available': bool(samples), 'observed_duration_s': observed,
            'uncovered_duration_s': end - start - observed,
            'inactive_observed_duration_s': inactive,
            'active_to_inactive_observed_transitions': transitions,
            'recovery_observed_duration_s': recovery, 'fault_observed_duration_s': fault,
            'counter_deltas': counters,
            'policy': 'eligible intervals; counter deltas exclude unobserved boundary events'}
    return result


def analyze_phases(streams, context, phase_windows, active_start, end, *,
                   max_gap=.15, max_skew=.08, overhead_region=None,
                   minimum_horizontal_radius=1e-9, limit_proximity_fraction=.05,
                   tolerance=None):
    """Summarize startup, the active window and named absolute-source-time phases.

    phase_windows contains mappings with name/start/end. Phases must be ordered
    and disjoint; they are clipped to the active interval. Shared boundary
    observations may appear in both phases, while durations never overlap.
    h is world target Z minus physical base Z; azimuth subtracts physical heading.
    vertical_velocity uses adjacent world target Z observations, never a gap.
    Reach is shoulder-to-camera Euclidean distance using authoritative camera FK.
    This is sampled evidence, with no collision, occlusion or continuous safety claim.
    """
    if tolerance is None:
        tolerance = context.get('physical_numerical_tolerance', .001)
    scalars = [active_start, end, max_gap, max_skew, minimum_horizontal_radius,
               limit_proximity_fraction, tolerance]
    if (not np.isfinite(scalars).all() or end <= active_start or max_gap <= 0
            or max_skew < 0 or minimum_horizontal_radius < 0
            or not 0 <= limit_proximity_fraction <= .5 or tolerance < 0):
        raise ValueError('invalid window, gap, alignment or physical tolerance parameters')
    if overhead_region is None:
        raise ValueError('overhead_region must be explicitly configured')
    region = overhead_region
    if (set(region) != {'rho', 'height'} or any(
            np.asarray(region[key]).shape != (2,) or not np.isfinite(region[key]).all()
            or region[key][1] < region[key][0] for key in region) or region['rho'][0] < 0):
        raise ValueError('invalid overhead region bounds')
    windows, names_seen, previous_end = [], {'startup', 'active'}, -np.inf
    for phase in phase_windows:
        name, low, high = phase['name'], float(phase['start']), float(phase['end'])
        if not isinstance(name, str) or not name or name in names_seen:
            raise ValueError('phase names must be unique and not startup/active')
        if not np.isfinite([low, high]).all() or high <= low:
            raise ValueError('invalid phase window')
        if low < previous_end:
            raise ValueError('phase windows overlap or are unordered')
        previous_end = high
        names_seen.add(name)
        if min(high, end) > max(low, active_start):
            windows.append((name, max(low, active_start), min(high, end)))
    parameters = robot_model_parameters_from_description(context['description'])
    names = list(context['joint_names'])
    keys = list(context['description']['robot']['manipulator']['joints'])
    if len(names) != len(keys) or len(set(names)) != len(names):
        raise ValueError('authoritative joint name order must match description dimension')
    if np.any(parameters.joint_limits.upper <= parameters.joint_limits.lower):
        raise ValueError('joint position intervals must have positive width')
    model = create_robot_model(parameters)
    axis = finite_vector(context['control']['tracking_forward_axis'], 3)
    radius = float(context['control']['tracking_radius'])
    if np.linalg.norm(axis) == 0 or not np.isfinite(radius) or radius <= 0:
        raise ValueError('tracking axis and radius must be nonzero and finite')
    axis /= np.linalg.norm(axis)
    arm_yaw_index = keys.index('base_yaw') if 'base_yaw' in keys else None
    records, availability = {}, {}
    for stream in ['base_truth', 'target_truth', 'joints']:
        records[stream], rejected = _records(streams.get(stream, []))
        availability[stream] = {
            'received': len(streams.get(stream, [])), 'rejected_epochs': rejected}
    joint_series = {name: named_joint_series(records['joints'], name) for name in names}
    base_rows, valid_base, valid_joints = [], [], []
    for sample in records['base_truth']:
        row = {'time': float(sample['source_time']), 'base_position_world_m': None,
               'base_yaw_rad': None}
        try:
            if sample.get('frame_id') != context['world_frame']:
                raise ValueError('physical base frame mismatch')
            transform = _transform(sample['data'])
            row.update(base_position_world_m=transform[:3, 3].tolist(),
                       base_yaw_rad=float(math.atan2(transform[1, 0], transform[0, 0])))
            valid_base.append((sample, transform))
        except _INVALID:
            availability['base_truth']['invalid_frame_or_pose'] = (
                availability['base_truth'].get('invalid_frame_or_pose', 0) + 1)
        base_rows.append(row)
    for sample in records['joints']:
        try:
            valid_joints.append((sample, _ordered(sample['data'], names, 'position')))
        except _INVALID:
            availability['joints']['invalid_full_position'] = (
                availability['joints'].get('invalid_full_position', 0) + 1)
    target_records = [sample for sample in records['target_truth'] if sample['source_time'] <= end]
    target_times = [sample['source_time'] for sample in target_records]
    base_indices = causal_indices(
        [sample['source_time'] for sample, _ in valid_base], target_times, max_skew)
    joint_indices = causal_indices(
        [sample['source_time'] for sample, _ in valid_joints], target_times, max_skew)
    availability.update(base_alignment_missing=0, joint_alignment_missing=0,
                        invalid_target_samples=0)
    rows, previous_target = [], None
    shoulder_height = (parameters.manipulator.body_height / 2
                       + parameters.manipulator.arm_base_height)
    shoulder_base = np.array([0., 0., shoulder_height, 1.])
    for sample, base_index, joint_index in zip(target_records, base_indices, joint_indices):
        time = float(sample['source_time'])
        row = {field: None for field in [
            'target_world_position_m', 'base_relative_position', 'rho_m', 'height_m',
            'azimuth_rad', 'reach_m', 'base_yaw_rad', 'arm_yaw_rad',
            'distance_error_m', 'pointing_error_rad', 'vertical_velocity_m_s',
            'vertical_velocity_interval_start', 'base_source_time', 'joint_source_time',
            'camera_world_position_m']}
        row.update(time=time, missing=[])
        try:
            if sample.get('frame_id') != context['world_frame']:
                raise ValueError('physical target frame mismatch')
            target = finite_vector(sample['data']['position'], 3)
            row['target_world_position_m'] = target.tolist()
        except _INVALID:
            availability['invalid_target_samples'] += 1
            row['missing'].append('target truth malformed or wrong frame')
            previous_target = None
            rows.append(row)
            continue
        if previous_target is not None:
            last_time, last_position = previous_target
            if time - last_time <= max_gap + 1e-12:
                row['vertical_velocity_m_s'] = float(
                    (target[2] - last_position[2]) / (time - last_time))
                row['vertical_velocity_interval_start'] = last_time
        previous_target = time, target
        if base_index < 0:
            availability['base_alignment_missing'] += 1
            row['missing'].append('physical base absent, future or stale')
        if joint_index < 0:
            availability['joint_alignment_missing'] += 1
            row['missing'].append('physical joints absent, future or stale')
        if joint_index >= 0:
            joint_sample, q = valid_joints[joint_index]
            row['joint_source_time'] = float(joint_sample['source_time'])
            if arm_yaw_index is not None:
                row['arm_yaw_rad'] = float(q[arm_yaw_index])
        if base_index >= 0:
            base_sample, transform = valid_base[base_index]
            row['base_source_time'] = float(base_sample['source_time'])
            relative = target - transform[:3, 3]
            heading = float(math.atan2(transform[1, 0], transform[0, 0]))
            rho = float(np.linalg.norm(relative[:2]))
            row.update(base_relative_position=relative.tolist(), rho_m=rho,
                       height_m=float(relative[2]), base_yaw_rad=heading)
            if rho > minimum_horizontal_radius:
                angle = math.atan2(relative[1], relative[0]) - heading
                row['azimuth_rad'] = math.atan2(math.sin(angle), math.cos(angle))
            else:
                row['missing'].append('relative azimuth undefined at horizontal zero')
            if joint_index >= 0:
                camera = WholeBodyKinematics.compute(model, transform, q)
                shoulder = (transform @ shoulder_base)[:3]
                delta = target - camera[:3, 3]
                distance = float(np.linalg.norm(delta))
                row.update(reach_m=float(np.linalg.norm(camera[:3, 3] - shoulder)),
                           camera_world_position_m=camera[:3, 3].tolist(),
                           distance_error_m=abs(distance - radius))
                if distance > 0:
                    optical = camera[:3, :3] @ axis
                    row['pointing_error_rad'] = float(
                        np.arccos(np.clip(optical @ delta / distance, -1., 1.)))
        rows.append(row)
    epochs = [sample['source_time'] for group in records.values() for sample in group
              if sample['source_time'] <= active_start]
    source_start = min(epochs, default=active_start)
    all_windows = [('active', active_start, end), *windows]
    if source_start < active_start:
        all_windows.insert(0, ('startup', source_start, active_start))
    summaries = {}
    for name, start, stop in all_windows:
        phase_rows = [dict(row) for row in rows if start <= row['time'] <= stop]
        for row in phase_rows:
            if (row['vertical_velocity_interval_start'] is not None
                    and row['vertical_velocity_interval_start'] < start):
                row['vertical_velocity_m_s'] = None
        base_yaw = _angular(base_rows, 'base_yaw_rad', start, stop, max_gap)
        base_distance = sum(float(np.linalg.norm(
            np.asarray(right['base_position_world_m'])[:2]
            - np.asarray(left['base_position_world_m'])[:2]))
            for left, right, _ in _pairs(
                base_rows, ['base_position_world_m'], start, stop, max_gap))
        observed_base = sum(dt for _, _, dt in _pairs(
            base_rows, ['base_position_world_m'], start, stop, max_gap))
        summaries[name] = {
            'window_s': [float(start), float(stop)],
            'joints': _joint_metrics(joint_series, names, parameters.joint_limits,
                                     start, stop, max_gap, tolerance, limit_proximity_fraction),
            'base': {'yaw': base_yaw, 'observed_planar_travel_m': (
                base_distance if base_yaw['available'] else None),
                     'observed_duration_s': observed_base,
                     'coverage_fraction': observed_base / (stop - start)},
            'target': _target_metrics(phase_rows, start, stop, max_gap, region),
            'reach': _scalar(phase_rows, 'reach_m', 'm', start, stop, max_gap),
            'tracking_distance': _scalar(phase_rows, 'distance_error_m',
                                         'm', start, stop, max_gap),
            'pointing_error': _scalar(
                phase_rows, 'pointing_error_rad', 'rad', start, stop, max_gap),
            'health': _health(streams, start, stop, max_gap)}
    # Export unwrapped plotting series only within observed contiguous segments.
    for field, output in [('azimuth_rad', 'relative_azimuth_rad'),
                          ('base_yaw_rad', 'base_yaw_unwrapped_rad')]:
        previous_time, previous_angle, unwrapped, segment = None, None, None, -1
        for row in rows:
            value = row[field]
            if value is None:
                row[output] = None
                row[output+'_segment'] = None
                previous_time = None
                continue
            if previous_time is None or row['time']-previous_time > max_gap:
                segment += 1
                unwrapped = value
            else:
                unwrapped += math.atan2(
                    math.sin(value - previous_angle), math.cos(value - previous_angle))
            row[output], row[output + '_segment'] = unwrapped, segment
            previous_time, previous_angle = row['time'], value
    summaries.setdefault('startup', {'available': False,
                                     'window_s': [active_start, active_start],
                                     'reason': 'no source interval before active_start'})
    return {'schema_version': 1, 'windows': summaries, 'series': rows,
            'availability': availability,
            'alignment_policy': ('causal latest physical base/joint sample, age <= max_skew; '
                                 'no future feedback'),
            'thresholds': {'max_gap_s': max_gap, 'max_skew_s': max_skew,
                           'overhead_region': region,
                           'minimum_horizontal_radius_m': minimum_horizontal_radius,
                           'limit_proximity_fraction': limit_proximity_fraction,
                           'physical_tolerance_rad_or_rad_s': tolerance},
            'limitations': ['Sampled intervals only; missing intervals are unobserved.',
                            'Angle unwrap assumes less than pi between eligible samples.',
                            'Kinematic reach does not establish collision/occlusion feasibility.']}
