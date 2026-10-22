"""Pure sampled joint diagnostics with explicit gaps and causal command alignment.

No ROS clock conversion, interpolation across missing feedback, or physics claim is
made. Integrals use trapezoids of sampled velocity squared on eligible intervals;
violation durations use the left sample on those same intervals.
"""

import numpy as np


def named_joint_series(records, name):
    """Extract one joint by name in every record, rejecting malformed samples.

    Input must belong to one source-clock epoch. Duplicates and nonincreasing
    timestamps are rejected; a backwards clock raises rather than merging epochs.
    """
    times, positions, velocities = [], [], []
    rejected = 0
    previous = -np.inf
    for record in records:
        try:
            stamp = float(record['source_time'])
            data = record['data']
            names = data['names']
            if len(names) != len(set(names)):
                raise ValueError('duplicate joint name')
            index = names.index(name)
            q, dq = float(data['position'][index]), float(data['velocity'][index])
            if not np.isfinite([stamp, q, dq]).all() or stamp == previous:
                raise ValueError('invalid or duplicate sample')
        except (KeyError, TypeError, ValueError, IndexError):
            rejected += 1
            continue
        if stamp < previous:
            raise ValueError('backwards source time: split clock epochs before analysis')
        times.append(stamp)
        positions.append(q)
        velocities.append(dq)
        previous = stamp
    return np.asarray(times), np.asarray(positions), np.asarray(velocities), rejected


def sampled_motion(times, positions, velocities, start, end, max_gap,
                   speed_limit=None, position_limits=None, tolerance=0.001):
    """Summarize physical samples; unobserved intervals never count as zero.

    Range and adjacent-sample travel describe the received positions. Travel
    excluding long gaps is separately exposed. RMS uses *observed* duration.
    The full-window normalized observed integral is a partial sampled integral,
    not an estimate or rigorous bound on true full-window physical RMS. p95 is sample weighted.
    """
    t, q, v = (np.asarray(a, dtype=float) for a in (times, positions, velocities))
    if (t.ndim != 1 or q.shape != t.shape or v.shape != t.shape or
            not np.isfinite([start, end, max_gap, tolerance]).all() or
            end <= start or max_gap <= 0 or tolerance < 0):
        raise ValueError('invalid dimensions, window, gap, or tolerance')
    if not (np.isfinite(t).all() and np.isfinite(q).all() and np.isfinite(v).all()):
        raise ValueError('nonfinite samples')
    if np.any(np.diff(t) <= 0):
        raise ValueError('samples must increase within one clock epoch')
    use = (t >= start) & (t <= end)
    t, q, v = t[use], q[use], v[use]
    duration = float(end - start)
    result = {'sample_count': len(t), 'window_s': [float(start), float(end)],
              'window_duration_s': duration, 'max_eligible_gap_s': float(max_gap),
              'rms_policy': 'trapezoid sampled squared velocity / observed duration',
              'violation_policy': 'left-held sample on eligible feedback intervals',
              'p95_policy': 'equal sample weights'}
    if not len(t):
        result.update(available=False, observed_duration_s=0., coverage_fraction=0.,
                      largest_gap_including_edges_s=duration)
        return result
    dt = np.diff(t)
    eligible = dt <= max_gap + 1e-12
    weights = np.where(eligible, dt, 0.)
    observed = float(np.sum(weights))
    integral = float(np.sum(weights * (v[:-1] ** 2 + v[1:] ** 2) / 2))
    result.update(
        available=True, first_time_s=float(t[0]), last_time_s=float(t[-1]),
        q_min_rad=float(np.min(q)), q_max_rad=float(np.max(q)),
        angular_range_rad=float(np.ptp(q)),
        sampled_adjacent_travel_rad=float(np.sum(np.abs(np.diff(q)))),
        observed_travel_excluding_gaps_rad=float(np.sum(np.abs(np.diff(q))[eligible])),
        velocity_squared_integral_rad2_per_s=integral,
        observed_duration_s=observed, uncovered_duration_s=duration-observed,
        coverage_fraction=observed/duration,
        velocity_rms_observed_rad_s=float(np.sqrt(integral/observed)) if observed else None,
        full_window_normalized_observed_rms_rad_s=float(np.sqrt(integral/duration)),
        peak_abs_velocity_rad_s=float(np.max(np.abs(v))),
        p95_abs_velocity_rad_s=float(np.percentile(np.abs(v), 95)),
        median_sample_interval_s=float(np.median(dt)) if len(dt) else None,
        largest_internal_gap_s=float(np.max(dt)) if len(dt) else None,
        largest_gap_including_edges_s=float(max(t[0]-start, end-t[-1], max(dt, default=0))),
        excluded_gap_count=int(np.sum(~eligible)),
    )
    if speed_limit is not None:
        if not np.isfinite(speed_limit) or speed_limit <= 0:
            raise ValueError('invalid speed limit')
        excess = np.maximum(np.abs(v) - speed_limit, 0.)
        violation = excess > tolerance
        result['speed_limit'] = {
            'description_rad_s': float(speed_limit), 'acceptance_tolerance_rad_s': tolerance,
            'max_excess_above_description_rad_s': float(np.max(excess)),
            'max_excess_above_acceptance_rad_s': float(max(0., np.max(excess)-tolerance)),
            'violating_samples': int(np.sum(violation)),
            'observed_violation_duration_s': float(np.sum(weights * violation[:-1])),
        }
    if position_limits is not None:
        low, high = position_limits
        if not np.isfinite([low, high]).all() or high <= low:
            raise ValueError('invalid position limits')
        excess = np.maximum(np.maximum(low-q, q-high), 0.)
        result['position_limits'] = {
            'description_rad': [float(low), float(high)], 'acceptance_tolerance_rad': tolerance,
            'max_excess_rad': float(np.max(excess)),
            'violating_samples': int(np.sum(excess > tolerance)),
            'observed_violation_duration_s': float(np.sum(weights * (excess[:-1] > tolerance))),
        }
    return result


def causal_indices(source_times, query_times, max_age):
    """Return most recent nonfuture source index, or -1 for absent/stale data."""
    source = np.asarray(source_times, dtype=float)
    query = np.asarray(query_times, dtype=float)
    if (source.ndim != 1 or query.ndim != 1 or not np.isfinite(source).all() or
            not np.isfinite(query).all() or np.any(np.diff(source) <= 0) or
            not np.isfinite(max_age) or max_age < 0):
        raise ValueError('invalid causal index inputs')
    indices = np.searchsorted(source, query, side='right') - 1
    if not len(source):
        return np.full(len(query), -1, dtype=int)
    safe = np.maximum(indices, 0)
    return np.where((indices >= 0) & (query-source[safe] <= max_age + 1e-12), indices, -1)


def body_to_world(orientation_xyzw, vector):
    """Rotate a body vector into world using a finite normalized quaternion."""
    q, v = np.asarray(orientation_xyzw, float), np.asarray(vector, float)
    if q.shape != (4,) or v.shape != (3,) or not np.isfinite(np.r_[q, v]).all():
        raise ValueError('invalid quaternion/vector')
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('zero quaternion')
    q = q/norm
    return v + 2*np.cross(q[:3], np.cross(q[:3], v) + q[3]*v)
