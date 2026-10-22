"""Pure source-time target diagnostics; no ROS or deployed filter mutation.

Interpolation is for evaluation only. Missing brackets remain missing, and the
roughness metric compares all streams on exactly the same valid grid triples.
The CV replay uses the implementation's discrete acceleration covariance, not a
continuous white-noise approximation with different powers of dt.
"""

import numpy as np


def _series(times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    if (times.ndim != 1 or not times.size or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0.) or values.ndim != 2
            or values.shape[0] != times.size or not values.shape[1]
            or not np.isfinite(values).all()):
        raise ValueError('finite vectors and strictly increasing epochs required')
    return times, values


def bounded_interpolate(times, values, queries, max_gap):
    """Linearly interpolate only within brackets no wider than max_gap.

    Exact observations are retained even next to a gap. No extrapolation occurs.
    Callers must split clock resets and incompatible frames before invoking this
    numerical function; unordered/duplicate input epochs are rejected.
    """
    times, values = _series(times, values)
    queries = np.asarray(queries, dtype=float)
    if (queries.ndim != 1 or not np.isfinite(queries).all()
            or not np.isfinite(max_gap) or max_gap <= 0.):
        raise ValueError('finite queries and positive gap required')
    output = np.full((queries.size, values.shape[1]), np.nan)
    for j, query in enumerate(queries):
        index = np.searchsorted(times, query)
        if index < times.size and abs(times[index] - query) <= 1e-9:
            output[j] = values[index]
        elif index and abs(times[index - 1] - query) <= 1e-9:
            output[j] = values[index - 1]
        elif index and index < times.size:
            low, high = index - 1, index
            if times[high] - times[low] <= max_gap + 1e-9:
                weight = (query - times[low]) / (times[high] - times[low])
                output[j] = values[low] + weight * (values[high] - values[low])
    return output


def bounded_previous(times, queries, max_age):
    """Return causal previous-sample indices, or -1 if absent/stale."""
    times, _ = _series(times, np.zeros((len(times), 1)))
    queries = np.asarray(queries, dtype=float)
    if (queries.ndim != 1 or not np.isfinite(queries).all()
            or not np.isfinite(max_age) or max_age < 0.):
        raise ValueError('finite queries and nonnegative age required')
    indices = np.searchsorted(times, queries, side='right') - 1
    valid = indices >= 0
    ages = queries - times[np.maximum(indices, 0)]
    valid &= ages <= max_age + 1e-9
    return np.where(valid, indices, -1)


def error_metrics(errors):
    """Norm RMSE, axis RMSE and signed bias over identical valid observations."""
    errors = np.asarray(errors, dtype=float)
    if (errors.ndim != 2 or errors.shape[1] != 3 or not errors.shape[0]
            or not np.isfinite(errors).all()):
        raise ValueError('nonempty finite Nx3 errors required')
    return {
        'sample_count': int(errors.shape[0]),
        'rmse_norm_m': float(np.sqrt(np.mean(np.sum(errors ** 2, axis=1)))),
        'rmse_axis_m': np.sqrt(np.mean(errors ** 2, axis=0)).tolist(),
        'bias_axis_m': np.mean(errors, axis=0).tolist(),
        'error_std_axis_m': np.std(errors, axis=0).tolist(),
    }


def sampling_summary(times, gap_threshold=.1):
    """Report recorded sequence faults before considering unique sorted epochs."""
    raw = np.asarray(times, dtype=float)
    if raw.ndim != 1 or not np.isfinite(gap_threshold) or gap_threshold <= 0.:
        raise ValueError('one-dimensional epochs and positive gap required')
    finite = raw[np.isfinite(raw)]
    unique = np.unique(finite)
    gaps = np.diff(unique)
    duration = float(unique[-1] - unique[0]) if unique.size > 1 else 0.
    return {
        'samples': int(raw.size), 'nonfinite_times': int(raw.size - finite.size),
        'duplicates': int(finite.size - unique.size),
        'backward_steps': int(np.sum(np.diff(finite) < 0.)),
        'start_s': float(unique[0]) if unique.size else None,
        'end_s': float(unique[-1]) if unique.size else None,
        'effective_rate_hz': float((unique.size - 1) / duration) if duration else None,
        'median_interval_s': float(np.median(gaps)) if gaps.size else None,
        'largest_gap_s': float(np.max(gaps)) if gaps.size else None,
        'gap_threshold_s': float(gap_threshold),
        'gaps_over_threshold': int(np.sum(gaps > gap_threshold + 1e-9)),
    }


def common_grid_roughness(times, series, step=.1, max_gap=.12):
    """RMS norm of second differences / step**2 on common valid triples.

    Each input uses the declared common source epochs. Values on the grid use
    bounded interpolation, and no second difference bridges missing grid points.
    The result is descriptive grid-dependent curvature/noise, not physical
    acceleration of a noisy measured trajectory or a filter-quality criterion.
    """
    if not np.isfinite(step) or step <= 0. or not series:
        raise ValueError('positive grid step and at least one series required')
    times, _ = _series(times, series[0])
    grid = np.arange(np.ceil(times[0] / step) * step, times[-1] + 1e-9, step)
    aligned = [bounded_interpolate(times, v, grid, max_gap) for v in series]
    valid = np.logical_and.reduce([np.isfinite(v).all(axis=1) for v in aligned])
    triples = valid[:-2] & valid[1:-1] & valid[2:]
    summaries = []
    for values in aligned:
        second = np.diff(values, n=2, axis=0)[triples] / step ** 2
        pairs = valid[:-1] & valid[1:]
        travel = np.linalg.norm(np.diff(values, axis=0)[pairs], axis=1)
        summaries.append({
            'acceleration_rms_m_s2': (
                float(np.sqrt(np.mean(np.sum(second ** 2, axis=1))))
                if second.size else None),
            'acceleration_rms_axis_m_s2': (
                np.sqrt(np.mean(second ** 2, axis=0)).tolist() if second.size else None),
            'grid_path_length_m': float(np.sum(travel)),
        })
    return {'grid_step_s': float(step), 'valid_points': int(np.sum(valid)),
            'grid_points': int(grid.size), 'valid_triples': int(np.sum(triples)),
            'series': summaries}


def replay_constant_velocity(times, measurements, initial_mean, initial_covariance,
                             position_variance, acceleration_noise_std):
    """Reconstruct the deployed linear CV/Joseph updates on one complete epoch.

    The first observation updates the initial prior without a prediction, matching
    estimation_node. Genuine innovations use the pre-update state. Reproduction
    against the recorded posterior is a caller-enforced prerequisite for inferring
    live innovations/gains from this replay; the function alone proves no identity.
    """
    times, measurements = _series(times, measurements)
    mean = np.asarray(initial_mean, dtype=float).copy()
    covariance = np.asarray(initial_covariance, dtype=float).copy()
    variance = np.asarray(position_variance, dtype=float)
    sigma = float(acceleration_noise_std)
    if (measurements.shape[1] != 3 or mean.shape != (6,)
            or covariance.shape != (6, 6) or variance.shape != (3,)
            or not np.isfinite(mean).all() or not np.isfinite(covariance).all()
            or not np.isfinite(variance).all() or np.any(variance <= 0.)
            or not np.isfinite(sigma) or sigma < 0.
            or not np.allclose(covariance, covariance.T)
            or np.min(np.linalg.eigvalsh(covariance)) < -1e-10):
        raise ValueError('invalid CV state or positive-definite measurement noise')
    identity = np.eye(6)
    h_matrix = np.column_stack([np.eye(3), np.zeros((3, 3))])
    measurement_covariance = np.diag(variance)
    result = {key: [] for key in ['mean', 'covariance', 'innovation',
                                  'innovation_covariance', 'gain', 'correction', 'nis']}
    for index, observation in enumerate(measurements):
        if index:
            dt = times[index] - times[index - 1]
            transition = np.eye(6)
            transition[:3, 3:] = dt * np.eye(3)
            noise = sigma ** 2 * np.block([
                [dt ** 4 / 4. * np.eye(3), dt ** 3 / 2. * np.eye(3)],
                [dt ** 3 / 2. * np.eye(3), dt ** 2 * np.eye(3)]])
            mean = transition @ mean
            covariance = transition @ covariance @ transition.T + noise
        innovation = observation - h_matrix @ mean
        s_matrix = h_matrix @ covariance @ h_matrix.T + measurement_covariance
        gain = np.linalg.solve(s_matrix.T, (covariance @ h_matrix.T).T).T
        correction = gain @ innovation
        mean += correction
        joseph = identity - gain @ h_matrix
        covariance = (joseph @ covariance @ joseph.T
                      + gain @ measurement_covariance @ gain.T)
        for key, value in [('mean', mean.copy()), ('covariance', covariance.copy()),
                           ('innovation', innovation), ('innovation_covariance', s_matrix),
                           ('gain', gain), ('correction', correction),
                           ('nis', innovation @ np.linalg.solve(s_matrix, innovation))]:
            result[key].append(value)
    return {key: np.asarray(value) for key, value in result.items()}


def turn_lags(times, truth, estimate, step=.05, radius=.4, max_delay=1.,
              minimum_excursion=.02):
    """Descriptive local axis shift near truth extrema, searched from 0 to max_delay.

    Extrema must have minimum_excursion on each side in the truth window. For
    each candidate shift d, compare estimate(t+d) against truth(t) over that
    window. A boundary optimum is marked unresolved; this is not a causal delay
    estimate and is sensitive to noise, local shape and the chosen search window.
    """
    times, truth = _series(times, truth)
    _, estimate = _series(times, estimate)
    if (truth.shape[1] != 3 or estimate.shape != truth.shape
            or not np.isfinite([step, radius, max_delay, minimum_excursion]).all()
            or step <= 0. or radius < step or max_delay < 0. or minimum_excursion < 0.):
        raise ValueError('invalid lag search parameters')
    grid = np.arange(times[0], times[-1] + 1e-9, step)
    actual = bounded_interpolate(times, truth, grid, step * 2.01)
    events = []
    offsets = np.arange(0., max_delay + step / 2., step)
    half = int(round(radius / step))
    for axis in range(3):
        derivative = np.diff(actual[:, axis])
        candidates = np.flatnonzero(derivative[:-1] * derivative[1:] < 0.) + 1
        previous = -np.inf
        for index in candidates:
            epoch = grid[index]
            if epoch - previous < 2. * radius or index < half or index + half >= grid.size:
                continue
            local = slice(index - half, index + half + 1)
            reference = actual[local, axis]
            if (not np.isfinite(reference).all()
                    or min(abs(reference[half] - reference[0]),
                           abs(reference[half] - reference[-1])) < minimum_excursion):
                continue
            scores = []
            for delay in offsets:
                shifted = bounded_interpolate(
                    times, estimate, grid[local] + delay, step * 2.01)[:, axis]
                scores.append(float(np.sqrt(np.mean((shifted - reference) ** 2)))
                              if np.isfinite(shifted).all() else np.inf)
            if not np.isfinite(scores).any():
                continue
            best = int(np.argmin(scores))
            supported = np.flatnonzero(np.isfinite(scores))
            events.append({'axis': 'xyz'[axis], 'truth_turn_s': float(epoch),
                           'lag_s': float(offsets[best]),
                           'local_rmse_m': float(scores[best]),
                           'unshifted_rmse_m': float(scores[0]),
                           'boundary_optimum': bool(best in (supported[0], supported[-1])),
                           'maximum_supported_delay_s': float(offsets[supported[-1]]),
                           'search_fully_supported': bool(supported.size == offsets.size)})
            previous = epoch
    return events
