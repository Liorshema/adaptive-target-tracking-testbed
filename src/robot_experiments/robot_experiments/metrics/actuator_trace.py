"""Full physics-iteration actuator acceptance; CSV input, no ROS or state alteration."""
import csv
import hashlib

import numpy as np

from robot_experiments.metrics.joint_diagnostics import sampled_motion


def analyze_trace(path, context, active_start):
    """Evaluate every recorded physical sample, including pre-controller startup."""
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    physical = context['description']['robot']['manipulator']['joints']
    names = context['joint_names']
    if not names or len(names) != len(physical) or len(set(names)) != len(names):
        raise ValueError('Invalid actuator joint layout')
    result = {'path': str(path), 'joints': {}, 'complete': bool(rows), 'accepted': True,
              'torque_semantics': 'PID written effort is a command; '
              'native motor torque unavailable'}
    if {r['name'] for r in rows} != set(names):
        result['complete'] = result['accepted'] = False
    common_grid = None
    for name, limits in zip(names, physical.values()):
        samples = [r for r in rows if r['name'] == name]
        if not samples:
            result['complete'] = result['accepted'] = False
            continue

        def arr(key):
            return np.array([float(r[key]) for r in samples])
        t, q, v, dt, iterations = [arr(k) for k in ['time', 'q_post', 'v_post', 'dt', 'iteration']]
        finite = np.isfinite(np.r_[t, q, v, dt]).all()
        contiguous = finite and np.all(np.diff(iterations) == 1) and np.all(np.diff(t) > 0)
        complete = bool(contiguous and t[0] <= np.max(dt) + 1e-9 and
                        np.all(np.diff(t) <= dt[1:] * 1.01 + 1e-9))
        grid = (t, iterations)
        if common_grid is None:
            common_grid = grid
        else:
            complete &= all(np.array_equal(a, b) for a, b in zip(grid, common_grid))
        if not finite or len(t) < 2:
            result['complete'] = result['accepted'] = False
            continue
        metrics = sampled_motion(t, q, v, t[0], t[-1], float(np.max(dt) * 1.01),
                                 limits['velocity'], (limits['lower'], limits['upper']))
        pieces = {}
        for label, start, end in [('startup', t[0], active_start),
                                  ('tracking', active_start, t[-1])]:
            if end > start:
                pieces[label] = sampled_motion(
                    t, q, v, start, end, float(np.max(dt)*1.01),
                    limits['velocity'], (limits['lower'], limits['upper']))
        qdiff = np.diff(q) / np.diff(t)
        reference = arr('position_reference')
        commanded = arr('commanded_effort')
        available = np.isfinite(commanded)
        metrics.update(
            complete=complete, intervals=pieces,
            max_position_difference_velocity_rad_s=float(np.max(np.abs(qdiff))),
            feedback_vs_position_difference_max_rad_s=float(np.max(np.abs(qdiff-v[1:]))),
            position_reference_error_rms_rad=float(np.sqrt(np.mean((reference-q)**2))),
            max_abs_position_reference_error_rad=float(np.max(np.abs(reference-q))),
            max_abs_written_effort_nm=(float(np.max(np.abs(commanded[available])))
                                       if available.any() else None),
            effort_limit_nm=limits['effort'], saturation_count=int(np.sum(arr('saturation'))),
            invalid_target_count=int(max(arr('invalid_commands'))))
        result['joints'][name] = metrics
        result['complete'] &= complete
        result['accepted'] &= (
            complete and not metrics['speed_limit']['violating_samples'] and
            not metrics['position_limits']['violating_samples'] and
            (not available.any() or
             np.max(np.abs(commanded[available])) <= limits['effort']+1e-9))
    result['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result
