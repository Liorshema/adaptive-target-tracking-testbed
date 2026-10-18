"""Simulation-source-time alignment; no ROS, extrapolation, or clock conversion."""

import math
from bisect import bisect_left

import numpy as np


def finite_vector(value, size):
    """Return a finite vector of the declared layout, or reject it."""
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f'expected finite vector of length {size}')
    return array


def statistics(values, unit, reason='no valid compatible samples'):
    """Summarize equally weighted samples; RMSE is sqrt(mean(value**2))."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    result = {'available': bool(values.size), 'unit': unit, 'sample_count': int(values.size),
              'weighting': 'equal per valid sample'}
    if not values.size:
        result['reason'] = reason
        return result
    result.update(mean=float(np.mean(values)), max=float(np.max(values)),
                  rmse=float(np.sqrt(np.mean(np.square(values)))),
                  p95=float(np.percentile(values, 95)))
    return result


def prepare(records, start, end, max_skew, preserve_duplicates=False):
    """Sort epochs and expose faults; physical samples may preserve duplicates."""
    accepted, previous, seen = [], None, set()
    stats = {'received': len(records), 'invalid_timestamps': 0, 'duplicates': 0,
             'out_of_order': 0, 'outside_window': 0}
    for record in records:
        try:
            stamp = float(record['source_time'])
            arrival = float(record['arrival_monotonic'])
            clock = record.get('clock')
            if not math.isfinite(stamp) or not math.isfinite(arrival) or (
                    clock is not None and not math.isfinite(float(clock))):
                raise ValueError('nonfinite timestamp')
            if not isinstance(record['data'], dict):
                raise ValueError('missing data mapping')
        except (KeyError, TypeError, ValueError, OverflowError):
            stats['invalid_timestamps'] += 1
            continue
        if previous is not None and stamp < previous:
            stats['out_of_order'] += 1
        previous = stamp
        if stamp in seen:
            stats['duplicates'] += 1
            if not preserve_duplicates:
                continue
        seen.add(stamp)
        accepted.append(record)
    accepted.sort(key=lambda sample: sample['source_time'])
    selected = [sample for sample in accepted if start <= sample['source_time'] <= end]
    stats['outside_window'] = len(accepted) - len(selected)
    stats['sample_count'] = len(selected)
    coverage = sum(
        min(max_skew, max(0., min(end, selected[i + 1]['source_time']
            if i + 1 < len(selected) else end) - sample['source_time']))
        for i, sample in enumerate(selected))
    stats['observed_duration'] = coverage
    stats['uncovered_duration'] = max(0., end - start - coverage)
    gaps = np.diff([sample['source_time'] for sample in selected])
    stats['dropout_intervals'] = int(np.sum(gaps > max_skew))
    stats['max_gap'] = float(np.max(gaps)) if gaps.size else None
    ages = [float(sample['clock']) - sample['source_time'] for sample in selected
            if sample.get('clock') is not None]
    stats['future_samples'] = sum(age < -1e-9 for age in ages)
    stats['freshness'] = statistics([age for age in ages if age >= -1e-9], 's',
                                    'no simultaneous simulation clock observations')
    stats['availability'] = 'available' if selected else 'unavailable'
    return accepted, selected, stats


class SourceIndex:
    """Bounded nearest samples and bracket-only vector interpolation."""

    def __init__(self, records, frame=None):
        self.frame = frame
        self.records = [r for r in records if frame is None or (
            bool(frame) and r.get('frame_id') == frame)]
        self.times = [r['source_time'] for r in self.records]

    def nearest(self, time, max_skew):
        index = bisect_left(self.times, time)
        choices = [i for i in [index - 1, index] if 0 <= i < len(self.times)]
        if not choices:
            raise ValueError('missing samples or incompatible frame')
        index = min(choices, key=lambda i: abs(self.times[i] - time))
        if abs(self.times[index] - time) > max_skew + 1e-12:
            raise ValueError('source-time alignment exceeds max_skew')
        return self.records[index]

    def interpolate_position(self, time, max_skew):
        index = bisect_left(self.times, time)
        if index < len(self.times) and abs(self.times[index] - time) < 1e-10:
            return finite_vector(self.records[index]['data']['position'], 3)
        if index == 0 or index == len(self.times):
            raise ValueError('truth extrapolation forbidden')
        low, high = self.records[index - 1], self.records[index]
        t0, t1 = low['source_time'], high['source_time']
        if max(time - t0, t1 - time) > max_skew + 1e-12:
            raise ValueError('truth bracket exceeds max_skew')
        a = finite_vector(low['data']['position'], 3)
        b = finite_vector(high['data']['position'], 3)
        return a + (b - a) * ((time - t0) / (t1 - t0))
