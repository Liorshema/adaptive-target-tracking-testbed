"""Bounded causal reordering for observations feeding one shared filter."""

from collections import Counter, defaultdict
import heapq
import math


class TimestampBuffer:
    """Commit observations behind a clock watermark; never rewind a filter."""

    def __init__(self, holdback, max_age, future_tolerance, capacity):
        """Configure seconds and a maximum number of pending observations."""
        if (not all(math.isfinite(v) for v in [holdback, max_age, future_tolerance]) or
                not 0 <= holdback < max_age or future_tolerance < 0 or
                not isinstance(capacity, int) or capacity <= 0):
            raise ValueError('Invalid observation reordering budget')
        self.holdback = holdback
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.capacity = capacity
        self.counters = defaultdict(Counter)
        self.resets = 0
        self._clear()

    def _clear(self):
        self._heap = []
        self._pending = set()
        self._last_source = {}
        self._sequence = 0
        self.committed = None
        self.latest_received = None

    def reset(self):
        """Discard former-epoch observations and watermarks, retaining diagnostics."""
        self._clear()
        self.resets += 1

    def push(self, source, stamp, payload, now):
        """Validate receipt time without applying an observation at receipt time."""
        counter = self.counters[source]
        counter['received'] += 1
        reason = None
        if not math.isfinite(stamp) or stamp < 0 or not math.isfinite(now):
            reason = 'invalid'
        elif stamp > now + self.future_tolerance:
            reason = 'future'
        elif now - stamp > self.max_age:
            reason = 'stale'
        elif (source, stamp) in self._pending or self._last_source.get(source) == stamp:
            reason = 'duplicate'
        elif self.committed is not None and stamp < self.committed:
            reason = 'late'
        elif len(self._heap) >= self.capacity:
            reason = 'overflow'
        if reason:
            counter[reason] += 1
            return False
        if self.latest_received is not None and stamp < self.latest_received:
            counter['delayed'] += 1
        self.latest_received = max(stamp, self.latest_received or stamp)
        counter['arrival_age_sum_s'] += now - stamp
        counter['arrival_age_max_s'] = max(counter['arrival_age_max_s'], now - stamp)
        counter['queued'] += 1
        heapq.heappush(self._heap, (stamp, self._sequence, source, payload, now))
        self._pending.add((source, stamp))
        self._sequence += 1
        return True

    def pop_ready(self, now):
        """Return (source, timestamp, payload) in stable measurement-time order."""
        if not math.isfinite(now):
            raise ValueError('Buffer clock must be finite')
        ready = []
        while self._heap and self._heap[0][0] <= now - self.holdback + 1e-12:
            stamp, _, source, payload, arrival = heapq.heappop(self._heap)
            self._pending.remove((source, stamp))
            counter = self.counters[source]
            if now - stamp > self.max_age:
                counter['stale'] += 1
                continue
            if self.committed is not None and stamp < self.committed:
                counter['late'] += 1
                continue
            self.committed = stamp
            self._last_source[source] = stamp
            counter['accepted'] += 1
            counter['buffer_delay_sum_s'] += now - arrival
            counter['buffer_delay_max_s'] = max(counter['buffer_delay_max_s'], now - arrival)
            counter['processing_age_max_s'] = max(counter['processing_age_max_s'], now - stamp)
            ready.append((source, stamp, payload))
        return ready

    def diagnostics(self):
        """Expose counters and pending depth without leaking queued observations."""
        return {
            'pending': len(self._heap), 'committed': self.committed, 'resets': self.resets,
            'sources': {source: dict(counts) for source, counts in self.counters.items()},
        }
