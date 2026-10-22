"""Causal physical base sample contract for context-dependent acceleration fields."""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PhysicalBaseContext:
    """World position held from a stamped physical source, never a static fallback."""

    position_world: np.ndarray
    timestamp: float
    reference_frame: str
    provenance: str
    max_age: float

    def __post_init__(self):
        """Validate and freeze the measured physical sample."""
        p = np.asarray(self.position_world, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('physical base position must be a finite 3-vector')
        if not np.isfinite([self.timestamp, self.max_age]).all() or self.max_age <= 0:
            raise ValueError('physical source timestamp/maximum age invalid')
        if not self.reference_frame or not self.provenance:
            raise ValueError('physical frame and provenance required')
        p = p.copy()
        p.flags.writeable = False
        object.__setattr__(self, 'position_world', p)

    def position_at(self, time, reference_frame):
        """Reject wrong frames, future data and expired zero-order-held samples."""
        age = time-self.timestamp
        if (not np.isfinite(time) or reference_frame != self.reference_frame
                or age < -1e-9 or age > self.max_age):
            raise ValueError('physical base context frame/time is unavailable')
        return self.position_world


class PhysicalPositionHistory:
    """Bounded source-time buffer selecting latest nonfuture physical position."""

    def __init__(self, reference_frame, provenance, max_age, history_duration=None):
        """Validate explicit constructor parameters."""
        self.reference_frame = reference_frame
        self.provenance = provenance
        self.max_age = max_age
        self.history_duration = 2*max_age if history_duration is None else history_duration
        if not np.isfinite(self.history_duration) or self.history_duration < max_age:
            raise ValueError('history duration must be finite and at least max_age')
        self._samples = []

    def add(self, position, timestamp, reference_frame):
        """Ignore wrong frames/out-of-order arrivals; caller clears on clock reset."""
        if reference_frame != self.reference_frame:
            return
        sample = PhysicalBaseContext(position, timestamp, reference_frame,
                                     self.provenance, self.max_age)
        if self._samples and timestamp <= self._samples[-1].timestamp:
            return
        self._samples.append(sample)
        self._samples = [s for s in self._samples
                         if timestamp - s.timestamp <= self.history_duration]

    def at(self, time):
        """Return causal fresh context or absence; never fabricate/extrapolate a pose."""
        for sample in reversed(self._samples):
            if sample.timestamp <= time:
                try:
                    sample.position_at(time, self.reference_frame)
                except ValueError:
                    return None
                return sample
        return None

    def clear(self):
        """Drop the previous source-time epoch."""
        self._samples.clear()
