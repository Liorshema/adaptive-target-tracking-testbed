"""Measured-initialized, bounded velocity-to-position reference integration."""

import numpy as np


class PositionReference:
    """Integrate joint velocities without accumulating unreachable references."""

    def __init__(self, lower, upper, velocity_limit, max_lead, max_dt):
        """Configure physical limits and the allowed lead over measured motion."""
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.velocity_limit = np.asarray(velocity_limit, dtype=float)
        self.max_lead = float(max_lead)
        self.max_dt = float(max_dt)
        if (self.lower.ndim != 1 or not self.lower.size or
                self.upper.shape != self.lower.shape or
                self.velocity_limit.shape != self.lower.shape or
                not np.all(np.isfinite(np.r_[self.lower, self.upper,
                                             self.velocity_limit, max_lead, max_dt])) or
                np.any(self.lower >= self.upper) or
                np.any(self.velocity_limit <= 0) or max_lead <= 0 or max_dt <= 0):
            raise ValueError('Invalid position-reference limits')
        self.position = None

    def _vector(self, values):
        result = np.asarray(values, dtype=float)
        if result.shape != self.lower.shape or not np.all(np.isfinite(result)):
            raise ValueError('Joint vector must have the configured finite shape')
        return result

    def initialize(self, measured):
        """Start from fresh measured joints, clamped to physical position limits."""
        self.position = np.clip(self._vector(measured), self.lower, self.upper)

    def step(self, velocity, measured, dt):
        """Advance only with positive bounded clock time and safe velocities."""
        velocity = self._vector(velocity)
        measured = self._vector(measured)
        if (self.position is None or not np.isfinite(dt) or
                not 0 < dt <= self.max_dt or
                np.any(np.abs(velocity) > self.velocity_limit + 1e-9)):
            raise ValueError('Uninitialized reference, unsafe velocity or clock interval')
        lower = np.maximum(self.lower, measured - self.max_lead)
        upper = np.minimum(self.upper, measured + self.max_lead)
        if np.any(lower > upper):
            raise ValueError('Measured joints are outside the permitted physical envelope')
        candidate = np.clip(self.position + dt * velocity, lower, upper)
        if np.any(np.abs(candidate - self.position) > dt * self.velocity_limit + 1e-9):
            raise ValueError('Measured lead clipping would create a reference discontinuity')
        self.position = candidate
        return self.position.copy()
