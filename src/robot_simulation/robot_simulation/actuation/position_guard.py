"""Pure physical position supervision; recovery never changes physical limits."""

import numpy as np


class PositionGuard:
    """Latch serious excursions and recover small ones with inward hysteresis."""

    def __init__(self, lower, upper, velocity_limit, *, numerical_tolerance,
                 recoverable_violation, recovery_speed, recovery_inset, release_margin):
        """Configure angles [rad] and recovery reference slew [rad/s]."""
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.speed = np.minimum(np.asarray(velocity_limit, dtype=float), recovery_speed)
        self.numerical_tolerance = float(numerical_tolerance)
        self.recoverable_violation = float(recoverable_violation)
        self.inset = float(recovery_inset)
        self.release_margin = float(release_margin)
        if (self.lower.ndim != 1 or not self.lower.size or
                self.upper.shape != self.lower.shape or self.speed.shape != self.lower.shape or
                not np.all(np.isfinite(np.r_[self.lower, self.upper, self.speed,
                                             numerical_tolerance, recoverable_violation,
                                             recovery_speed, recovery_inset, release_margin])) or
                not 0 <= numerical_tolerance < recoverable_violation or
                not 0 < release_margin < recovery_inset or recovery_speed <= 0 or
                np.any(self.speed <= 0) or
                np.any(2 * recovery_inset >= self.upper - self.lower)):
            raise ValueError('Invalid physical position recovery policy')
        self.side = np.zeros(self.lower.size, dtype=int)
        self.mode = 'normal'
        self.fault = None
        self.fault_positions = None
        self.events = 0

    def _vector(self, value):
        result = np.asarray(value, dtype=float)
        if result.shape != self.lower.shape or not np.all(np.isfinite(result)):
            raise ValueError('Position guard requires a finite joint vector')
        return result

    def observe(self, measured, *, release_boundary=False):
        """Observe fresh feedback; a physical fault remains latched for this instance."""
        q = self._vector(measured)
        if self.fault is not None:
            return 'fault'
        excess = np.maximum(np.maximum(self.lower - q, q - self.upper), 0.)
        if np.any(excess > self.recoverable_violation):
            self.fault = 'Serious position violation (latched)'
            self.fault_positions = q.copy()
            self.mode = 'fault'
            return self.mode
        # Keep recovery until it has crossed an interior release margin.
        self.side[(self.side == 1) & (q >= self.lower + self.release_margin)] = 0
        self.side[(self.side == -1) & (q <= self.upper - self.release_margin)] = 0
        new_side = np.where(q < self.lower, 1, np.where(q > self.upper, -1, 0))
        if release_boundary:
            # A stopped estimator can sit infinitesimally outside a hard stop even
            # when measured q rounds to its legal side. This authorizes only an
            # inward release inside the same numerical band, never normal motion.
            new_side = np.where(q <= self.lower + self.numerical_tolerance, 1,
                                np.where(q >= self.upper - self.numerical_tolerance,
                                         -1, new_side))
        entering = (new_side != 0) & (self.side == 0)
        if np.any(entering):
            self.events += 1
            self.side[entering] = new_side[entering]
        if not np.any(self.side):
            self.mode = 'normal'
        elif np.any(excess > self.numerical_tolerance) or self.mode == 'recovery':
            self.mode = 'recovery'
        else:
            self.mode = 'numerical_recovery'
        return self.mode

    def step(self, reference, measured, dt):
        """Slew only affected legal references inward; the adapter owns freshness."""
        reference = self._vector(reference)
        if (not np.isfinite(dt) or dt <= 0 or
                np.any(reference < self.lower) or np.any(reference > self.upper)):
            raise ValueError('Recovery requires legal references and positive finite dt')
        if self.observe(measured) == 'fault':
            raise ValueError(self.fault)
        target = reference.copy()
        lower_side, upper_side = self.side == 1, self.side == -1
        target[lower_side] = np.maximum(reference, self.lower + self.inset)[lower_side]
        target[upper_side] = np.minimum(reference, self.upper - self.inset)[upper_side]
        return reference + np.clip(target - reference, -dt * self.speed, dt * self.speed)

    def diagnostic(self, names, measured):
        """Report measured excesses with externally supplied authoritative joint names."""
        q = self._vector(measured if self.fault_positions is None else self.fault_positions)
        indices = np.flatnonzero((q < self.lower) | (q > self.upper) | (self.side != 0))
        details = []
        for i in indices:
            excess = max(self.lower[i] - q[i], q[i] - self.upper[i], 0.)
            details.append(f'{names[i]} q={q[i]:.12g} rad bounds=['
                           f'{self.lower[i]:.12g},{self.upper[i]:.12g}] rad '
                           f'excess={excess:.6g} rad')
        return '; '.join(details)
