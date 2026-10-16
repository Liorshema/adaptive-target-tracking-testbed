"""Joint-bound information as Gaussian inequality conditioning, never mean-only clipping."""

import numpy as np

from robot_estimation.common.state import StateEstimate

from scipy.stats import truncnorm


class JointBoundsConditioner:
    """Condition active marginal violations and their full correlated state moments."""

    def __init__(self, indices, lower, upper, numerical_tolerance, feedback_max_age):
        """Take model-order indices, physical angular bounds and explicit time budget."""
        self.indices = np.asarray(indices, dtype=int)
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        if (self.indices.ndim != 1 or not self.indices.size or
                len(set(self.indices)) != len(self.indices) or np.any(self.indices < 0) or
                self.lower.shape != self.indices.shape or self.upper.shape != self.indices.shape or
                not np.all(np.isfinite(np.r_[self.lower, self.upper,
                                             numerical_tolerance, feedback_max_age])) or
                np.any(self.lower >= self.upper) or numerical_tolerance < 0 or
                feedback_max_age <= 0):
            raise ValueError('Invalid joint-bound conditioning configuration')
        self.tolerance = numerical_tolerance
        self.max_age = feedback_max_age
        self.positions = None
        self.timestamp = None
        self.excess = np.zeros(len(self.indices))
        self.conditionings = 0
        self.physical_violation_samples = 0

    @property
    def physical_valid(self):
        """Expose true raw excursions rather than legitimizing a projected estimate."""
        return self.positions is not None and bool(np.all(self.excess <= self.tolerance))

    def observe(self, positions, timestamp):
        """Record the encoder at its committed measurement time."""
        q = np.asarray(positions, dtype=float)
        if q.shape != self.lower.shape or not np.all(np.isfinite(q)):
            raise ValueError('Invalid physical joint observation')
        self.positions = q.copy()
        self.timestamp = timestamp
        self.excess = np.maximum(np.maximum(self.lower - q, q - self.upper), 0.)
        self.physical_violation_samples += int(not self.physical_valid)

    def __call__(self, state):
        """Apply exact scalar truncated moments with a sequential Gaussian approximation."""
        if (self.timestamp is None or state.timestamp is None or
                not 0 <= state.timestamp - self.timestamp <= self.max_age):
            return state
        result = state.copy()
        for j, i in enumerate(self.indices):
            if (self.excess[j] > self.tolerance or
                    self.lower[j] <= result.mean[i] <= self.upper[j]):
                continue
            variance = result.covariance[i, i]
            if not np.isfinite(variance) or variance <= 0:
                raise ValueError('Cannot condition an infeasible zero-variance joint prior')
            sigma = np.sqrt(variance)
            a = (self.lower[j] - result.mean[i]) / sigma
            b = (self.upper[j] - result.mean[i]) / sigma
            standardized_mean, standardized_variance = truncnorm.stats(a, b, moments='mv')
            if (not np.isfinite(standardized_mean) or
                    not 0 < standardized_variance <= 1. + 1e-10):
                raise ValueError('Ill-conditioned truncated joint moments')
            bounded_mean = np.clip(result.mean[i] + sigma * standardized_mean,
                                   self.lower[j], self.upper[j])
            column = result.covariance[:, i].copy()
            result.mean += column * ((bounded_mean - result.mean[i]) / variance)
            # Remove only cancellation roundoff in the moment formula.
            result.mean[i] = bounded_mean
            result.covariance += np.outer(column, column) * (
                (variance * standardized_variance - variance) / variance**2)
            result.covariance = (result.covariance + result.covariance.T) / 2
            self.conditionings += 1
        return StateEstimate(result.mean, result.covariance, result.timestamp)
