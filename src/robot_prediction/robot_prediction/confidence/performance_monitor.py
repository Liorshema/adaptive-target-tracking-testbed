from dataclasses import dataclass

import numpy as np

from robot_prediction.confidence.prediction_confidence import (
    PredictionConfidence,
)
from robot_prediction.history.residual_buffer import ResidualBuffer
from robot_prediction.residuals.prediction_residual import (
    compute_residual,
)
from robot_prediction.residuals.residual_statistics import (
    ResidualStatistics,
)


@dataclass(frozen=True)
class PerformanceSnapshot:
    """Store current prediction-performance metrics."""

    residual: np.ndarray
    mean_residual: np.ndarray
    rms_residual: np.ndarray
    covariance: np.ndarray
    confidence: float
    sample_count: int


class PerformanceMonitor:
    """Track recent prediction performance."""

    def __init__(
        self,
        buffer_capacity: int,
        confidence_scale: float,
    ):
        self._residual_buffer = ResidualBuffer(
            capacity=buffer_capacity
        )

        self._confidence_model = PredictionConfidence(
            scale=confidence_scale
        )

    def update(
        self,
        observed_state: np.ndarray,
        predicted_state: np.ndarray,
    ) -> PerformanceSnapshot:
        """Update performance metrics from a new prediction outcome."""
        residual = compute_residual(
            observed_state=observed_state,
            predicted_state=predicted_state,
        )

        self._residual_buffer.append(
            residual
        )

        statistics = self.statistics

        confidence = self._confidence_model.compute(
            statistics
        )

        return PerformanceSnapshot(
            residual=residual.copy(),
            mean_residual=statistics.mean.copy(),
            rms_residual=statistics.rms.copy(),
            covariance=statistics.covariance.copy(),
            confidence=confidence,
            sample_count=statistics.count,
        )

    def clear(
        self,
    ) -> None:
        """Clear stored performance history."""
        self._residual_buffer.clear()

    @property
    def statistics(
        self,
    ) -> ResidualStatistics:
        """Return statistics for the current residual history."""
        if self._residual_buffer.size == 0:
            raise RuntimeError(
                'no residual samples are available.'
            )

        return ResidualStatistics(
            residuals=self._residual_buffer.as_array()
        )

    @property
    def sample_count(
        self,
    ) -> int:
        """Return the number of stored residual samples."""
        return self._residual_buffer.size

    @property
    def is_full(
        self,
    ) -> bool:
        """Return whether the residual history is full."""
        return self._residual_buffer.is_full
        """Return whether the residual history is full."""
        return self._residual_buffer.is_full
