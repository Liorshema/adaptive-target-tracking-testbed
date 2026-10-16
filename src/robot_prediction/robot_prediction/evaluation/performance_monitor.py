from dataclasses import dataclass

import numpy as np

from robot_prediction.evaluation.prediction_confidence import (
    PredictionConfidence,
)
from robot_prediction.history.residual_buffer import ResidualBuffer
from robot_prediction.evaluation.prediction_residual import (
    compute_residual,
)
from robot_prediction.evaluation.residual_statistics import (
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
        self._baseline_residual_buffer = ResidualBuffer(
            capacity=buffer_capacity
        )

        self._confidence_model = PredictionConfidence(
            scale=confidence_scale
        )

    def update(
        self,
        observed_state: np.ndarray,
        predicted_state: np.ndarray,
        baseline_predicted_state: np.ndarray | None = None,
    ) -> PerformanceSnapshot:
        """Update performance metrics from a new prediction outcome."""
        residual = compute_residual(
            observed_state=observed_state,
            predicted_state=predicted_state,
        )
        baseline_residual = (
            residual
            if baseline_predicted_state is None
            else compute_residual(
                observed_state=observed_state,
                predicted_state=baseline_predicted_state,
            )
        )

        self._residual_buffer.append(
            residual
        )
        self._baseline_residual_buffer.append(baseline_residual)

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
        self._baseline_residual_buffer.clear()

    @property
    def baseline_statistics(self) -> ResidualStatistics:
        """Return uncorrected forecast errors for residual-bias adaptation."""
        if self._baseline_residual_buffer.size == 0:
            raise RuntimeError('no baseline residual samples are available.')
        return ResidualStatistics(
            residuals=self._baseline_residual_buffer.as_array()
        )

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
