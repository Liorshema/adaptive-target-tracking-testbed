import numpy as np

from robot_prediction.residuals.residual_statistics import (
    ResidualStatistics,
)


class PredictionConfidence:
    """Compute prediction confidence from residual statistics."""

    def __init__(
        self,
        scale: float,
    ):
        if scale <= 0.0:
            raise ValueError(
                'scale must be positive.'
            )

        self._scale = scale

    def compute(
        self,
        statistics: ResidualStatistics,
    ) -> float:
        """Compute confidence from residual RMS."""
        rms_error = np.linalg.norm(
            statistics.rms
        )

        confidence = np.exp(
            -0.5
            * (rms_error / self._scale) ** 2
        )

        return float(confidence)
