import numpy as np

from robot_prediction.residuals.residual_statistics import (
    ResidualStatistics,
)


class ResidualAdapter:
    """Generate trajectory corrections from recent residual bias."""

    def __init__(
        self,
        correction_gain: float,
    ):
        if correction_gain < 0.0:
            raise ValueError(
                'correction_gain must be non-negative.'
            )

        self._correction_gain = correction_gain

    def compute_correction(
        self,
        statistics: ResidualStatistics,
        horizon: int,
    ) -> np.ndarray:
        """Generate a correction trajectory from residual bias."""
        if horizon <= 0:
            raise ValueError(
                'horizon must be positive.'
            )

        mean_residual = statistics.mean

        correction_vector = (
            self._correction_gain
            * mean_residual
        )

        correction = np.tile(
            correction_vector,
            (horizon, 1),
        )

        return correction
