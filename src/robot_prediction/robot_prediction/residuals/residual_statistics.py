import numpy as np


class ResidualStatistics:
    """Compute statistics over a collection of prediction residuals."""

    def __init__(
        self,
        residuals: np.ndarray,
    ):
        residuals = np.asarray(
            residuals,
            dtype=float,
        )

        if residuals.ndim != 2:
            raise ValueError(
                'residuals must have shape (N, state_dimension).'
            )

        if residuals.shape[0] == 0:
            raise ValueError(
                'residuals must contain at least one sample.'
            )

        self._residuals = residuals.copy()

    @property
    def mean(self) -> np.ndarray:
        """Return mean residual vector."""
        return np.mean(
            self._residuals,
            axis=0,
        )

    @property
    def bias(self) -> np.ndarray:
        """Return systematic prediction bias."""
        return self.mean

    @property
    def covariance(self) -> np.ndarray:
        """Return sample covariance matrix of residuals."""
        state_dimension = self._residuals.shape[1]

        if self._residuals.shape[0] < 2:
            return np.zeros(
                (state_dimension, state_dimension),
                dtype=float,
            )

        centered = (
            self._residuals
            - self.mean
        )

        return (
            centered.T
            @ centered
            / (self._residuals.shape[0] - 1)
        )

    @property
    def rms(self) -> np.ndarray:
        """Return RMS residual for each state component."""
        return np.sqrt(
            np.mean(
                self._residuals ** 2,
                axis=0,
            )
        )

    @property
    def count(self) -> int:
        """Return number of residual samples."""
        return self._residuals.shape[0]
