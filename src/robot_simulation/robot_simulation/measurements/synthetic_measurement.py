
"""Generic linear synthetic measurement generation."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


@dataclass(frozen=True)
class SyntheticMeasurement:
    """A simulated observation with uncertainty and timestamp."""

    value: Vector
    covariance: Matrix
    timestamp: float


class SyntheticMeasurementGenerator:
    """Generate noisy linear observations of a state vector.

    Measurement model:
        z = H @ x + v
        v ~ N(0, R)

    The generator is independent of ROS, Gazebo,
    target dynamics, and state dimensions.
    """

    def __init__(
        self,
        observation_matrix: Matrix,
        covariance: Matrix,
        rng: np.random.Generator,
    ) -> None:
        self._H = np.asarray(
            observation_matrix, dtype=float
        ).copy()
        self._R = np.asarray(
            covariance, dtype=float
        ).copy()

        if self._H.ndim != 2 or 0 in self._H.shape:
            raise ValueError("H must be a nonempty 2D matrix.")

        measurement_dim = self._H.shape[0]

        if self._R.shape != (measurement_dim, measurement_dim):
            raise ValueError("Covariance dimensions do not match H.")

        if not (
            np.all(np.isfinite(self._H))
            and np.all(np.isfinite(self._R))
        ):
            raise ValueError("Matrices must contain finite values.")

        if not np.allclose(self._R, self._R.T):
            raise ValueError("Covariance must be symmetric.")

        if np.linalg.eigvalsh(self._R).min() < -1e-10:
            raise ValueError(
                "Covariance must be positive semidefinite."
            )

        if not isinstance(rng, np.random.Generator):
            raise TypeError("rng must be a NumPy Generator.")

        self._rng = rng

    def measure(
        self,
        state: Vector,
        timestamp: float,
    ) -> SyntheticMeasurement:
        """Generate an observation from the supplied true state."""
        x = np.asarray(state, dtype=float)

        if x.shape != (self._H.shape[1],):
            raise ValueError(
                f"Expected state shape ({self._H.shape[1]},), "
                f"got {x.shape}."
            )

        if not np.all(np.isfinite(x)):
            raise ValueError("State must contain finite values.")

        if not np.isfinite(timestamp):
            raise ValueError("Timestamp must be finite.")

        mean = self._H @ x

        noise = self._rng.multivariate_normal(
            mean=np.zeros(self._H.shape[0]),
            cov=self._R,
        )

        return SyntheticMeasurement(
            value=mean + noise,
            covariance=self._R.copy(),
            timestamp=float(timestamp),
        )
