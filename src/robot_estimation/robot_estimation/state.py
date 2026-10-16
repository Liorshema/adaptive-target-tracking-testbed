from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


@dataclass
class StateEstimate:
    """
    Generic state estimate.

    Represents an estimated state vector together with its covariance
    and optional timestamp.

    Attributes
    ----------
    mean:
        Estimated state vector x_hat with shape (n,).

    covariance:
        State covariance matrix P with shape (n, n).

    timestamp:
        Optional measurement/state time in seconds.
    """

    mean: Vector
    covariance: Matrix
    timestamp: Optional[float] = None

    def __post_init__(self) -> None:
        self.mean = np.asarray(self.mean, dtype=float)
        self.covariance = np.asarray(self.covariance, dtype=float)

        if self.mean.ndim != 1:
            raise ValueError(
                f"State mean must be a 1D vector, got shape {self.mean.shape}."
            )

        n = self.mean.shape[0]

        if self.covariance.shape != (n, n):
            raise ValueError(
                "State covariance must have shape "
                f"({n}, {n}), got {self.covariance.shape}."
            )

    @property
    def dimension(self) -> int:
        """Return the dimension of the state vector."""
        return self.mean.shape[0]

    def copy(self) -> "StateEstimate":
        """Return a deep copy of the state estimate."""
        return StateEstimate(
            mean=self.mean.copy(),
            covariance=self.covariance.copy(),
            timestamp=self.timestamp,
        )