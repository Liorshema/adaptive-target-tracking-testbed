from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


@dataclass
class Measurement:
    """
    Generic sensor measurement.

    Represents a measurement vector together with its measurement
    covariance and optional timestamp.

    Attributes
    ----------
    value:
        Measurement vector z with shape (m,).

    covariance:
        Measurement covariance matrix R with shape (m, m).

    timestamp:
        Optional measurement time in seconds.
    """

    value: Vector
    covariance: Matrix
    timestamp: Optional[float] = None

    def __post_init__(self) -> None:
        self.value = np.asarray(self.value, dtype=float)
        self.covariance = np.asarray(self.covariance, dtype=float)

        if self.value.ndim != 1:
            raise ValueError(
                f"Measurement value must be a 1D vector, got shape {self.value.shape}."
            )

        m = self.value.shape[0]

        if self.covariance.shape != (m, m):
            raise ValueError(
                "Measurement covariance must have shape "
                f"({m}, {m}), got {self.covariance.shape}."
            )

    @property
    def dimension(self) -> int:
        """Return the dimension of the measurement vector."""
        return self.value.shape[0]

    def copy(self) -> "Measurement":
        """Return a deep copy of the measurement."""
        return Measurement(
            value=self.value.copy(),
            covariance=self.covariance.copy(),
            timestamp=self.timestamp,
        )