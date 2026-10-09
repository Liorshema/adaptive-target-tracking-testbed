
"""Interface for external acceleration fields."""

from typing import Protocol

import numpy as np


class AccelerationField(Protocol):
    """Interface for position-dependent acceleration fields."""

    def acceleration(
        self,
        position: np.ndarray,
        time: float = 0.0,
    ) -> np.ndarray:
        """Compute acceleration at a given position and time."""
        ...
