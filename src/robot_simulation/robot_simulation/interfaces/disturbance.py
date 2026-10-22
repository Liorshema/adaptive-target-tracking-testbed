
"""Interface for external acceleration fields."""

from typing import Protocol, runtime_checkable

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


@runtime_checkable
class ContextualAccelerationField(Protocol):
    """Explicit opt-in contract; existing position/time-only fields are unchanged."""

    def acceleration_with_context(self, position, time, context) -> np.ndarray:
        """Evaluate against a typed physical source sample."""
        ...
