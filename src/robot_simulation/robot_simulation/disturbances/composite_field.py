
"""Composition of multiple acceleration fields."""

from collections.abc import Sequence

import numpy as np

from robot_simulation.interfaces.disturbance import AccelerationField


class CompositeAccelerationField:
    """Sum accelerations from independently configured fields."""

    def __init__(
        self,
        fields: Sequence[AccelerationField],
    ) -> None:
        self.fields = tuple(fields)

    def acceleration(
        self,
        position: np.ndarray,
        time: float = 0.0,
    ) -> np.ndarray:
        """Evaluate and sum all acceleration contributions."""
        position = np.asarray(position, dtype=float)

        if position.ndim != 1 or not np.all(np.isfinite(position)):
            raise ValueError("Position must be a finite vector.")

        total = np.zeros_like(position)

        for field in self.fields:
            contribution = np.asarray(
                field.acceleration(position, time),
                dtype=float,
            )

            if contribution.shape != position.shape:
                raise ValueError("Acceleration field dimension mismatch.")

            if not np.all(np.isfinite(contribution)):
                raise ValueError("Acceleration field must be finite.")

            total += contribution

        return total
