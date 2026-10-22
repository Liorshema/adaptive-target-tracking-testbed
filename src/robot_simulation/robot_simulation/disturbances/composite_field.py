
"""Composition of multiple acceleration fields."""

from collections.abc import Sequence

import numpy as np

from robot_simulation.interfaces.disturbance import AccelerationField, ContextualAccelerationField


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
        return self.acceleration_with_context(position, time, None)

    @property
    def context_requirements(self):
        """Declare the union of context dependencies for deployment preflight."""
        return frozenset().union(*(getattr(f, "context_requirements", frozenset())
                                   for f in self.fields))

    def acceleration_with_context(self, position, time, context):
        """Sum ordinary fields and explicitly contextual fields."""
        position = np.asarray(position, dtype=float)

        if position.ndim != 1 or not np.all(np.isfinite(position)):
            raise ValueError("Position must be a finite vector.")

        total = np.zeros_like(position)

        for field in self.fields:
            contribution = np.asarray(
                (field.acceleration_with_context(position, time, context)
                 if isinstance(field, ContextualAccelerationField)
                 else field.acceleration(position, time)),
                dtype=float,
            )

            if contribution.shape != position.shape:
                raise ValueError("Acceleration field dimension mismatch.")

            if not np.all(np.isfinite(contribution)):
                raise ValueError("Acceleration field must be finite.")

            total += contribution

        return total
