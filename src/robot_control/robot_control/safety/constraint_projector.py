
"""Projection of generalized velocity commands onto box constraints.

This module is independent of robot models, ROS, Gazebo,
and specific control algorithms.
"""

from collections.abc import Iterable

import numpy as np
from numpy.typing import NDArray

from robot_control.interfaces.control_data import ControlMode, ControlOutput

Array = NDArray[np.float64]
Bounds = tuple[Array, Array]


def _vector(value: Array, name: str) -> Array:
    """Validate and copy a finite one-dimensional vector."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.ndim != 1 or result.size == 0:
        raise ValueError(f"{name} must be a nonempty 1D vector")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")

    return result


class ConstraintProjector:
    """Project velocity commands onto intersecting box constraints.

    Each constraint supplies lower and upper bounds:

        lower <= velocity <= upper

    The projection is the Euclidean nearest admissible command.

    No physical constraint types or robot dimensions are hard-coded.
    """

    def project(
        self,
        output: ControlOutput,
        bounds: Iterable[Bounds],
    ) -> ControlOutput:
        """Project a requested velocity onto all supplied bounds.

        Args:
            output: Requested generalized velocity command.
            bounds: Iterable of (lower, upper) vector pairs.

        Returns:
            A new ControlOutput containing the projected command.

        Raises:
            TypeError: If the output type is unsupported.
            ValueError: If bounds are invalid or infeasible.
        """
        if not isinstance(output, ControlOutput):
            raise TypeError("output must be ControlOutput")

        if output.mode != ControlMode.VELOCITY:
            raise ValueError(
                "ConstraintProjector supports velocity mode only"
            )

        requested = _vector(output.command, "command")
        dimension = requested.size

        lower = np.full(dimension, -np.inf)
        upper = np.full(dimension, np.inf)

        constraint_count = 0

        for index, (lower_bound, upper_bound) in enumerate(bounds):
            current_lower = _vector(
                lower_bound,
                f"bounds[{index}].lower",
            )
            current_upper = _vector(
                upper_bound,
                f"bounds[{index}].upper",
            )

            if (
                current_lower.shape != requested.shape
                or current_upper.shape != requested.shape
            ):
                raise ValueError(
                    "Constraint dimensions must match command"
                )

            if np.any(current_lower > current_upper):
                raise ValueError(
                    "Infeasible constraints: "
                    f"empty interval at index {index}"
                )

            lower = np.maximum(lower, current_lower)
            upper = np.minimum(upper, current_upper)

            constraint_count += 1

        if constraint_count == 0:
            raise ValueError("At least one constraint is required")

        if np.any(lower > upper):
            raise ValueError(
                "Infeasible constraints: empty intersection"
            )

        command = np.clip(requested, lower, upper)

        return ControlOutput(
            command=command,
            mode=ControlMode.VELOCITY,
        )
