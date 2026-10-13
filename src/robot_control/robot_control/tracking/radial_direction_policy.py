
"""Radial direction policy for spherical tracking references."""

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]


class RadialDirectionPolicy:
    """Select a spherical reference toward the current robot direction.

    The policy returns [radius, azimuth, elevation] and is
    intended for use with SphereReference.

    Both positions must be expressed in the same 3D frame.
    """

    def __init__(
        self,
        radius: float,
        epsilon: float = 1e-9,
    ) -> None:
        """Initialize the desired radius and singularity threshold."""
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError("radius must be positive and finite")

        if not np.isfinite(epsilon) or epsilon <= 0.0:
            raise ValueError("epsilon must be positive and finite")

        self._radius = float(radius)
        self._epsilon = float(epsilon)

    def select_parameters(
        self,
        current_position: Array,
        target_position: Array,
    ) -> Array:
        """Return radius, azimuth and elevation.

        Raises:
            ValueError: If positions are invalid or the
                direction is undefined.
        """
        current = np.asarray(
            current_position,
            dtype=np.float64,
        )
        target = np.asarray(
            target_position,
            dtype=np.float64,
        )

        if current.shape != (3,) or target.shape != (3,):
            raise ValueError("Positions must have shape (3,)")

        if not (
            np.all(np.isfinite(current))
            and np.all(np.isfinite(target))
        ):
            raise ValueError("Positions must be finite")

        direction = current - target
        distance = np.linalg.norm(direction)

        if distance <= self._epsilon:
            raise ValueError(
                "Radial direction is undefined near the target"
            )

        azimuth = np.arctan2(
            direction[1],
            direction[0],
        )

        elevation = np.arctan2(
            direction[2],
            np.linalg.norm(direction[:2]),
        )

        return np.array(
            [self._radius, azimuth, elevation],
            dtype=np.float64,
        )
