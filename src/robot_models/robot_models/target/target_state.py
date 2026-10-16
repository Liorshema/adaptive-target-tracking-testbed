"""TargetState domain records with validated numerical shapes."""

from dataclasses import dataclass

import numpy as np



@dataclass
class TargetState:
    """Estimated target state in Cartesian space."""

    position: np.ndarray
    velocity: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(
            self.position,
            dtype=float,
        )

        self.velocity = np.asarray(
            self.velocity,
            dtype=float,
        )

        if self.position.shape != (3,):
            raise ValueError(
                'position must have shape (3,)'
            )

        if self.velocity.shape != (3,):
            raise ValueError(
                'velocity must have shape (3,)'
            )
