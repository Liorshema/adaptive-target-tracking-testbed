
"""Shared state of the target simulation."""

from dataclasses import dataclass

import numpy as np

from robot_models.types import TargetState


@dataclass(frozen=True)
class SimulationState:
    """Snapshot of target motion and tracking-frame position."""

    target: TargetState
    tracking_position_world: np.ndarray
    time: float

    def __post_init__(self) -> None:
        if not isinstance(self.target, TargetState):
            raise TypeError("target must be a TargetState.")

        tracking_position = np.asarray(
            self.tracking_position_world, dtype=float
        )

        if tracking_position.shape != self.target.position.shape:
            raise ValueError("Tracking position dimension mismatch.")

        if not np.all(np.isfinite(tracking_position)):
            raise ValueError("Tracking position must be finite.")

        if not np.all(np.isfinite(self.target.position)):
            raise ValueError("Target position must be finite.")

        if not np.all(np.isfinite(self.target.velocity)):
            raise ValueError("Target velocity must be finite.")

        if not np.isfinite(self.time):
            raise ValueError("Simulation time must be finite.")

        object.__setattr__(
            self,
            "tracking_position_world",
            tracking_position.copy(),
        )

        object.__setattr__(
            self,
            "target",
            TargetState(
                position=self.target.position.copy(),
                velocity=self.target.velocity.copy(),
            ),
        )
