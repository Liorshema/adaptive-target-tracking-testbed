
"""Convert target states into Gazebo-compatible pose data."""

from dataclasses import dataclass

import numpy as np

from robot_models.types import TargetState


@dataclass(frozen=True)
class TargetPose:
    """Position and orientation of a simulated target."""

    position: np.ndarray
    orientation_xyzw: np.ndarray


class GazeboTargetAdapter:
    """Convert target state into a pose for Gazebo."""

    def __init__(
        self,
        orientation_xyzw: np.ndarray | None = None,
    ) -> None:
        if orientation_xyzw is None:
            orientation_xyzw = np.array(
                [0.0, 0.0, 0.0, 1.0],
                dtype=float,
            )

        orientation = np.asarray(
            orientation_xyzw,
            dtype=float,
        )

        if orientation.shape != (4,):
            raise ValueError("Orientation must have shape (4,).")

        if not np.all(np.isfinite(orientation)):
            raise ValueError("Orientation must be finite.")

        norm = np.linalg.norm(orientation)

        if norm <= 0.0:
            raise ValueError("Orientation quaternion cannot be zero.")

        self.orientation_xyzw = orientation / norm

    def to_pose(self, state: TargetState) -> TargetPose:
        """Construct a target pose from its estimated world position."""
        position = np.asarray(state.position, dtype=float)

        if position.shape != (3,):
            raise ValueError("Target position must have shape (3,).")

        if not np.all(np.isfinite(position)):
            raise ValueError("Target position must be finite.")

        return TargetPose(
            position=position.copy(),
            orientation_xyzw=self.orientation_xyzw.copy(),
        )
