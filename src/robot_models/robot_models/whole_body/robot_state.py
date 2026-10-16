"""RobotState domain records with validated numerical shapes."""

from dataclasses import dataclass

import numpy as np

from robot_models.spatial.spatial_data import Pose, Twist


@dataclass
class RobotState:
    """Estimated state of the mobile manipulator."""

    base_pose: Pose
    base_twist: Twist
    joint_positions: np.ndarray
    joint_velocities: np.ndarray

    def __post_init__(self) -> None:
        self.joint_positions = np.asarray(
            self.joint_positions,
            dtype=float,
        )

        self.joint_velocities = np.asarray(
            self.joint_velocities,
            dtype=float,
        )

        if self.joint_positions.ndim != 1:
            raise ValueError(
                'joint_positions must be a 1D vector'
            )

        if self.joint_velocities.shape != self.joint_positions.shape:
            raise ValueError(
                'joint_velocities must match joint_positions shape'
            )
