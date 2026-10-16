
"""Convert planar-base control states into kinematic model inputs.

The adapter assumes configuration coordinates [x, y, yaw, joints].
Generalized velocities remain in the RobotState contract.

This module does not depend on ROS, Gazebo, or control algorithms.
"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from robot_models.spatial.transforms import make_transform

from robot_control.interfaces.control_data import RobotState

Array = NDArray[np.float64]


@dataclass(frozen=True)
class KinematicState:
    """Kinematic inputs required by whole-body model evaluation."""

    transform_world_base: Array
    joint_positions: Array

    def __post_init__(self) -> None:
        """Validate and independently store kinematic quantities."""
        transform = np.array(
            self.transform_world_base,
            dtype=np.float64,
            copy=True,
        )
        joints = np.array(
            self.joint_positions,
            dtype=np.float64,
            copy=True,
        )

        if transform.shape != (4, 4):
            raise ValueError("transform_world_base must have shape (4, 4)")

        if joints.ndim != 1:
            raise ValueError("joint_positions must be a 1D vector")

        if not np.all(np.isfinite(transform)):
            raise ValueError("transform_world_base must be finite")

        if not np.all(np.isfinite(joints)):
            raise ValueError("joint_positions must be finite")

        if not np.allclose(
            transform[3, :],
            [0.0, 0.0, 0.0, 1.0],
            atol=1e-8,
            rtol=0.0,
        ):
            raise ValueError("Invalid homogeneous transformation")

        rotation = transform[:3, :3]

        if not np.allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1e-6,
            rtol=0.0,
        ):
            raise ValueError("Rotation must be orthonormal")

        if not np.isclose(
            np.linalg.det(rotation),
            1.0,
            atol=1e-6,
            rtol=0.0,
        ):
            raise ValueError("Rotation must be proper")

        object.__setattr__(self, "transform_world_base", transform)
        object.__setattr__(self, "joint_positions", joints)


class PlanarKinematicsAdapter:
    """Interpret a planar mobile-base configuration for robot models.

    The base configuration is [x, y, yaw].
    Remaining configuration coordinates are manipulator joints.

    Joint count is supplied externally; robot geometry is not embedded.
    """

    BASE_CONFIGURATION_DIMENSION = 3
    BASE_VELOCITY_DIMENSION = 2

    def __init__(self, joint_count: int) -> None:
        """Configure the expected manipulator dimension."""
        if (
            isinstance(joint_count, (bool, np.bool_))
            or not isinstance(joint_count, (int, np.integer))
            or joint_count < 0
        ):
            raise ValueError("joint_count must be a nonnegative integer")

        self._joint_count = int(joint_count)

    def convert(self, state: RobotState) -> KinematicState:
        """Extract world-frame base pose and manipulator positions.

        Args:
            state: Generic control state using planar-base coordinates.

        Returns:
            KinematicState containing a 4x4 base pose and joint vector.

        Raises:
            TypeError: If state does not implement RobotState.
            ValueError: If state dimensions are inconsistent.
        """
        if not isinstance(state, RobotState):
            raise TypeError("state must be RobotState")

        expected_configuration = (
            self.BASE_CONFIGURATION_DIMENSION + self._joint_count
        )
        expected_velocity = (
            self.BASE_VELOCITY_DIMENSION + self._joint_count
        )

        if state.configuration.shape != (expected_configuration,):
            raise ValueError(
                "Robot configuration must have shape "
                f"({expected_configuration},)"
            )

        if state.velocity.shape != (expected_velocity,):
            raise ValueError(
                "Robot velocity must have shape "
                f"({expected_velocity},)"
            )

        x, y, yaw = state.configuration[:3]

        cosine = np.cos(yaw)
        sine = np.sin(yaw)

        rotation = np.array(
            [
                [cosine, -sine, 0.0],
                [sine, cosine, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        position = np.array(
            [x, y, 0.0],
            dtype=np.float64,
        )

        transform = make_transform(
            rotation=rotation,
            position=position,
        )

        return KinematicState(
            transform_world_base=transform,
            joint_positions=state.configuration[
                self.BASE_CONFIGURATION_DIMENSION:
            ],
        )
