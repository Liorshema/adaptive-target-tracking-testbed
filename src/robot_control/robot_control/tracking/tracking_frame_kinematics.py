
"""Rigid tracking-frame kinematics.

Convert end-effector pose and world-frame geometric Jacobian
into pose and Jacobian of an attached tracking frame.

This module is independent of ROS, Gazebo, and robot geometry.
"""

import numpy as np
from numpy.typing import NDArray

from robot_models.common.rotations import skew

Array = NDArray[np.float64]


def _validate_transform(transform: Array, name: str) -> Array:
    """Validate a rigid homogeneous transform."""
    result = np.asarray(
        transform,
        dtype=np.float64,
    )

    if (
        result.shape != (4, 4)
        or not np.all(np.isfinite(result))
    ):
        raise ValueError(
            f"{name} must be a finite 4x4 transform"
        )

    if not np.allclose(
        result[3, :],
        [0.0, 0.0, 0.0, 1.0],
        atol=1e-8,
        rtol=0.0,
    ):
        raise ValueError(
            f"{name} has an invalid homogeneous row"
        )

    rotation = result[:3, :3]

    if (
        not np.allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1e-6,
            rtol=0.0,
        )
        or not np.isclose(
            np.linalg.det(rotation),
            1.0,
            atol=1e-6,
            rtol=0.0,
        )
    ):
        raise ValueError(
            f"{name} must contain a proper rotation"
        )

    return result.copy()


class TrackingFrameKinematics:
    """Shift end-effector kinematics to a rigid tracking frame.

    Twist convention:
        [linear_velocity_world, angular_velocity_world]

    The offset is expressed in the end-effector coordinate frame.
    """

    def __init__(
        self,
        transform_end_effector_tracking: Array,
    ) -> None:
        """Configure the fixed transform T_E_T."""
        self._transform_et = _validate_transform(
            transform_end_effector_tracking,
            "transform_end_effector_tracking",
        )

    def transform_pose(
        self,
        transform_world_end_effector: Array,
    ) -> Array:
        """Return the world pose of the attached tracking frame."""
        transform_we = _validate_transform(
            transform_world_end_effector,
            "transform_world_end_effector",
        )

        return transform_we @ self._transform_et

    def compute(
        self,
        transform_world_end_effector: Array,
        jacobian_world_end_effector: Array,
    ) -> tuple[Array, Array]:
        """Return tracking-frame world pose and geometric Jacobian.

        Args:
            transform_world_end_effector: Current T_W_E.
            jacobian_world_end_effector: World-frame 6xN Jacobian
                with linear velocity in the first three rows.

        Returns:
            (T_W_T, J_W_T), where J_W_T has shape (6, N).
        """
        transform_we = _validate_transform(
            transform_world_end_effector,
            "transform_world_end_effector",
        )

        jacobian = np.asarray(
            jacobian_world_end_effector,
            dtype=np.float64,
        )

        if (
            jacobian.ndim != 2
            or jacobian.shape[0] != 6
            or not np.all(np.isfinite(jacobian))
        ):
            raise ValueError(
                "jacobian_world_end_effector must be finite 6xN"
            )

        # Tracking-frame pose in world coordinates.
        transform_wt = self.transform_pose(transform_we)

        # Offset from end effector to tracking frame,
        # expressed in world coordinates.
        offset_world = (
            transform_we[:3, :3]
            @ self._transform_et[:3, 3]
        )

        # Shift the linear Jacobian to the tracking-frame origin.
        # Angular Jacobian remains unchanged.
        jacobian_tracking = jacobian.copy()

        jacobian_tracking[:3, :] = (
            jacobian[:3, :]
            - skew(offset_world) @ jacobian[3:, :]
        )

        return transform_wt, jacobian_tracking
