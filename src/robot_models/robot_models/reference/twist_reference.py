"""Frame twist-reference utilities."""

import numpy as np

from robot_models.common.rotations import (
    rotation_log_vector,
)


class TwistReference:
    """Compute desired frame twist from consecutive poses."""

    @staticmethod
    def compute(
        transform_world_frame_current: np.ndarray,
        transform_world_frame_next: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Return desired 6D frame twist in the world frame."""
        transform_world_frame_current = np.asarray(
            transform_world_frame_current,
            dtype=float,
        )

        transform_world_frame_next = np.asarray(
            transform_world_frame_next,
            dtype=float,
        )

        if transform_world_frame_current.shape != (4, 4):
            raise ValueError(
                'transform_world_frame_current must have shape (4, 4)'
            )

        if transform_world_frame_next.shape != (4, 4):
            raise ValueError(
                'transform_world_frame_next must have shape (4, 4)'
            )

        if dt <= 0.0:
            raise ValueError(
                'dt must be positive'
            )

        position_current = (
            transform_world_frame_current[:3, 3]
        )

        position_next = (
            transform_world_frame_next[:3, 3]
        )

        linear_velocity_world = (
            position_next - position_current
        ) / dt

        rotation_world_current = (
            transform_world_frame_current[:3, :3]
        )

        rotation_world_next = (
            transform_world_frame_next[:3, :3]
        )

        rotation_current_next = (
            rotation_world_current.T
            @ rotation_world_next
        )

        rotation_vector_current = (
            rotation_log_vector(
                rotation_current_next
            )
        )

        angular_velocity_world = (
            rotation_world_current
            @ rotation_vector_current
        ) / dt

        return np.concatenate(
            (
                linear_velocity_world,
                angular_velocity_world,
            )
        )
