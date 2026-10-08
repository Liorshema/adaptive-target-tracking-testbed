"""Task-space pose error utilities."""

import numpy as np

from robot_models.common.rotations import (
    rotation_log_vector,
)


class PoseError:
    """Compute frame pose error in the world frame."""

    @staticmethod
    def compute(
        transform_world_frame: np.ndarray,
        transform_world_frame_desired: np.ndarray,
    ) -> np.ndarray:
        """Return 6D pose error [position_error, rotation_error]."""
        transform_world_frame = np.asarray(
            transform_world_frame,
            dtype=float,
        )

        transform_world_frame_desired = np.asarray(
            transform_world_frame_desired,
            dtype=float,
        )

        if transform_world_frame.shape != (4, 4):
            raise ValueError(
                'transform_world_frame must have shape (4, 4)'
            )

        if transform_world_frame_desired.shape != (4, 4):
            raise ValueError(
                'transform_world_frame_desired must have shape (4, 4)'
            )

        position_world_frame = (
            transform_world_frame[:3, 3]
        )

        position_world_frame_desired = (
            transform_world_frame_desired[:3, 3]
        )

        position_error_world = (
            position_world_frame_desired
            - position_world_frame
        )

        rotation_world_frame = (
            transform_world_frame[:3, :3]
        )

        rotation_world_frame_desired = (
            transform_world_frame_desired[:3, :3]
        )

        rotation_frame_desired = (
            rotation_world_frame.T
            @ rotation_world_frame_desired
        )

        rotation_error_frame = (
            rotation_log_vector(
                rotation_frame_desired
            )
        )

        rotation_error_world = (
            rotation_world_frame
            @ rotation_error_frame
        )

        return np.concatenate(
            (
                position_error_world,
                rotation_error_world,
            )
        )
