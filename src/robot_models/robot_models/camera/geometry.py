"""Camera-frame geometry utilities."""

import numpy as np

from robot_models.spatial.transforms import (
    invert_transform,
)


class CameraGeometry:
    """Express target geometry in the camera optical frame."""

    @staticmethod
    def target_transform_optical(
        transform_world_optical: np.ndarray,
        transform_world_target: np.ndarray,
    ) -> np.ndarray:
        """
        Compute target pose relative to the camera optical frame.

        T_O_T = inv(T_W_O) @ T_W_T
        """
        transform_world_optical = np.asarray(
            transform_world_optical,
            dtype=float,
        )

        transform_world_target = np.asarray(
            transform_world_target,
            dtype=float,
        )

        if transform_world_optical.shape != (4, 4):
            raise ValueError(
                'transform_world_optical must have shape (4, 4)'
            )

        if transform_world_target.shape != (4, 4):
            raise ValueError(
                'transform_world_target must have shape (4, 4)'
            )

        transform_optical_world = invert_transform(
            transform_world_optical
        )

        return (
            transform_optical_world
            @ transform_world_target
        )

    @staticmethod
    def target_position_optical(
        transform_world_optical: np.ndarray,
        target_position_world: np.ndarray,
    ) -> np.ndarray:
        """Express a target point in the camera optical frame."""
        transform_world_optical = np.asarray(
            transform_world_optical,
            dtype=float,
        )

        target_position_world = np.asarray(
            target_position_world,
            dtype=float,
        )

        if transform_world_optical.shape != (4, 4):
            raise ValueError(
                'transform_world_optical must have shape (4, 4)'
            )

        if target_position_world.shape != (3,):
            raise ValueError(
                'target_position_world must have shape (3,)'
            )

        rotation_world_optical = (
            transform_world_optical[:3, :3]
        )

        optical_position_world = (
            transform_world_optical[:3, 3]
        )

        return (
            rotation_world_optical.T
            @ (
                target_position_world
                - optical_position_world
            )
        )
