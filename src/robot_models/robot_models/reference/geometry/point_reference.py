"""Point tracking-reference geometry."""

import numpy as np

from robot_models.reference.geometry.reference_geometry import (
    ReferenceGeometry,
)


class PointReference(ReferenceGeometry):
    """Track a point defined relative to the target."""

    def desired_point(
        self,
        target_position_world: np.ndarray,
        geometry_parameters: np.ndarray,
    ) -> np.ndarray:
        """Return target position plus a world-frame offset."""
        target_position_world = np.asarray(
            target_position_world,
            dtype=float,
        )

        offset_world = np.asarray(
            geometry_parameters,
            dtype=float,
        )

        if target_position_world.shape != (3,):
            raise ValueError(
                'target_position_world must have shape (3,)'
            )

        if offset_world.shape != (3,):
            raise ValueError(
                'geometry_parameters must have shape (3,)'
            )

        return target_position_world + offset_world

    def residual(
        self,
        current_position_world: np.ndarray,
        target_position_world: np.ndarray,
        geometry_parameters: np.ndarray,
    ) -> np.ndarray:
        """Return desired-minus-current point residual."""
        current_position_world = np.asarray(
            current_position_world,
            dtype=float,
        )

        if current_position_world.shape != (3,):
            raise ValueError(
                'current_position_world must have shape (3,)'
            )

        desired_position_world = self.desired_point(
            target_position_world,
            geometry_parameters,
        )

        return desired_position_world - current_position_world
