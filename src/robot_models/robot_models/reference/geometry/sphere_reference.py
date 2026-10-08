"""Constant-distance spherical tracking-reference geometry."""

import numpy as np

from robot_models.reference.geometry.reference_geometry import (
    ReferenceGeometry,
)


class SphereReference(ReferenceGeometry):
    """Represent a preferred spherical tracking geometry."""

    def desired_point(
        self,
        target_position_world: np.ndarray,
        geometry_parameters: np.ndarray,
    ) -> np.ndarray:
        """
        Return a desired point on the sphere.

        geometry_parameters = [radius, azimuth, elevation].
        """
        target_position_world = np.asarray(
            target_position_world,
            dtype=float,
        )

        parameters = np.asarray(
            geometry_parameters,
            dtype=float,
        )

        if target_position_world.shape != (3,):
            raise ValueError(
                'target_position_world must have shape (3,)'
            )

        if parameters.shape != (3,):
            raise ValueError(
                'geometry_parameters must have shape (3,)'
            )

        radius, azimuth, elevation = parameters

        if radius <= 0.0:
            raise ValueError(
                'sphere radius must be positive'
            )

        direction_world = np.array([
            np.cos(azimuth) * np.cos(elevation),
            np.sin(azimuth) * np.cos(elevation),
            np.sin(elevation),
        ])

        return (
            target_position_world
            + radius * direction_world
        )

    def residual(
        self,
        current_position_world: np.ndarray,
        target_position_world: np.ndarray,
        geometry_parameters: np.ndarray,
    ) -> np.ndarray:
        """Return constant-distance sphere residual."""
        current_position_world = np.asarray(
            current_position_world,
            dtype=float,
        )

        target_position_world = np.asarray(
            target_position_world,
            dtype=float,
        )

        parameters = np.asarray(
            geometry_parameters,
            dtype=float,
        )

        if current_position_world.shape != (3,):
            raise ValueError(
                'current_position_world must have shape (3,)'
            )

        if target_position_world.shape != (3,):
            raise ValueError(
                'target_position_world must have shape (3,)'
            )

        if parameters.shape != (3,):
            raise ValueError(
                'geometry_parameters must have shape (3,)'
            )

        radius = parameters[0]

        if radius <= 0.0:
            raise ValueError(
                'sphere radius must be positive'
            )

        distance = np.linalg.norm(
            current_position_world
            - target_position_world
        )

        return np.array([
            distance - radius
        ])
