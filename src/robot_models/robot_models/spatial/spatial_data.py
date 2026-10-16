"""Pose, Twist domain records with validated numerical shapes."""

from dataclasses import dataclass

import numpy as np

from robot_models.spatial.rotations import is_rotation_matrix


@dataclass
class Pose:
    """Rigid-body pose represented by position and rotation."""

    position: np.ndarray
    rotation: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(
            self.position,
            dtype=float,
        )

        self.rotation = np.asarray(
            self.rotation,
            dtype=float,
        )

        if self.position.shape != (3,):
            raise ValueError(
                'position must have shape (3,)'
            )

        if self.rotation.shape != (3, 3):
            raise ValueError(
                'rotation must have shape (3, 3)'
            )

        if not is_rotation_matrix(self.rotation):
            raise ValueError(
                'rotation must be a valid rotation matrix'
            )



@dataclass
class Twist:
    """Spatial velocity represented by linear and angular velocity."""

    linear: np.ndarray
    angular: np.ndarray

    def __post_init__(self) -> None:
        self.linear = np.asarray(
            self.linear,
            dtype=float,
        )

        self.angular = np.asarray(
            self.angular,
            dtype=float,
        )

        if self.linear.shape != (3,):
            raise ValueError(
                'linear must have shape (3,)'
            )

        if self.angular.shape != (3,):
            raise ValueError(
                'angular must have shape (3,)'
            )

    @property
    def vector(self) -> np.ndarray:
        """Return the 6D twist vector [v, omega]."""
        return np.concatenate(
            (
                self.linear,
                self.angular,
            )
        )
