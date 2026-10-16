"""Homogeneous transformation utilities for SE(3)."""

import numpy as np

from robot_models.spatial.rotations import is_rotation_matrix
from robot_models.spatial.spatial_data import Pose


def make_transform(
    rotation: np.ndarray,
    position: np.ndarray,
) -> np.ndarray:
    """Build a 4x4 homogeneous transformation matrix."""
    rotation = np.asarray(rotation, dtype=float)
    position = np.asarray(position, dtype=float)

    if rotation.shape != (3, 3):
        raise ValueError('rotation must have shape (3, 3)')

    if position.shape != (3,):
        raise ValueError('position must have shape (3,)')

    if not is_rotation_matrix(rotation):
        raise ValueError('rotation must be a valid rotation matrix')

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = position

    return transform


def pose_to_transform(pose: Pose) -> np.ndarray:
    """Convert a Pose object to a homogeneous transform."""
    return make_transform(
        rotation=pose.rotation,
        position=pose.position,
    )


def transform_to_pose(transform: np.ndarray) -> Pose:
    """Convert a homogeneous transform to a Pose object."""
    transform = np.asarray(transform, dtype=float)

    if transform.shape != (4, 4):
        raise ValueError('transform must have shape (4, 4)')

    if not np.allclose(
        transform[3, :],
        np.array([0.0, 0.0, 0.0, 1.0]),
    ):
        raise ValueError('invalid homogeneous transformation matrix')

    rotation = transform[:3, :3]
    position = transform[:3, 3]

    if not is_rotation_matrix(rotation):
        raise ValueError('transform contains an invalid rotation matrix')

    return Pose(
        position=position,
        rotation=rotation,
    )


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Return the inverse of a homogeneous transform."""
    transform = np.asarray(transform, dtype=float)

    if transform.shape != (4, 4):
        raise ValueError('transform must have shape (4, 4)')

    rotation = transform[:3, :3]
    position = transform[:3, 3]

    if not is_rotation_matrix(rotation):
        raise ValueError('transform contains an invalid rotation matrix')

    inverse = np.eye(4, dtype=float)

    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ position

    return inverse


def compose_transforms(
    transform_a_b: np.ndarray,
    transform_b_c: np.ndarray,
) -> np.ndarray:
    """Compose transforms T_A_B and T_B_C into T_A_C."""
    transform_a_b = np.asarray(transform_a_b, dtype=float)
    transform_b_c = np.asarray(transform_b_c, dtype=float)

    if transform_a_b.shape != (4, 4):
        raise ValueError(
            'transform_a_b must have shape (4, 4)'
        )

    if transform_b_c.shape != (4, 4):
        raise ValueError(
            'transform_b_c must have shape (4, 4)'
        )

    return transform_a_b @ transform_b_c


def transform_point(
    transform: np.ndarray,
    point: np.ndarray,
) -> np.ndarray:
    """Transform a 3D point using a homogeneous transform."""
    transform = np.asarray(transform, dtype=float)
    point = np.asarray(point, dtype=float)

    if transform.shape != (4, 4):
        raise ValueError('transform must have shape (4, 4)')

    if point.shape != (3,):
        raise ValueError('point must have shape (3,)')

    point_h = np.concatenate(
        (
            point,
            np.array([1.0]),
        )
    )

    transformed_h = transform @ point_h

    return transformed_h[:3]
