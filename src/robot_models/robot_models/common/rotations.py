"""Rotation-matrix utilities."""

import numpy as np


def skew(vector: np.ndarray) -> np.ndarray:
    """Return the 3x3 skew-symmetric matrix of a 3D vector."""
    vector = np.asarray(
        vector,
        dtype=float,
    )

    if vector.shape != (3,):
        raise ValueError(
            'vector must have shape (3,)'
        )

    x, y, z = vector

    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def axis_angle_rotation(
    axis: np.ndarray,
    angle: float,
) -> np.ndarray:
    """Return rotation matrix from axis-angle representation."""
    axis = np.asarray(
        axis,
        dtype=float,
    )

    if axis.shape != (3,):
        raise ValueError(
            'axis must have shape (3,)'
        )

    axis_norm = np.linalg.norm(axis)

    if axis_norm == 0.0:
        raise ValueError(
            'axis must be non-zero'
        )

    axis = axis / axis_norm

    K = skew(axis)

    return (
        np.eye(3)
        + np.sin(angle) * K
        + (1.0 - np.cos(angle)) * (K @ K)
    )


def rotation_log_vector(
    rotation: np.ndarray,
) -> np.ndarray:
    """
    Return the SO(3) logarithm as a 3D rotation vector.

    The returned vector has direction equal to the rotation axis
    and magnitude equal to the rotation angle.
    """
    rotation = np.asarray(
        rotation,
        dtype=float,
    )

    if rotation.shape != (3, 3):
        raise ValueError(
            'rotation must have shape (3, 3)'
        )

    if not is_rotation_matrix(rotation):
        raise ValueError(
            'rotation must be a valid rotation matrix'
        )

    cos_angle = (
        np.trace(rotation) - 1.0
    ) / 2.0

    cos_angle = np.clip(
        cos_angle,
        -1.0,
        1.0,
    )

    angle = np.arccos(cos_angle)

    if np.isclose(angle, 0.0):
        return np.zeros(3)

    skew_part = (
        rotation - rotation.T
    )

    axis = np.array(
        [
            skew_part[2, 1],
            skew_part[0, 2],
            skew_part[1, 0],
        ],
        dtype=float,
    ) / (
        2.0 * np.sin(angle)
    )

    return axis * angle


def is_rotation_matrix(
    rotation: np.ndarray,
    atol: float = 1e-8,
) -> bool:
    """Return True if the matrix belongs to SO(3)."""
    rotation = np.asarray(
        rotation,
        dtype=float,
    )

    if rotation.shape != (3, 3):
        return False

    identity = np.eye(3)

    orthogonal = np.allclose(
        rotation.T @ rotation,
        identity,
        atol=atol,
    )

    determinant = np.isclose(
        np.linalg.det(rotation),
        1.0,
        atol=atol,
    )

    return bool(
        orthogonal and determinant
    )


def rotation_x(
    angle: float,
) -> np.ndarray:
    """Return rotation matrix about the positive x-axis."""
    return axis_angle_rotation(
        np.array([1.0, 0.0, 0.0]),
        angle,
    )


def rotation_y(
    angle: float,
) -> np.ndarray:
    """Return rotation matrix about the positive y-axis."""
    return axis_angle_rotation(
        np.array([0.0, 1.0, 0.0]),
        angle,
    )


def rotation_z(
    angle: float,
) -> np.ndarray:
    """Return rotation matrix about the positive z-axis."""
    return axis_angle_rotation(
        np.array([0.0, 0.0, 1.0]),
        angle,
    )


def rpy_to_rotation(
    rpy: np.ndarray,
) -> np.ndarray:
    """Return rotation matrix from roll-pitch-yaw angles."""
    rpy = np.asarray(
        rpy,
        dtype=float,
    )

    if rpy.shape != (3,):
        raise ValueError(
            'rpy must have shape (3,)'
        )

    roll, pitch, yaw = rpy

    return (
        rotation_z(yaw)
        @ rotation_y(pitch)
        @ rotation_x(roll)
    )
