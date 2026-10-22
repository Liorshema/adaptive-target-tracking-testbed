"""Finite rigid transforms and honest candidate tracking-radius geometry."""

from numbers import Real

import numpy as np


def finite_array(value, shape, name):
    """Validate numeric input without silently coercing strings or booleans."""
    raw = np.asarray(value)
    if raw.dtype.kind not in 'iuf' or raw.shape != shape:
        raise ValueError(f'{name} must be a numeric array with shape {shape}')
    result = np.asarray(raw, dtype=float)
    if not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite')
    return result.copy()


def rigid_transform(value):
    """Validate a proper right-handed homogeneous transform."""
    result = finite_array(value, (4, 4), 'transform')
    rotation = result[:3, :3]
    if (not np.allclose(result[3], [0., 0., 0., 1.], atol=1e-8, rtol=0.) or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0.) or
            not np.isclose(np.linalg.det(rotation), 1., atol=1e-6, rtol=0.)):
        raise ValueError('transform must be a proper rigid transform')
    return result


def transform_points(transform, points):
    """Apply T_destination_source to a finite N-by-3 set of source points."""
    transform = rigid_transform(transform)
    raw = np.asarray(points)
    if raw.ndim != 2 or raw.shape[1] != 3:
        raise ValueError('points must have shape (N,3)')
    points = finite_array(raw, raw.shape, 'points')
    return points @ transform[:3, :3].T + transform[:3, 3]


def radius_shell(center, radius, samples=48):
    """Return LINE_LIST endpoint pairs for three great circles on the radius sphere.

    These are candidate camera positions around the target, never an optimized pose.
    """
    center = finite_array(center, (3,), 'center')
    if (isinstance(radius, bool) or not isinstance(radius, Real) or
            not np.isfinite(radius) or radius <= 0):
        raise ValueError('radius must be finite and positive')
    if type(samples) is not int or samples < 4:
        raise ValueError('samples must be an integer >= 4')
    angle = np.arange(samples + 1) * (2 * np.pi / samples)
    result = []
    for first, second in ((0, 1), (0, 2), (1, 2)):
        circle = np.tile(center, (samples + 1, 1))
        circle[:, first] += radius * np.cos(angle)
        circle[:, second] += radius * np.sin(angle)
        result.extend(np.stack([circle[:-1], circle[1:]], axis=1).reshape(-1, 3).tolist())
    return result
