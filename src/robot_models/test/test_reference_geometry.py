"""Tests for generic tracking-reference geometry."""

import numpy as np

from robot_models.reference.geometry.point_reference import (
    PointReference,
)
from robot_models.reference.geometry.sphere_reference import (
    SphereReference,
)


def test_point_reference() -> None:
    """Verify desired point and residual."""
    geometry = PointReference()

    target = np.array([1.0, 2.0, 3.0])
    offset = np.array([0.5, -0.5, 0.0])

    desired = geometry.desired_point(
        target,
        offset,
    )

    assert np.allclose(
        desired,
        np.array([1.5, 1.5, 3.0]),
    )

    residual = geometry.residual(
        np.array([1.0, 1.0, 3.0]),
        target,
        offset,
    )

    assert np.allclose(
        residual,
        np.array([0.5, 0.5, 0.0]),
    )


def test_sphere_reference_desired_point() -> None:
    """Verify a desired point on the reference sphere."""
    geometry = SphereReference()

    desired = geometry.desired_point(
        target_position_world=np.zeros(3),
        geometry_parameters=np.array([
            2.0,
            0.0,
            0.0,
        ]),
    )

    assert np.allclose(
        desired,
        np.array([2.0, 0.0, 0.0]),
    )


def test_sphere_reference_residual() -> None:
    """Verify constant-distance residual."""
    geometry = SphereReference()

    residual = geometry.residual(
        current_position_world=np.array([
            3.0,
            0.0,
            0.0,
        ]),
        target_position_world=np.zeros(3),
        geometry_parameters=np.array([
            2.0,
            0.0,
            0.0,
        ]),
    )

    assert np.allclose(
        residual,
        np.array([1.0]),
    )
