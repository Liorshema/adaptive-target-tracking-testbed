
"""Unit tests for the radial spherical reference policy."""

import numpy as np
import pytest
from robot_models.reference.geometry.sphere_reference import SphereReference

from robot_control.tracking.radial_direction_policy import (
    RadialDirectionPolicy,
)
from robot_control.tracking.reference_selector import ReferenceSelector


@pytest.mark.parametrize(
    "direction",
    [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 2.0, 3.0],
        [-2.0, 1.0, -3.0],
    ],
)
def test_reference_follows_radial_direction(direction):
    """The reference must lie along the target-to-robot ray."""
    radius = 2.5
    target = np.array([2.0, -1.0, 3.0])
    direction = np.asarray(direction, dtype=float)
    current = target + direction

    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=RadialDirectionPolicy(radius=radius),
    )

    reference = selector.select(
        current_position=current,
        target_position=target,
    )

    expected = target + radius * direction / np.linalg.norm(
        direction
    )

    np.testing.assert_allclose(
        reference.position,
        expected,
        atol=1e-12,
    )


def test_reference_distance():
    """The selected reference must have the requested radius."""
    radius = 3.0
    target = np.array([1.0, 2.0, 3.0])
    current = np.array([5.0, -2.0, 6.0])

    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=RadialDirectionPolicy(radius=radius),
    )

    reference = selector.select(current, target)

    np.testing.assert_allclose(
        np.linalg.norm(reference.position - target),
        radius,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "radius",
    [0.0, -1.0, np.nan, np.inf],
)
def test_invalid_radius(radius):
    """Reject nonpositive or nonfinite radii."""
    with pytest.raises(ValueError):
        RadialDirectionPolicy(radius=radius)


def test_coincident_positions():
    """Reject an undefined radial direction."""
    policy = RadialDirectionPolicy(radius=2.0)

    with pytest.raises(ValueError):
        policy.select_parameters(
            current_position=np.ones(3),
            target_position=np.ones(3),
        )


def test_near_coincident_positions():
    """Respect the configured singularity threshold."""
    policy = RadialDirectionPolicy(
        radius=2.0,
        epsilon=1e-6,
    )

    with pytest.raises(ValueError):
        policy.select_parameters(
            current_position=np.array([1e-7, 0.0, 0.0]),
            target_position=np.zeros(3),
        )


def test_invalid_position_dimension():
    """Reject positions that are not three-dimensional."""
    policy = RadialDirectionPolicy(radius=2.0)

    with pytest.raises(ValueError):
        policy.select_parameters(
            current_position=np.zeros(2),
            target_position=np.zeros(3),
        )


def test_nonfinite_position():
    """Reject invalid numerical positions."""
    policy = RadialDirectionPolicy(radius=2.0)

    with pytest.raises(ValueError):
        policy.select_parameters(
            current_position=np.array([np.nan, 0.0, 0.0]),
            target_position=np.zeros(3),
        )


def test_translation_invariance():
    """Translating the world must translate the reference equally."""
    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=RadialDirectionPolicy(radius=2.0),
    )

    current = np.array([4.0, 2.0, 3.0])
    target = np.array([1.0, 0.0, 1.0])
    translation = np.array([10.0, -5.0, 7.0])

    reference_a = selector.select(current, target)
    reference_b = selector.select(
        current + translation,
        target + translation,
    )

    np.testing.assert_allclose(
        reference_b.position,
        reference_a.position + translation,
        atol=1e-12,
    )
