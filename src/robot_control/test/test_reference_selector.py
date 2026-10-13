
"""Unit tests for geometry-independent reference selection."""

import numpy as np
import pytest
from robot_models.reference.geometry.point_reference import PointReference
from robot_models.reference.geometry.sphere_reference import SphereReference

from robot_control.tracking.reference_selector import (
    FixedGeometryParameters,
    ReferenceSelector,
)


def test_sphere_reference():
    """Select a desired point on a sphere around the target."""
    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=FixedGeometryParameters(
            np.array([2.0, 0.0, 0.0])
        ),
    )

    reference = selector.select(
        current_position=np.array([0.0, 0.0, 0.0]),
        target_position=np.array([2.0, 3.0, 1.0]),
    )

    np.testing.assert_allclose(
        reference.position,
        [4.0, 3.0, 1.0],
    )


def test_point_reference():
    """Select a target-relative point using another geometry."""
    selector = ReferenceSelector(
        geometry=PointReference(),
        policy=FixedGeometryParameters(
            np.array([1.0, -2.0, 0.5])
        ),
    )

    reference = selector.select(
        current_position=np.zeros(3),
        target_position=np.array([2.0, 3.0, 1.0]),
    )

    np.testing.assert_allclose(
        reference.position,
        [3.0, 1.0, 1.5],
    )


def test_custom_policy():
    """Allow policies to depend on current and target positions."""

    class RelativeDirectionPolicy:
        def select_parameters(
            self,
            current_position,
            target_position,
        ):
            direction = current_position - target_position
            norm = np.linalg.norm(direction)

            if norm == 0.0:
                raise ValueError("Direction is undefined")

            return np.array([
                2.0,
                np.arctan2(direction[1], direction[0]),
                np.arctan2(
                    direction[2],
                    np.linalg.norm(direction[:2]),
                ),
            ])

    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=RelativeDirectionPolicy(),
    )

    reference = selector.select(
        current_position=np.array([3.0, 0.0, 0.0]),
        target_position=np.zeros(3),
    )

    np.testing.assert_allclose(
        reference.position,
        [2.0, 0.0, 0.0],
        atol=1e-12,
    )


def test_fixed_parameters_are_copied():
    """Protect policy parameters from external mutation."""
    original = np.array([2.0, 0.0, 0.0])
    policy = FixedGeometryParameters(original)

    original[0] = 100.0

    parameters = policy.select_parameters(
        np.zeros(3),
        np.zeros(3),
    )

    assert parameters[0] == 2.0


def test_fixed_parameters_reject_nan():
    """Reject invalid geometric parameters."""
    with pytest.raises(ValueError):
        FixedGeometryParameters(
            np.array([1.0, np.nan, 0.0])
        )


def test_position_dimension_mismatch():
    """Reject incompatible current and target dimensions."""
    selector = ReferenceSelector(
        geometry=PointReference(),
        policy=FixedGeometryParameters(np.zeros(3)),
    )

    with pytest.raises(ValueError):
        selector.select(
            current_position=np.zeros(2),
            target_position=np.zeros(3),
        )


def test_invalid_sphere_radius():
    """Preserve validation responsibility in robot_models."""
    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=FixedGeometryParameters(
            np.array([-1.0, 0.0, 0.0])
        ),
    )

    with pytest.raises(ValueError):
        selector.select(
            current_position=np.zeros(3),
            target_position=np.ones(3),
        )


def test_reference_is_independent_of_current_position():
    """Fixed policy should not change with robot position."""
    selector = ReferenceSelector(
        geometry=PointReference(),
        policy=FixedGeometryParameters(
            np.array([0.0, 0.0, 1.0])
        ),
    )

    target = np.array([2.0, 3.0, 4.0])

    reference_a = selector.select(
        current_position=np.zeros(3),
        target_position=target,
    )

    reference_b = selector.select(
        current_position=np.array([10.0, 5.0, -3.0]),
        target_position=target,
    )

    np.testing.assert_array_equal(
        reference_a.position,
        reference_b.position,
    )
