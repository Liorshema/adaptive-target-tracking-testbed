
"""Tests for the restoring vector field."""

import numpy as np
import pytest

from robot_simulation.disturbances.vector_field import (
    RestoringVectorField,
    VectorFieldParameters,
)


def make_field():
    return RestoringVectorField(
        VectorFieldParameters(
            center=np.array([0.0, 0.0, 1.0]),
            gain_matrix=np.diag([0.5, 0.5, 1.2]),
        )
    )


def test_zero_acceleration_at_center():
    field = make_field()

    acceleration = field.acceleration(
        np.array([0.0, 0.0, 1.0])
    )

    np.testing.assert_allclose(acceleration, np.zeros(3))


def test_restoring_direction():
    field = make_field()

    acceleration = field.acceleration(
        np.array([2.0, -2.0, 2.0])
    )

    np.testing.assert_allclose(
        acceleration,
        np.array([-1.0, 1.0, -1.2]),
    )


def test_three_dimensional_matrix_gain():
    field = make_field()

    acceleration = field.acceleration(
        np.array([0.0, 0.0, 3.0])
    )

    np.testing.assert_allclose(
        acceleration,
        np.array([0.0, 0.0, -2.4]),
    )


def test_invalid_gain_matrix():
    with pytest.raises(ValueError):
        VectorFieldParameters(
            center=np.zeros(3),
            gain_matrix=np.diag([1.0, -1.0, 1.0]),
        )


def test_dimension_mismatch():
    field = make_field()

    with pytest.raises(ValueError):
        field.acceleration(np.array([1.0, 2.0]))
