
"""Tests for target-state vector conversions."""

import numpy as np

from robot_models.types import TargetState
from robot_simulation.state.target_state import (
    state_to_vector,
    vector_to_state,
)


def test_target_state_round_trip():
    """Vector conversion must preserve the original state."""
    state = TargetState(
        position=np.array([1.0, 2.0, 3.0]),
        velocity=np.array([0.1, 0.2, 0.3]),
    )

    vector = state_to_vector(state)
    restored = vector_to_state(vector)

    np.testing.assert_allclose(
        restored.position,
        state.position,
    )
    np.testing.assert_allclose(
        restored.velocity,
        state.velocity,
    )


def test_target_state_vector_layout():
    """Verify the agreed position-velocity layout."""
    state = TargetState(
        position=np.array([1.0, 2.0, 3.0]),
        velocity=np.array([4.0, 5.0, 6.0]),
    )

    np.testing.assert_array_equal(
        state_to_vector(state),
        np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
    )
