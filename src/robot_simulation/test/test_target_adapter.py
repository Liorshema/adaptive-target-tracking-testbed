
"""Tests for conversion of target states to Gazebo poses."""

import numpy as np
import pytest

from robot_models.types import TargetState
from robot_simulation.gazebo.target_adapter import GazeboTargetAdapter


def make_state():
    """Construct a target state for adapter tests."""
    return TargetState(
        position=np.array([1.0, 2.0, 3.0]),
        velocity=np.array([0.5, 0.0, -0.2]),
    )


def test_adapter_preserves_position():
    """The generated pose should preserve target position."""
    pose = GazeboTargetAdapter().to_pose(make_state())

    np.testing.assert_allclose(
        pose.position,
        [1.0, 2.0, 3.0],
    )


def test_default_orientation():
    """Default orientation should be the identity quaternion."""
    pose = GazeboTargetAdapter().to_pose(make_state())

    np.testing.assert_allclose(
        pose.orientation_xyzw,
        [0.0, 0.0, 0.0, 1.0],
    )


def test_orientation_normalization():
    """The configured orientation should be normalized."""
    adapter = GazeboTargetAdapter(
        orientation_xyzw=np.array([0.0, 0.0, 0.0, 2.0])
    )

    pose = adapter.to_pose(make_state())

    assert np.linalg.norm(pose.orientation_xyzw) == pytest.approx(1.0)


def test_invalid_orientation():
    """Zero-length quaternions must be rejected."""
    with pytest.raises(ValueError):
        GazeboTargetAdapter(
            orientation_xyzw=np.zeros(4)
        )


def test_pose_is_independent_of_input():
    """Output pose must not share mutable position storage."""
    state = make_state()
    pose = GazeboTargetAdapter().to_pose(state)

    pose.position[0] = 99.0

    assert state.position[0] == pytest.approx(1.0)
