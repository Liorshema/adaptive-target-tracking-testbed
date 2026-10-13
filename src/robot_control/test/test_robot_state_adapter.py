
"""Test estimator-to-controller robot state conversion."""

import numpy as np
import pytest

from robot_control.adapters.robot_state_adapter import RobotStateAdapter
from robot_control.interfaces.control_data import RobotState


@pytest.mark.parametrize("joint_count", [1, 2, 4, 7])
def test_convert_for_different_joint_counts(joint_count):
    """Verify state conversion for configurable manipulator dimensions."""
    adapter = RobotStateAdapter(joint_count=joint_count)

    base_pose = np.array([1.0, -2.0, 0.3])
    base_velocity = np.array([0.5, -0.1])
    joint_positions = np.arange(joint_count, dtype=float) * 0.1
    joint_velocities = np.arange(joint_count, dtype=float) * 0.01

    estimate = np.concatenate(
        (
            base_pose,
            base_velocity,
            joint_positions,
            joint_velocities,
        )
    )

    result = adapter.convert(
        estimated_state=estimate,
        timestamp=12.5,
        frame_id="world",
    )

    assert isinstance(result, RobotState)
    assert adapter.state_dimension == 5 + 2 * joint_count

    np.testing.assert_array_equal(
        result.configuration,
        np.concatenate((base_pose, joint_positions)),
    )
    np.testing.assert_array_equal(
        result.velocity,
        np.concatenate((base_velocity, joint_velocities)),
    )

    assert result.timestamp == 12.5
    assert result.frame_id == "world"


@pytest.mark.parametrize("joint_count", [0, -1, 1.5, True, "4"])
def test_reject_invalid_joint_count(joint_count):
    """Reject invalid adapter configuration."""
    with pytest.raises(ValueError):
        RobotStateAdapter(joint_count=joint_count)


def test_reject_wrong_state_dimension():
    """Reject estimator states with incorrect dimensions."""
    adapter = RobotStateAdapter(joint_count=4)

    with pytest.raises(ValueError, match="shape"):
        adapter.convert(
            estimated_state=np.zeros(12),
            timestamp=1.0,
            frame_id="world",
        )


@pytest.mark.parametrize("invalid_value", [np.nan, np.inf, -np.inf])
def test_reject_nonfinite_state(invalid_value):
    """Reject estimates containing invalid numerical values."""
    adapter = RobotStateAdapter(joint_count=2)
    estimate = np.zeros(adapter.state_dimension)
    estimate[3] = invalid_value

    with pytest.raises(ValueError, match="nonfinite"):
        adapter.convert(
            estimated_state=estimate,
            timestamp=1.0,
            frame_id="world",
        )


def test_reject_invalid_timestamp():
    """Reject nonfinite timestamps."""
    adapter = RobotStateAdapter(joint_count=2)

    with pytest.raises(ValueError, match="timestamp"):
        adapter.convert(
            estimated_state=np.zeros(adapter.state_dimension),
            timestamp=np.nan,
            frame_id="world",
        )


def test_reject_empty_frame():
    """Reject missing coordinate frame identifiers."""
    adapter = RobotStateAdapter(joint_count=2)

    with pytest.raises(ValueError, match="frame_id"):
        adapter.convert(
            estimated_state=np.zeros(adapter.state_dimension),
            timestamp=1.0,
            frame_id="",
        )


def test_conversion_copies_source_state():
    """Ensure the resulting state is independent of the input array."""
    adapter = RobotStateAdapter(joint_count=2)
    estimate = np.arange(adapter.state_dimension, dtype=float)

    result = adapter.convert(
        estimated_state=estimate,
        timestamp=1.0,
        frame_id="world",
    )

    original_configuration = result.configuration.copy()
    original_velocity = result.velocity.copy()

    estimate[:] = -100.0

    np.testing.assert_array_equal(
        result.configuration,
        original_configuration,
    )
    np.testing.assert_array_equal(
        result.velocity,
        original_velocity,
    )
