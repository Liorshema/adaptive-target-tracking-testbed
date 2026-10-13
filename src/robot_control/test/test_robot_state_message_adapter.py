
"""Test conversion of ROS state-estimate messages to control states."""

import numpy as np
import pytest

from robot_interfaces.msg import StateEstimate

from robot_control.adapters.robot_state_message_adapter import (
    RobotStateMessageAdapter,
)
from robot_control.interfaces.control_data import RobotState


def make_message(joint_count=4):
    """Create a valid state estimate for an arbitrary joint count."""
    dimension = 5 + 2 * joint_count

    message = StateEstimate()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 250_000_000
    message.header.frame_id = "odom"
    message.mean = np.arange(dimension, dtype=float).tolist()
    message.covariance = np.eye(dimension).ravel().tolist()

    return message


@pytest.mark.parametrize("joint_count", [1, 2, 4, 7])
def test_convert_for_different_joint_counts(joint_count):
    """Verify conversion across configurable manipulator dimensions."""
    message = make_message(joint_count)
    adapter = RobotStateMessageAdapter(joint_count)

    result = adapter.convert(message)

    assert isinstance(result, RobotState)
    assert result.configuration.shape == (3 + joint_count,)
    assert result.velocity.shape == (2 + joint_count,)
    assert result.timestamp == 10.25
    assert result.frame_id == "odom"

    mean = np.asarray(message.mean)

    np.testing.assert_array_equal(
        result.configuration,
        np.concatenate((mean[:3], mean[5:5 + joint_count])),
    )
    np.testing.assert_array_equal(
        result.velocity,
        np.concatenate((mean[3:5], mean[5 + joint_count:])),
    )


def test_reject_wrong_message_type():
    """Reject objects that are not StateEstimate messages."""
    adapter = RobotStateMessageAdapter(joint_count=4)

    with pytest.raises(TypeError, match="StateEstimate"):
        adapter.convert(object())


def test_reject_wrong_mean_dimension():
    """Reject a mean vector inconsistent with the configured robot."""
    message = make_message()
    message.mean = message.mean[:-1]

    with pytest.raises(ValueError, match="mean"):
        RobotStateMessageAdapter(4).convert(message)


def test_reject_wrong_covariance_dimension():
    """Reject incorrectly flattened covariance matrices."""
    message = make_message()
    message.covariance = message.covariance[:-1]

    with pytest.raises(ValueError, match="covariance"):
        RobotStateMessageAdapter(4).convert(message)


@pytest.mark.parametrize("invalid_value", [np.nan, np.inf, -np.inf])
def test_reject_nonfinite_mean(invalid_value):
    """Reject nonfinite entries in the estimated state."""
    message = make_message()
    mean = list(message.mean)
    mean[0] = invalid_value
    message.mean = mean

    with pytest.raises(ValueError, match="nonfinite"):
        RobotStateMessageAdapter(4).convert(message)


@pytest.mark.parametrize("invalid_value", [np.nan, np.inf, -np.inf])
def test_reject_nonfinite_covariance(invalid_value):
    """Reject nonfinite entries in the covariance matrix."""
    message = make_message()
    covariance = list(message.covariance)
    covariance[0] = invalid_value
    message.covariance = covariance

    with pytest.raises(ValueError, match="nonfinite"):
        RobotStateMessageAdapter(4).convert(message)


def test_reject_asymmetric_covariance():
    """Reject covariance matrices that violate symmetry."""
    message = make_message()

    covariance = np.eye(13)
    covariance[0, 1] = 0.5
    message.covariance = covariance.ravel().tolist()

    with pytest.raises(ValueError, match="symmetric"):
        RobotStateMessageAdapter(4).convert(message)


def test_reject_negative_covariance_diagonal():
    """Reject negative state variances."""
    message = make_message()

    covariance = np.eye(13)
    covariance[2, 2] = -1.0
    message.covariance = covariance.ravel().tolist()

    with pytest.raises(ValueError, match="negative diagonal"):
        RobotStateMessageAdapter(4).convert(message)


def test_reject_empty_frame():
    """Reject an estimate without a coordinate frame."""
    message = make_message()
    message.header.frame_id = ""

    with pytest.raises(ValueError, match="frame_id"):
        RobotStateMessageAdapter(4).convert(message)


def test_timestamp_conversion():
    """Verify nanosecond precision in ROS timestamp conversion."""
    message = make_message()
    message.header.stamp.sec = 42
    message.header.stamp.nanosec = 125_000_000

    result = RobotStateMessageAdapter(4).convert(message)

    assert result.timestamp == pytest.approx(42.125)


def test_conversion_copies_source_data():
    """Ensure the control state is independent of message arrays."""
    message = make_message()
    result = RobotStateMessageAdapter(4).convert(message)

    expected_configuration = result.configuration.copy()
    expected_velocity = result.velocity.copy()

    message.mean = [0.0] * len(message.mean)

    np.testing.assert_array_equal(
        result.configuration,
        expected_configuration,
    )
    np.testing.assert_array_equal(
        result.velocity,
        expected_velocity,
    )
