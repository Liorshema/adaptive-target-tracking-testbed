
"""Test ROS target-prediction message conversion."""

import numpy as np
import pytest
from robot_interfaces.msg import PredictedTrajectory

from robot_control.adapters.prediction_message_adapter import (
    PredictionMessageAdapter,
)
from robot_control.interfaces.control_data import PredictedTargetTrajectory


def make_message(horizon=3, state_dimension=6):
    """Create a valid predicted-trajectory message."""
    message = PredictedTrajectory()
    message.header.stamp.sec = 10
    message.header.frame_id = "odom"
    message.horizon = horizon
    message.state_dimension = state_dimension
    message.timestamps = [
        10.0 + 0.1 * (index + 1)
        for index in range(horizon)
    ]
    message.states = np.arange(
        horizon * state_dimension, dtype=float
    ).tolist()
    return message


@pytest.mark.parametrize(
    "horizon,state_dimension",
    [(1, 2), (3, 6), (5, 4), (10, 8)],
)
def test_convert_different_dimensions(horizon, state_dimension):
    """Convert predictions of configurable shapes."""
    message = make_message(horizon, state_dimension)

    result = PredictionMessageAdapter().convert(message)

    assert isinstance(result, PredictedTargetTrajectory)
    assert result.states.shape == (horizon, state_dimension)
    assert result.timestamps.shape == (horizon,)
    assert result.frame_id == "odom"


def test_reject_wrong_message_type():
    """Reject objects that are not prediction ROS messages."""
    with pytest.raises(TypeError, match="PredictedTrajectory"):
        PredictionMessageAdapter().convert(object())


@pytest.mark.parametrize(
    "horizon,state_dimension",
    [(0, 6), (3, 0)],
)
def test_reject_empty_dimensions(horizon, state_dimension):
    """Reject empty trajectory dimensions."""
    message = make_message(3, 6)
    message.horizon = horizon
    message.state_dimension = state_dimension

    with pytest.raises(ValueError, match="positive"):
        PredictionMessageAdapter().convert(message)


def test_reject_wrong_timestamp_count():
    """Reject a timestamp array inconsistent with the horizon."""
    message = make_message()
    message.timestamps = message.timestamps[:-1]

    with pytest.raises(ValueError, match="timestamps"):
        PredictionMessageAdapter().convert(message)


def test_reject_wrong_state_count():
    """Reject an incorrectly flattened state matrix."""
    message = make_message()
    message.states = message.states[:-1]

    with pytest.raises(ValueError, match="states"):
        PredictionMessageAdapter().convert(message)


@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_reject_nonfinite_timestamps(invalid):
    """Reject nonfinite trajectory timestamps."""
    message = make_message()
    values = list(message.timestamps)
    values[0] = invalid
    message.timestamps = values

    with pytest.raises(ValueError, match="nonfinite"):
        PredictionMessageAdapter().convert(message)


@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_reject_nonfinite_states(invalid):
    """Reject nonfinite predicted states."""
    message = make_message()
    values = list(message.states)
    values[0] = invalid
    message.states = values

    with pytest.raises(ValueError, match="nonfinite"):
        PredictionMessageAdapter().convert(message)


def test_reject_nonincreasing_timestamps():
    """Reject timestamps that are not strictly increasing."""
    message = make_message()
    message.timestamps = [10.1, 10.3, 10.2]

    with pytest.raises(ValueError, match="increasing"):
        PredictionMessageAdapter().convert(message)


def test_reject_prediction_before_reference_time():
    """Reject prediction samples that precede their reference time."""
    message = make_message()
    message.timestamps = [9.9, 10.1, 10.2]

    with pytest.raises(ValueError, match="reference timestamp"):
        PredictionMessageAdapter().convert(message)


def test_reject_empty_frame():
    """Reject predictions without a coordinate frame."""
    message = make_message()
    message.header.frame_id = ""

    with pytest.raises(ValueError, match="frame_id"):
        PredictionMessageAdapter().convert(message)


def test_preserve_sample_values():
    """Preserve state ordering when reshaping row-major data."""
    message = make_message(horizon=3, state_dimension=6)

    result = PredictionMessageAdapter().convert(message)

    expected = np.arange(18, dtype=float).reshape(3, 6)
    np.testing.assert_array_equal(result.states, expected)


def test_conversion_copies_source_data():
    """Return arrays independent of the incoming ROS message."""
    message = make_message()
    result = PredictionMessageAdapter().convert(message)

    states_before = result.states.copy()
    timestamps_before = result.timestamps.copy()

    message.states = [0.0] * len(message.states)
    message.timestamps = [11.0, 12.0, 13.0]

    np.testing.assert_array_equal(result.states, states_before)
    np.testing.assert_array_equal(result.timestamps, timestamps_before)
