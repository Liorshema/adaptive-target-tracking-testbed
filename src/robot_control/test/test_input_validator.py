
"""Test freshness and coordinate-frame validation for control inputs."""

import numpy as np
import pytest

from robot_control.interfaces.control_data import (
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.safety.input_validator import (
    InputValidationConfig,
    InputValidator,
)


def make_robot(timestamp=10.0, frame_id="odom"):
    """Create a valid robot state with configurable metadata."""
    return RobotState(
        configuration=np.zeros(7),
        velocity=np.zeros(6),
        timestamp=timestamp,
        frame_id=frame_id,
    )


def make_prediction(
    timestamps=(10.1, 10.2, 10.3),
    frame_id="odom",
):
    """Create a valid prediction with configurable timestamps."""
    return PredictedTargetTrajectory(
        states=np.zeros((len(timestamps), 6)),
        timestamps=np.asarray(timestamps, dtype=float),
        frame_id=frame_id,
    )


def make_validator(
    max_robot_state_age=0.2,
    max_prediction_age=0.5,
    future_tolerance=0.01,
):
    """Create a validator with configurable freshness thresholds."""
    return InputValidator(
        InputValidationConfig(
            max_robot_state_age=max_robot_state_age,
            max_prediction_age=max_prediction_age,
            future_tolerance=future_tolerance,
        )
    )


def test_accept_valid_inputs():
    """Accept fresh inputs sharing the same coordinate frame."""
    validator = make_validator()

    validator.validate(
        robot_state=make_robot(),
        prediction=make_prediction(),
        prediction_reference_timestamp=10.0,
        now=10.05,
    )


def test_reject_stale_robot_state():
    """Reject robot state older than its configured maximum age."""
    with pytest.raises(ValueError, match="Robot state is stale"):
        make_validator().validate(
            robot_state=make_robot(timestamp=9.0),
            prediction=make_prediction(),
            prediction_reference_timestamp=10.0,
            now=10.1,
        )


def test_reject_stale_prediction():
    """Reject prediction based on an outdated target estimate."""
    with pytest.raises(ValueError, match="Target prediction is stale"):
        make_validator().validate(
            robot_state=make_robot(timestamp=10.0),
            prediction=make_prediction(),
            prediction_reference_timestamp=9.0,
            now=10.1,
        )


@pytest.mark.parametrize(
    "robot_timestamp,prediction_timestamp",
    [
        (11.0, 10.0),
        (10.0, 11.0),
    ],
)
def test_reject_future_timestamps(
    robot_timestamp,
    prediction_timestamp,
):
    """Reject robot or prediction timestamps too far in the future."""
    with pytest.raises(ValueError, match="future"):
        make_validator().validate(
            robot_state=make_robot(timestamp=robot_timestamp),
            prediction=make_prediction(),
            prediction_reference_timestamp=prediction_timestamp,
            now=10.0,
        )


def test_reject_frame_mismatch():
    """Reject inputs expressed in different coordinate frames."""
    with pytest.raises(ValueError, match="frames do not match"):
        make_validator().validate(
            robot_state=make_robot(frame_id="odom"),
            prediction=make_prediction(frame_id="map"),
            prediction_reference_timestamp=10.0,
            now=10.05,
        )


def test_reject_expired_prediction_horizon():
    """Reject a trajectory whose final prediction is in the past."""
    with pytest.raises(ValueError, match="horizon has expired"):
        make_validator().validate(
            robot_state=make_robot(timestamp=10.35),
            prediction=make_prediction(),
            prediction_reference_timestamp=10.0,
            now=10.35,
        )


@pytest.mark.parametrize(
    "name,value",
    [
        ("max_robot_state_age", -0.1),
        ("max_prediction_age", np.nan),
        ("future_tolerance", np.inf),
    ],
)
def test_reject_invalid_configuration(name, value):
    """Reject invalid freshness configuration parameters."""
    parameters = {
        "max_robot_state_age": 0.2,
        "max_prediction_age": 0.5,
        "future_tolerance": 0.01,
    }
    parameters[name] = value

    with pytest.raises(ValueError, match=name):
        InputValidationConfig(**parameters)


def test_accept_robot_state_at_age_limit():
    """Accept a robot state exactly at the configured age boundary."""
    make_validator().validate(
        robot_state=make_robot(timestamp=10.0),
        prediction=make_prediction(),
        prediction_reference_timestamp=10.0,
        now=10.2,
    )


def test_accept_timestamp_within_future_tolerance():
    """Accept timestamps within the configured future tolerance."""
    make_validator().validate(
        robot_state=make_robot(timestamp=10.005),
        prediction=make_prediction(),
        prediction_reference_timestamp=10.005,
        now=10.0,
    )


def test_reject_nonfinite_current_time():
    """Reject an invalid evaluation timestamp."""
    with pytest.raises(ValueError, match="timestamps must be finite"):
        make_validator().validate(
            robot_state=make_robot(),
            prediction=make_prediction(),
            prediction_reference_timestamp=10.0,
            now=np.nan,
        )


def test_reject_horizon_shorter_than_requested_lookahead():
    """Input readiness requires coverage of the actual selected target time."""
    validator = InputValidator(InputValidationConfig(
        max_robot_state_age=0.2,
        max_prediction_age=0.5,
        lookahead_time=0.3,
    ))
    with pytest.raises(ValueError, match="horizon"):
        validator.validate(
            robot_state=make_robot(timestamp=10.1),
            prediction=make_prediction(),
            prediction_reference_timestamp=10.0,
            now=10.1,
        )
