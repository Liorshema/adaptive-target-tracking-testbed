
"""Unit tests for generic robot control data contracts."""

import numpy as np
import pytest

from robot_control.interfaces.control_data import (
    ControlInput,
    ControlMode,
    ControlOutput,
    PredictedTargetTrajectory,
    ReferenceTrajectory,
    RobotState,
    TrackingReference,
)


def make_robot_state():
    """Create a generic test robot state."""
    return RobotState(
        configuration=np.zeros(5),
        velocity=np.zeros(4),
        timestamp=1.0,
        frame_id="world",
    )


def test_robot_state():
    """Accept generic robot state dimensions."""
    state = make_robot_state()

    assert state.configuration.shape == (5,)
    assert state.velocity.shape == (4,)


def test_robot_state_rejects_nan():
    """Reject nonfinite state values."""
    with pytest.raises(ValueError):
        RobotState(
            configuration=np.array([0.0, np.nan]),
            velocity=np.zeros(2),
            timestamp=1.0,
            frame_id="world",
        )


def test_reference_dimensions():
    """Require matching reference position and velocity."""
    with pytest.raises(ValueError):
        TrackingReference(
            position=np.zeros(3),
            velocity=np.zeros(2),
        )


def test_reference_trajectory():
    """Accept a time-indexed reference trajectory."""
    reference = ReferenceTrajectory(
        positions=np.zeros((10, 3)),
        timestamps=np.arange(10, dtype=float),
        frame_id="world",
    )

    assert reference.positions.shape == (10, 3)


def test_reference_trajectory_rejects_time_order():
    """Require strictly increasing timestamps."""
    with pytest.raises(ValueError):
        ReferenceTrajectory(
            positions=np.zeros((3, 2)),
            timestamps=np.array([0.0, 2.0, 1.0]),
            frame_id="world",
        )


def test_target_prediction():
    """Accept an arbitrary prediction horizon and state size."""
    prediction = PredictedTargetTrajectory(
        states=np.zeros((20, 6)),
        timestamps=np.arange(1, 21, dtype=float),
        frame_id="world",
    )

    assert prediction.states.shape == (20, 6)


def test_generic_control_input():
    """Support both instantaneous and predictive control."""
    state = make_robot_state()
    reference = TrackingReference(position=np.zeros(3))

    feedback_input = ControlInput(
        robot_state=state,
        reference=reference,
    )

    assert feedback_input.target_prediction is None

    prediction = PredictedTargetTrajectory(
        states=np.zeros((5, 6)),
        timestamps=np.arange(2, 7, dtype=float),
        frame_id="world",
    )

    predictive_input = ControlInput(
        robot_state=state,
        reference=reference,
        target_prediction=prediction,
    )

    assert predictive_input.target_prediction is not None


@pytest.mark.parametrize(
    "mode",
    [
        ControlMode.VELOCITY,
        ControlMode.ACCELERATION,
        ControlMode.EFFORT,
    ],
)
def test_control_modes(mode):
    """Support multiple physical actuation modes."""
    output = ControlOutput(
        command=np.zeros(4),
        mode=mode,
    )

    assert output.mode == mode


def test_predictive_control_output():
    """Allow an optional future command sequence."""
    output = ControlOutput(
        command=np.zeros(4),
        mode=ControlMode.VELOCITY,
        predicted_commands=np.zeros((10, 4)),
    )

    assert output.predicted_commands.shape == (10, 4)


def test_output_rejects_dimension_mismatch():
    """Reject incompatible predicted command dimensions."""
    with pytest.raises(ValueError):
        ControlOutput(
            command=np.zeros(4),
            mode=ControlMode.VELOCITY,
            predicted_commands=np.zeros((10, 3)),
        )


def test_output_copies_input_array():
    """Avoid accidental mutation through the original array."""
    original = np.array([1.0, 2.0])

    output = ControlOutput(
        command=original,
        mode=ControlMode.VELOCITY,
    )

    original[0] = 100.0

    assert output.command[0] == 1.0
