
"""Unit tests for the generic controller interface."""

import numpy as np
import pytest

from robot_control.interfaces.control_data import (
    ControlInput,
    ControlMode,
    ControlOutput,
    RobotState,
    TrackingReference,
)
from robot_control.interfaces.controller import Controller


class DummyController(Controller):
    """Minimal concrete controller used only for testing."""

    def compute(self, control_input, model, model_data):
        """Return a zero-velocity command."""
        return ControlOutput(
            command=np.zeros(3),
            mode=ControlMode.VELOCITY,
        )


def make_control_input():
    """Create a valid generic control input."""
    return ControlInput(
        robot_state=RobotState(
            configuration=np.zeros(3),
            velocity=np.zeros(3),
            timestamp=1.0,
            frame_id="world",
        ),
        reference=TrackingReference(
            position=np.zeros(3),
        ),
    )


def test_abstract_controller_cannot_be_instantiated():
    """Prevent direct instantiation of an abstract controller."""
    with pytest.raises(TypeError):
        Controller()


def test_concrete_controller_can_be_instantiated():
    """Allow a controller implementing the required interface."""
    controller = DummyController()

    assert isinstance(controller, Controller)


def test_controller_returns_control_output():
    """Verify that a concrete controller can return a command."""
    controller = DummyController()

    output = controller.compute(
        control_input=make_control_input(),
        model=None,
        model_data=None,
    )

    assert isinstance(output, ControlOutput)
    assert output.mode == ControlMode.VELOCITY
    np.testing.assert_array_equal(
        output.command,
        np.zeros(3),
    )


def test_incomplete_controller_cannot_be_instantiated():
    """Reject implementations that do not implement compute."""

    class IncompleteController(Controller):
        pass

    with pytest.raises(TypeError):
        IncompleteController()
