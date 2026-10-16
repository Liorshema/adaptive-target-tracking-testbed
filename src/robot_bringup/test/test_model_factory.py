"""Integration tests for robot-model composition."""

import numpy as np

from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import (
    load_robot_model_parameters,
)


def test_robot_model_factory() -> None:
    """Verify robot model construction from robot description."""
    parameters = load_robot_model_parameters()
    robot = create_robot_model(parameters)

    assert robot.base.velocity_dimension == 2
    assert robot.manipulator.dof == 4
    assert robot.generalized_velocity_dimension == 6

    assert np.isclose(
        robot.base.wheel_radius,
        parameters.base.wheel_radius,
    )

    assert np.isclose(
        robot.base.track_width,
        parameters.base.track_width,
    )

    assert np.allclose(
        parameters.manipulator.joint_axes,
        np.array([
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
        ]),
    )
