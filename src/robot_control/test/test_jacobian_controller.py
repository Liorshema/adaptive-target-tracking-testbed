
"""Unit tests for the generic Jacobian DLS controller."""

from types import SimpleNamespace

import numpy as np
import pytest
from robot_models.interfaces.model_data import ModelData

from robot_control.baseline.jacobian_controller import JacobianController
from robot_control.interfaces.control_data import (
    ControlInput,
    ControlMode,
    RobotState,
    TrackingReference,
)
from robot_control.tracking.task_space_error import TaskSpaceError


def make_case(
    jacobian,
    position_error=None,
    desired_twist=None,
):
    """Build a synthetic, dimension-consistent control case."""
    jacobian = np.asarray(jacobian, dtype=float)
    dof = jacobian.shape[1]

    if position_error is None:
        position_error = np.zeros(3)

    if desired_twist is None:
        desired_twist = np.zeros(6)

    current = np.eye(4)
    desired = np.eye(4)
    desired[:3, 3] = position_error

    model = SimpleNamespace(
        generalized_velocity_dimension=dof
    )

    model_data = ModelData(
        end_effector_transform_world=current,
        end_effector_twist_world=np.zeros(6),
        desired_end_effector_transform_world=desired,
        desired_end_effector_twist_world=desired_twist,
        whole_body_jacobian_world=jacobian,
    )

    control_input = ControlInput(
        robot_state=RobotState(
            configuration=np.zeros(dof),
            velocity=np.zeros(dof),
            timestamp=1.0,
            frame_id="world",
        ),
        reference=TrackingReference(
            position=desired[:3, 3],
        ),
    )

    return control_input, model, model_data


def make_controller(damping=0.1, gains=None):
    """Create a position-only DLS controller."""
    if gains is None:
        gains = np.eye(3)

    return JacobianController(
        task=TaskSpaceError.position_only(),
        gains=gains,
        damping=damping,
    )


def test_feedback_command():
    """Verify the analytical DLS solution."""
    J = np.zeros((6, 2))
    J[0, 0] = 1.0
    J[1, 1] = 1.0

    inputs = make_case(
        J,
        position_error=[1.0, 2.0, 0.0],
    )

    output = make_controller().compute(*inputs)

    expected = np.array([1.0, 2.0]) / 1.01

    np.testing.assert_allclose(
        output.command,
        expected,
        atol=1e-12,
    )

    assert output.mode == ControlMode.VELOCITY


def test_zero_error_zero_command():
    """Zero error and zero feedforward produce zero command."""
    J = np.zeros((6, 3))
    J[:3, :] = np.eye(3)

    output = make_controller().compute(
        *make_case(J)
    )

    np.testing.assert_allclose(
        output.command,
        np.zeros(3),
        atol=1e-12,
    )


def test_feedforward():
    """Apply the desired task-space velocity."""
    J = np.zeros((6, 3))
    J[:3, :] = np.eye(3)

    twist = np.array([1.0, -2.0, 3.0, 0.0, 0.0, 0.0])

    output = make_controller().compute(
        *make_case(J, desired_twist=twist)
    )

    np.testing.assert_allclose(
        output.command,
        twist[:3] / 1.01,
        atol=1e-12,
    )


def test_feedback_and_feedforward():
    """Combine reference velocity and proportional feedback."""
    J = np.zeros((6, 3))
    J[:3, :] = np.eye(3)

    gains = np.diag([2.0, 3.0, 4.0])
    error = np.array([1.0, -1.0, 2.0])
    twist = np.array([0.5, 0.5, 0.5, 0.0, 0.0, 0.0])

    output = make_controller(gains=gains).compute(
        *make_case(
            J,
            position_error=error,
            desired_twist=twist,
        )
    )

    expected = (twist[:3] + gains @ error) / 1.01

    np.testing.assert_allclose(
        output.command,
        expected,
        atol=1e-12,
    )


@pytest.mark.parametrize("dof", [2, 3, 4, 6, 8])
def test_variable_robot_dimensions(dof):
    """Support arbitrary generalized velocity dimensions."""
    J = np.zeros((6, dof))

    for i in range(min(3, dof)):
        J[i, i] = 1.0

    output = make_controller().compute(
        *make_case(
            J,
            position_error=[1.0, 2.0, 3.0],
        )
    )

    assert output.command.shape == (dof,)
    assert np.all(np.isfinite(output.command))


def test_singular_jacobian():
    """Damping must handle a rank-deficient Jacobian."""
    J = np.zeros((6, 3))
    J[0, :] = [1.0, 1.0, 1.0]

    output = make_controller().compute(
        *make_case(
            J,
            position_error=[1.0, 2.0, 3.0],
        )
    )

    assert np.all(np.isfinite(output.command))

    np.testing.assert_allclose(
        output.command,
        np.ones(3) / 3.01,
        atol=1e-12,
    )


def test_zero_jacobian():
    """An immobile task must produce a finite zero command."""
    J = np.zeros((6, 4))

    output = make_controller().compute(
        *make_case(
            J,
            position_error=[1.0, 2.0, 3.0],
        )
    )

    np.testing.assert_array_equal(
        output.command,
        np.zeros(4),
    )


def test_full_pose_task():
    """Support six-dimensional pose tracking."""
    J = np.eye(6)

    controller = JacobianController(
        task=TaskSpaceError.full_pose(),
        gains=np.eye(6),
        damping=0.1,
    )

    output = controller.compute(
        *make_case(
            J,
            position_error=[1.0, 2.0, 3.0],
        )
    )

    np.testing.assert_allclose(
        output.command,
        [1 / 1.01, 2 / 1.01, 3 / 1.01, 0, 0, 0],
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "damping",
    [0.0, -0.1, np.nan, np.inf],
)
def test_invalid_damping(damping):
    """Reject invalid DLS regularization."""
    with pytest.raises(ValueError):
        make_controller(damping=damping)


def test_invalid_gain_dimension():
    """Reject gains incompatible with the selected task."""
    with pytest.raises(ValueError):
        make_controller(gains=np.eye(2))


def test_non_positive_definite_gains():
    """Reject gain matrices that are not positive definite."""
    with pytest.raises(ValueError):
        make_controller(
            gains=np.diag([1.0, 0.0, 1.0])
        )


def test_velocity_dimension_mismatch():
    """Reject Jacobians incompatible with the robot model."""
    J = np.zeros((6, 3))
    control_input, model, model_data = make_case(J)

    model.generalized_velocity_dimension = 4

    with pytest.raises(ValueError):
        make_controller().compute(
            control_input,
            model,
            model_data,
        )


def test_nonfinite_jacobian():
    """Reject invalid Jacobian data before solving."""
    J = np.zeros((6, 3))
    control_input, model, model_data = make_case(J)

    model_data.whole_body_jacobian_world[0, 0] = np.nan

    with pytest.raises(ValueError):
        make_controller().compute(
            control_input,
            model,
            model_data,
        )
