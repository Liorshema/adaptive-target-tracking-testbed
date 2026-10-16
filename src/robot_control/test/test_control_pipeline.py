
"""Test the ROS-independent whole-body control pipeline."""

import numpy as np
import pytest
from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.whole_body.composer import WholeBodyComposer

from robot_control.controllers.jacobian_controller import JacobianController
from robot_control.interfaces.control_data import (
    ControlInput,
    ControlMode,
    ControlOutput,
    RobotState,
    TrackingReference,
)
from robot_control.interfaces.controller import Controller
from robot_control.pipeline.control_pipeline import ControlPipeline
from robot_control.tracking.task_space_error import TaskSpaceError


class TestBase(BaseModel):
    """Base with one forward-translation velocity."""

    @property
    def velocity_dimension(self):
        """Return the base velocity dimension."""
        return 1

    def pose_derivative(self, pose, generalized_velocity):
        """Return zero for this static test model."""
        return np.zeros_like(pose)

    def spatial_jacobian(self, point_base):
        """Return the base translation Jacobian."""
        jacobian = np.zeros((6, 1))
        jacobian[0, 0] = 1.0
        return jacobian


class TestManipulator(ManipulatorModel):
    """Two-DOF Cartesian test manipulator."""

    @property
    def dof(self):
        """Return the joint count."""
        return 2

    def forward_kinematics(self, joint_positions):
        """Compute a simple translation-only pose."""
        transform = np.eye(4)
        transform[:3, 3] = [
            joint_positions[0],
            joint_positions[1],
            0.5,
        ]
        return transform

    def jacobian(self, joint_positions):
        """Return independent x and y joint velocities."""
        jacobian = np.zeros((6, 2))
        jacobian[0, 0] = 1.0
        jacobian[1, 1] = 1.0
        return jacobian


class RecordingController(Controller):
    """Record model inputs and return a configurable command."""

    def __init__(self, command=None):
        """Initialize a reusable test controller."""
        self.command = (
            np.zeros(3) if command is None else np.asarray(command)
        )
        self.received = None

    def compute(self, control_input, model, model_data):
        """Record the delegated computation."""
        self.received = (control_input, model, model_data)
        return ControlOutput(
            command=self.command,
            mode=ControlMode.VELOCITY,
        )


@pytest.fixture
def model():
    """Create a complete test robot model."""
    return WholeBodyComposer(
        base=TestBase(),
        manipulator=TestManipulator(),
    )


@pytest.fixture
def control_input():
    """Create the current robot state and reference."""
    return ControlInput(
        robot_state=RobotState(
            configuration=np.array([0.2, 0.3]),
            velocity=np.array([1.0, 2.0, 3.0]),
            timestamp=10.0,
            frame_id="odom",
        ),
        reference=TrackingReference(
            position=np.array([0.2, 0.3, 0.5]),
        ),
    )


@pytest.fixture
def kinematic_inputs():
    """Provide base pose, joints, and desired task data."""
    desired_pose = np.eye(4)
    desired_pose[:3, 3] = [0.2, 0.3, 0.5]

    return {
        "transform_world_base": np.eye(4),
        "joint_positions": np.array([0.2, 0.3]),
        "desired_end_effector_transform_world": desired_pose,
        "desired_end_effector_twist_world": np.zeros(6),
    }


def test_pipeline_delegates_to_controller(
    model, control_input, kinematic_inputs
):
    """Pass control inputs and robot model to the controller."""
    controller = RecordingController(command=[0.1, 0.2, 0.3])
    pipeline = ControlPipeline(model, controller)

    output = pipeline.compute(control_input, **kinematic_inputs)

    np.testing.assert_allclose(output.command, [0.1, 0.2, 0.3])
    assert output.mode == ControlMode.VELOCITY
    assert controller.received[0] is control_input
    assert controller.received[1] is model


def test_pipeline_builds_correct_model_data(
    model, control_input, kinematic_inputs
):
    """Supply current pose, Jacobian, and twist to the controller."""
    controller = RecordingController()
    pipeline = ControlPipeline(model, controller)

    pipeline.compute(control_input, **kinematic_inputs)

    model_data = controller.received[2]

    np.testing.assert_allclose(
        model_data.end_effector_transform_world[:3, 3],
        [0.2, 0.3, 0.5],
    )

    expected_jacobian = np.zeros((6, 3))
    expected_jacobian[0, 0] = 1.0
    expected_jacobian[0, 1] = 1.0
    expected_jacobian[1, 2] = 1.0

    np.testing.assert_allclose(
        model_data.whole_body_jacobian_world,
        expected_jacobian,
    )

    np.testing.assert_allclose(
        model_data.end_effector_twist_world,
        [3.0, 3.0, 0.0, 0.0, 0.0, 0.0],
    )


def test_controller_can_be_replaced(
    model, control_input, kinematic_inputs
):
    """Swap controller implementations without changing pipeline."""
    first = ControlPipeline(
        model,
        RecordingController(command=[1.0, 0.0, 0.0]),
    )
    second = ControlPipeline(
        model,
        RecordingController(command=[0.0, 1.0, 0.0]),
    )

    first_output = first.compute(control_input, **kinematic_inputs)
    second_output = second.compute(control_input, **kinematic_inputs)

    np.testing.assert_allclose(
        first_output.command,
        [1.0, 0.0, 0.0],
    )
    np.testing.assert_allclose(
        second_output.command,
        [0.0, 1.0, 0.0],
    )


def test_real_jacobian_controller(
    model, control_input, kinematic_inputs
):
    """Compute a known damped least-squares velocity command."""
    task = TaskSpaceError(
        selection_matrix=np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]),
    )

    controller = JacobianController(
        task=task,
        gains=np.eye(1),
        damping=1.0,
    )

    pipeline = ControlPipeline(model, controller)

    desired_twist = np.zeros(6)
    desired_twist[0] = 1.0

    kinematic_inputs["desired_end_effector_twist_world"] = desired_twist

    output = pipeline.compute(control_input, **kinematic_inputs)

    # J_selected = [1, 1, 0].
    # qdot = J.T * (J @ J.T + damping**2 * I)^-1 * [1].
    np.testing.assert_allclose(
        output.command,
        [1.0 / 3.0, 1.0 / 3.0, 0.0],
        atol=1e-10,
    )
    assert output.mode == ControlMode.VELOCITY


def test_reject_wrong_command_dimension(
    model, control_input, kinematic_inputs
):
    """Reject controller outputs inconsistent with the robot."""
    pipeline = ControlPipeline(
        model,
        RecordingController(command=[1.0, 2.0]),
    )

    with pytest.raises(ValueError, match="dimension"):
        pipeline.compute(control_input, **kinematic_inputs)


def test_reject_invalid_control_input(model, kinematic_inputs):
    """Require the shared ControlInput contract."""
    pipeline = ControlPipeline(model, RecordingController())

    with pytest.raises(TypeError, match="ControlInput"):
        pipeline.compute(None, **kinematic_inputs)


def test_reject_invalid_robot_velocity(
    model, control_input, kinematic_inputs
):
    """Reject velocity vectors incompatible with model dimensions."""
    invalid_input = ControlInput(
        robot_state=RobotState(
            configuration=np.zeros(2),
            velocity=np.ones(2),
            timestamp=10.0,
            frame_id="odom",
        ),
        reference=control_input.reference,
    )

    pipeline = ControlPipeline(model, RecordingController())

    with pytest.raises(ValueError, match="generalized_velocity"):
        pipeline.compute(invalid_input, **kinematic_inputs)


def test_reject_invalid_base_transform(
    model, control_input, kinematic_inputs
):
    """Reject a non-rigid world-to-base transformation."""
    transform = np.eye(4)
    transform[0, 0] = 2.0
    kinematic_inputs["transform_world_base"] = transform

    pipeline = ControlPipeline(model, RecordingController())

    with pytest.raises(ValueError, match="orthonormal"):
        pipeline.compute(control_input, **kinematic_inputs)


def test_reject_non_controller(model):
    """Require an implementation of the Controller interface."""
    with pytest.raises(TypeError, match="Controller"):
        ControlPipeline(model, object())
