
"""Integration tests for safe prediction-based tracking control."""

import numpy as np
import pytest
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits
from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.reference.geometry.sphere_reference import SphereReference
from robot_models.whole_body.composer import WholeBodyComposer

from robot_control.controllers.jacobian_controller import JacobianController
from robot_control.interfaces.control_data import (
    ControlMode,
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.pipeline.safe_tracking_controller import (
    SafeTrackingController,
)
from robot_control.pipeline.tracking_control_orchestrator import (
    TrackingControlOrchestrator,
)
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_control.tracking.pose_reference_generator import (
    PoseReferenceGenerator,
)
from robot_control.tracking.radial_direction_policy import (
    RadialDirectionPolicy,
)
from robot_control.tracking.reference_selector import ReferenceSelector
from robot_control.tracking.task_space_error import TaskSpaceError
from robot_control.tracking.tracking_reference_builder import (
    TrackingReferenceBuilder,
)


class TestBase(BaseModel):
    """Simple base with one translational velocity."""

    @property
    def velocity_dimension(self):
        """Return the base velocity dimension."""
        return 1

    def pose_derivative(self, pose, generalized_velocity):
        """Return a zero pose derivative for the test."""
        return np.zeros_like(pose)

    def spatial_jacobian(self, point_base):
        """Map base velocity to translation along x."""
        jacobian = np.zeros((6, 1))
        jacobian[0, 0] = 1.0
        return jacobian


class TestManipulator(ManipulatorModel):
    """Simple two-joint Cartesian manipulator."""

    @property
    def dof(self):
        """Return the manipulator dimension."""
        return 2

    def forward_kinematics(self, joint_positions):
        """Compute the test end-effector pose."""
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


@pytest.fixture
def model():
    """Create an interchangeable whole-body test model."""
    return WholeBodyComposer(
        base=TestBase(),
        manipulator=TestManipulator(),
    )


@pytest.fixture
def tracking(model):
    """Build the complete reference-to-controller computation."""
    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=RadialDirectionPolicy(radius=1.0),
    )

    reference_builder = TrackingReferenceBuilder(
        selector=selector,
        pose_generator=PoseReferenceGenerator(
            forward_axis_local=np.array([0.0, 0.0, 1.0]),
            up_axis_local=np.array([0.0, -1.0, 0.0]),
            world_up=np.array([0.0, 0.0, 1.0]),
        ),
    )

    controller = JacobianController(
        task=TaskSpaceError(
            selection_matrix=np.array([
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]),
        ),
        gains=np.eye(1),
        damping=1.0,
    )

    return TrackingControlOrchestrator(
        model=model,
        controller=controller,
        reference_builder=reference_builder,
    )


@pytest.fixture
def prediction():
    """Provide one future target position."""
    return PredictedTargetTrajectory(
        states=np.array([
            [3.0, 0.0, 0.5, 0.0, 0.0, 0.0],
            [3.0, 0.0, 0.5, 0.0, 0.0, 0.0],
        ]),
        timestamps=np.array([10.1, 10.2]),
        frame_id="odom",
    )


@pytest.fixture
def inputs(prediction):
    """Create consistent robot and prediction inputs."""
    return {
        "robot_state": RobotState(
            configuration=np.zeros(2),
            velocity=np.zeros(3),
            timestamp=10.0,
            frame_id="odom",
        ),
        "prediction": prediction,
        "transform_world_base": np.eye(4),
        "joint_positions": np.zeros(2),
        "now": 10.0,
    }


def make_safety(
    velocity_limit=1.0,
    acceleration_limits=None,
    joint_limits=None,
):
    """Create model-defined safety constraints."""
    return SafeControlPipeline(
        velocity_limits=VelocityLimits(
            lower=np.full(3, -velocity_limit),
            upper=np.full(3, velocity_limit),
        ),
        acceleration_limits=acceleration_limits,
        joint_limits=joint_limits,
        joint_velocity_indices=(
            np.array([1, 2]) if joint_limits is not None else None
        ),
    )


def test_raw_and_safe_outputs_match_without_clipping(tracking, inputs):
    """Preserve a command that satisfies all configured limits."""
    safe_controller = SafeTrackingController(
        tracking=tracking,
        safety=make_safety(),
    )

    result = safe_controller.compute(**inputs)

    expected = [2.0 / 3.0, 2.0 / 3.0, 0.0]

    np.testing.assert_allclose(
        result.raw_output.command,
        expected,
        atol=1e-10,
    )
    np.testing.assert_allclose(
        result.safe_output.command,
        expected,
        atol=1e-10,
    )
    assert result.safe_output.mode == ControlMode.VELOCITY


def test_velocity_limits_preserve_raw_command(tracking, inputs):
    """Limit the safe command without changing the requested command."""
    safe_controller = SafeTrackingController(
        tracking=tracking,
        safety=make_safety(velocity_limit=0.3),
    )

    result = safe_controller.compute(**inputs)

    np.testing.assert_allclose(
        result.raw_output.command,
        [2.0 / 3.0, 2.0 / 3.0, 0.0],
        atol=1e-10,
    )
    np.testing.assert_allclose(
        result.safe_output.command,
        [0.3, 0.3, 0.0],
        atol=1e-10,
    )


def test_acceleration_constraints(tracking, inputs):
    """Enforce acceleration limits using explicit previous velocity."""
    acceleration_limits = AccelerationLimits(
        lower=np.full(3, -2.0),
        upper=np.full(3, 2.0),
    )

    safe_controller = SafeTrackingController(
        tracking=tracking,
        safety=make_safety(
            acceleration_limits=acceleration_limits,
        ),
    )

    result = safe_controller.compute(
        **inputs,
        previous_velocity=np.zeros(3),
        dt=0.1,
    )

    np.testing.assert_allclose(
        result.safe_output.command,
        [0.2, 0.2, 0.0],
        atol=1e-10,
    )


def test_joint_position_constraints(tracking, inputs):
    """Limit a joint approaching its upper position bound."""
    joint_limits = JointLimits(
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
    )

    safe_controller = SafeTrackingController(
        tracking=tracking,
        safety=make_safety(joint_limits=joint_limits),
    )

    inputs["joint_positions"] = np.array([0.99, 0.0])

    result = safe_controller.compute(
        **inputs,
        dt=0.1,
    )

    assert result.raw_output.command[1] > 0.1

    np.testing.assert_allclose(
        result.safe_output.command[1],
        0.1,
        atol=1e-10,
    )

    assert (
        inputs["joint_positions"][0]
        + 0.1 * result.safe_output.command[1]
    ) <= 1.0 + 1e-10


def test_infeasible_constraints_raise(tracking, inputs):
    """Reject conflicting acceleration and joint-position constraints."""
    acceleration_limits = AccelerationLimits(
        lower=np.full(3, -2.0),
        upper=np.full(3, 2.0),
    )
    joint_limits = JointLimits(
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
    )

    safe_controller = SafeTrackingController(
        tracking=tracking,
        safety=make_safety(
            acceleration_limits=acceleration_limits,
            joint_limits=joint_limits,
        ),
    )

    inputs["joint_positions"] = np.array([1.0, 0.0])

    with pytest.raises(ValueError, match="Infeasible"):
        safe_controller.compute(
            **inputs,
            previous_velocity=np.array([0.0, 0.9, 0.0]),
            dt=0.1,
        )


def test_missing_acceleration_context_raises(tracking, inputs):
    """Do not bypass acceleration safety when context is missing."""
    acceleration_limits = AccelerationLimits(
        lower=np.full(3, -2.0),
        upper=np.full(3, 2.0),
    )

    safe_controller = SafeTrackingController(
        tracking=tracking,
        safety=make_safety(
            acceleration_limits=acceleration_limits,
        ),
    )

    with pytest.raises(ValueError, match="previous_velocity and dt"):
        safe_controller.compute(**inputs)


def test_reject_invalid_safety_dependency(tracking):
    """Require the configured safety pipeline interface."""
    with pytest.raises(TypeError, match="SafeControlPipeline"):
        SafeTrackingController(
            tracking=tracking,
            safety=object(),
        )


def test_reject_invalid_tracking_dependency():
    """Require the configured tracking orchestrator interface."""
    with pytest.raises(TypeError, match="TrackingControlOrchestrator"):
        SafeTrackingController(
            tracking=object(),
            safety=make_safety(),
        )
