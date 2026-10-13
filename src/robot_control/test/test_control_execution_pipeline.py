
"""Integration tests for robot-state adaptation and safe control."""

import numpy as np
import pytest

from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.velocity_limits import VelocityLimits
from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.reference.geometry.sphere_reference import SphereReference
from robot_models.whole_body.composer import WholeBodyComposer

from robot_control.adapters.planar_kinematics_adapter import (
    PlanarKinematicsAdapter,
)
from robot_control.baseline.jacobian_controller import JacobianController
from robot_control.interfaces.control_data import (
    ControlMode,
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.pipeline.control_execution_pipeline import (
    ControlExecutionPipeline,
)
from robot_control.pipeline.safe_tracking_controller import (
    SafeTrackingController,
    SafeTrackingResult,
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
    """Planar base with forward and angular velocities."""

    @property
    def velocity_dimension(self):
        """Return the base generalized velocity dimension."""
        return 2

    def pose_derivative(self, pose, generalized_velocity):
        """Provide the unused test-model pose derivative."""
        return np.zeros_like(pose)

    def spatial_jacobian(self, point_base):
        """Map forward velocity and yaw rate to spatial twist."""
        jacobian = np.zeros((6, 2))
        jacobian[0, 0] = 1.0
        jacobian[5, 1] = 1.0
        return jacobian


class TestManipulator(ManipulatorModel):
    """Two-joint Cartesian manipulator with known kinematics."""

    @property
    def dof(self):
        """Return manipulator degrees of freedom."""
        return 2

    def forward_kinematics(self, joint_positions):
        """Translate the tool according to the joint configuration."""
        transform = np.eye(4)
        transform[:3, 3] = [
            joint_positions[0],
            joint_positions[1],
            0.5,
        ]
        return transform

    def jacobian(self, joint_positions):
        """Map joint velocities to local x and y translations."""
        jacobian = np.zeros((6, 2))
        jacobian[0, 0] = 1.0
        jacobian[1, 1] = 1.0
        return jacobian


@pytest.fixture
def prediction():
    """Provide a future target position in the odom frame."""
    return PredictedTargetTrajectory(
        states=np.array([
            [3.0, 0.0, 0.5, 0.0, 0.0, 0.0],
            [3.0, 0.0, 0.5, 0.0, 0.0, 0.0],
        ]),
        timestamps=np.array([10.1, 10.2]),
        frame_id="odom",
    )


@pytest.fixture
def robot_state():
    """Provide a planar robot with two manipulator joints."""
    return RobotState(
        configuration=np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
        velocity=np.zeros(4),
        timestamp=10.0,
        frame_id="odom",
    )


def make_pipeline(
    velocity_limit=1.0,
    acceleration_limits=None,
):
    """Build a real tracking and safety chain for integration tests."""
    model = WholeBodyComposer(
        base=TestBase(),
        manipulator=TestManipulator(),
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

    reference_builder = TrackingReferenceBuilder(
        selector=ReferenceSelector(
            geometry=SphereReference(),
            policy=RadialDirectionPolicy(radius=1.0),
        ),
        pose_generator=PoseReferenceGenerator(
            forward_axis_local=np.array([0.0, 0.0, 1.0]),
            up_axis_local=np.array([0.0, -1.0, 0.0]),
            world_up=np.array([0.0, 0.0, 1.0]),
        ),
    )

    tracking = TrackingControlOrchestrator(
        model=model,
        controller=controller,
        reference_builder=reference_builder,
    )

    safety = SafeControlPipeline(
        velocity_limits=VelocityLimits(
            lower=np.full(4, -velocity_limit),
            upper=np.full(4, velocity_limit),
        ),
        acceleration_limits=acceleration_limits,
    )

    return ControlExecutionPipeline(
        kinematics_adapter=PlanarKinematicsAdapter(joint_count=2),
        controller=SafeTrackingController(
            tracking=tracking,
            safety=safety,
        ),
    )


def test_complete_tracking_computation(robot_state, prediction):
    """Compute a known generalized velocity from robot and target."""
    pipeline = make_pipeline()

    result = pipeline.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
    )

    assert isinstance(result, SafeTrackingResult)
    assert result.raw_output.mode == ControlMode.VELOCITY
    assert result.safe_output.mode == ControlMode.VELOCITY

    np.testing.assert_allclose(
        result.raw_output.command,
        [2.0 / 3.0, 0.0, 2.0 / 3.0, 0.0],
        atol=1e-10,
    )

    np.testing.assert_allclose(
        result.safe_output.command,
        result.raw_output.command,
        atol=1e-10,
    )


def test_velocity_safety_after_tracking(robot_state, prediction):
    """Constrain the command without modifying the raw output."""
    pipeline = make_pipeline(velocity_limit=0.3)

    result = pipeline.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
    )

    np.testing.assert_allclose(
        result.raw_output.command,
        [2.0 / 3.0, 0.0, 2.0 / 3.0, 0.0],
        atol=1e-10,
    )

    np.testing.assert_allclose(
        result.safe_output.command,
        [0.3, 0.0, 0.3, 0.0],
        atol=1e-10,
    )


def test_acceleration_context_is_forwarded(robot_state, prediction):
    """Apply acceleration bounds using caller-provided velocity and dt."""
    acceleration = AccelerationLimits(
        lower=np.full(4, -2.0),
        upper=np.full(4, 2.0),
    )

    pipeline = make_pipeline(
        acceleration_limits=acceleration,
    )

    result = pipeline.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
        previous_velocity=np.zeros(4),
        dt=0.1,
    )

    np.testing.assert_allclose(
        result.raw_output.command,
        [2.0 / 3.0, 0.0, 2.0 / 3.0, 0.0],
        atol=1e-10,
    )

    np.testing.assert_allclose(
        result.safe_output.command,
        [0.2, 0.0, 0.2, 0.0],
        atol=1e-10,
    )


def test_missing_acceleration_context_fails(robot_state, prediction):
    """Never bypass configured acceleration limits."""
    acceleration = AccelerationLimits(
        lower=np.full(4, -2.0),
        upper=np.full(4, 2.0),
    )

    pipeline = make_pipeline(
        acceleration_limits=acceleration,
    )

    with pytest.raises(ValueError, match="previous_velocity and dt"):
        pipeline.compute(
            robot_state=robot_state,
            prediction=prediction,
            now=10.0,
        )


def test_invalid_state_dimension_fails(prediction):
    """Reject a state incompatible with the planar adapter."""
    pipeline = make_pipeline()

    invalid_state = RobotState(
        configuration=np.zeros(4),
        velocity=np.zeros(4),
        timestamp=10.0,
        frame_id="odom",
    )

    with pytest.raises(ValueError, match="configuration"):
        pipeline.compute(
            robot_state=invalid_state,
            prediction=prediction,
            now=10.0,
        )


def test_frame_mismatch_fails(robot_state, prediction):
    """Reject predictions expressed in another coordinate frame."""
    pipeline = make_pipeline()

    wrong_frame_prediction = PredictedTargetTrajectory(
        states=prediction.states,
        timestamps=prediction.timestamps,
        frame_id="map",
    )

    with pytest.raises(ValueError, match="frames"):
        pipeline.compute(
            robot_state=robot_state,
            prediction=wrong_frame_prediction,
            now=10.0,
        )


def test_prediction_horizon_fails(robot_state, prediction):
    """Reject computation beyond the prediction horizon."""
    pipeline = make_pipeline()

    with pytest.raises(ValueError, match="horizon"):
        pipeline.compute(
            robot_state=robot_state,
            prediction=prediction,
            now=10.5,
        )


def test_input_arrays_are_not_modified(robot_state, prediction):
    """Keep caller-owned robot and prediction arrays unchanged."""
    pipeline = make_pipeline()

    original_configuration = robot_state.configuration.copy()
    original_velocity = robot_state.velocity.copy()
    original_prediction = prediction.states.copy()

    pipeline.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
    )

    np.testing.assert_array_equal(
        robot_state.configuration,
        original_configuration,
    )

    np.testing.assert_array_equal(
        robot_state.velocity,
        original_velocity,
    )

    np.testing.assert_array_equal(
        prediction.states,
        original_prediction,
    )


def test_reject_invalid_dependencies():
    """Require valid kinematic and controller dependencies."""
    valid_pipeline = make_pipeline()

    with pytest.raises(TypeError, match="PlanarKinematicsAdapter"):
        ControlExecutionPipeline(
            kinematics_adapter=object(),
            controller=valid_pipeline._controller,
        )

    with pytest.raises(TypeError, match="SafeTrackingController"):
        ControlExecutionPipeline(
            kinematics_adapter=PlanarKinematicsAdapter(2),
            controller=object(),
        )
