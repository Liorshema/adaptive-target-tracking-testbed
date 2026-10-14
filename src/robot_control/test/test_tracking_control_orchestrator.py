
"""Integration tests for prediction-based tracking control."""

import numpy as np
import pytest
from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.reference.geometry.sphere_reference import SphereReference
from robot_models.whole_body.composer import WholeBodyComposer

from robot_control.baseline.jacobian_controller import JacobianController
from robot_control.interfaces.control_data import (
    ControlMode,
    ControlOutput,
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.interfaces.controller import Controller
from robot_control.pipeline.tracking_control_orchestrator import (
    TrackingControlOrchestrator,
)
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
        """Return a zero pose derivative for this test model."""
        return np.zeros_like(pose)

    def spatial_jacobian(self, point_base):
        """Map base velocity to end-effector x translation."""
        jacobian = np.zeros((6, 1))
        jacobian[0, 0] = 1.0
        return jacobian


class TestManipulator(ManipulatorModel):
    """Two-joint Cartesian manipulator."""

    @property
    def dof(self):
        """Return the number of manipulator joints."""
        return 2

    def forward_kinematics(self, joint_positions):
        """Compute the end-effector transform."""
        transform = np.eye(4)
        transform[:3, 3] = [
            joint_positions[0],
            joint_positions[1],
            0.5,
        ]
        return transform

    def jacobian(self, joint_positions):
        """Map joint velocities to x and y translation."""
        jacobian = np.zeros((6, 2))
        jacobian[0, 0] = 1.0
        jacobian[1, 1] = 1.0
        return jacobian


class RecordingController(Controller):
    """Record the data supplied by the tracking orchestrator."""

    def __init__(self):
        """Initialize the recording controller."""
        self.received = None

    def compute(self, control_input, model, model_data):
        """Record inputs and return a valid zero command."""
        self.received = (control_input, model, model_data)

        return ControlOutput(
            command=np.zeros(model.generalized_velocity_dimension),
            mode=ControlMode.VELOCITY,
        )


@pytest.fixture
def model():
    """Create a three-velocity whole-body model."""
    return WholeBodyComposer(
        base=TestBase(),
        manipulator=TestManipulator(),
    )


@pytest.fixture
def reference_builder():
    """Create a spherical reference with camera look-at orientation."""
    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=RadialDirectionPolicy(radius=1.0),
    )

    pose_generator = PoseReferenceGenerator(
        forward_axis_local=np.array([0.0, 0.0, 1.0]),
        up_axis_local=np.array([0.0, -1.0, 0.0]),
        world_up=np.array([0.0, 0.0, 1.0]),
    )

    return TrackingReferenceBuilder(
        selector=selector,
        pose_generator=pose_generator,
    )


@pytest.fixture
def robot_state():
    """Create a stationary robot in the odom frame."""
    return RobotState(
        configuration=np.zeros(2),
        velocity=np.zeros(3),
        timestamp=10.0,
        frame_id="odom",
    )


@pytest.fixture
def prediction():
    """Create a moving target prediction with three future samples."""
    return PredictedTargetTrajectory(
        states=np.array([
            [3.0, 0.0, 0.5, 0.0, 0.0, 0.0],
            [4.0, 0.0, 0.5, 0.0, 0.0, 0.0],
            [5.0, 0.0, 0.5, 0.0, 0.0, 0.0],
        ]),
        timestamps=np.array([10.1, 10.2, 10.3]),
        frame_id="odom",
    )


@pytest.fixture
def robot_kinematics():
    """Provide a stationary base and joint configuration."""
    return {
        "transform_world_base": np.eye(4),
        "joint_positions": np.zeros(2),
    }


def make_orchestrator(
    model,
    reference_builder,
    controller,
    lookahead_time=0.0,
):
    """Create an orchestrator using interchangeable components."""
    return TrackingControlOrchestrator(
        model=model,
        controller=controller,
        reference_builder=reference_builder,
        lookahead_time=lookahead_time,
    )


def test_linear_sampling_accepts_exact_single_forecast_sample(model, reference_builder):
    """An exact query needs no interpolation interval or extrapolation."""
    instance = TrackingControlOrchestrator(
        model, RecordingController(), reference_builder,
        prediction_sampling='linear', target_velocity_feedforward=True)
    forecast = PredictedTargetTrajectory(
        states=np.array([[1., 2., 3., .1, .2, .3]]),
        timestamps=np.array([10.1]), frame_id='odom')
    np.testing.assert_allclose(instance._select_target_state(forecast, 10.1), forecast.states[0])


def test_linear_prediction_interpolates_position_and_velocity(model, reference_builder):
    """Query time must determine both state components without rounding to the future."""
    instance = TrackingControlOrchestrator(
        model, RecordingController(), reference_builder,
        prediction_sampling='linear', target_velocity_feedforward=True)
    forecast = PredictedTargetTrajectory(
        states=np.array([[1., 2., 3., .1, .2, .3], [2., 4., 6., .2, .4, .6]]),
        timestamps=np.array([10.1, 10.3]), frame_id='odom')
    np.testing.assert_allclose(instance._select_target_state(forecast, 10.2),
                               (forecast.states[0] + forecast.states[1]) / 2)
    with pytest.raises(ValueError, match='horizon'):
        instance._select_target_state(forecast, 10.4)


def test_reference_generation(
    model, reference_builder, robot_state, prediction, robot_kinematics
):
    """Pass a consistent target-derived reference to the controller."""
    controller = RecordingController()
    orchestrator = make_orchestrator(
        model, reference_builder, controller
    )

    orchestrator.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
        **robot_kinematics,
    )

    control_input, received_model, model_data = controller.received

    assert received_model is model

    # Target = [3, 0, 0.5], camera = [0, 0, 0.5].
    # A radius of 1 places the desired camera at [2, 0, 0.5].
    np.testing.assert_allclose(
        control_input.reference.position,
        [2.0, 0.0, 0.5],
        atol=1e-10,
    )

    np.testing.assert_allclose(
        model_data.desired_end_effector_transform_world[:3, 3],
        control_input.reference.position,
    )

    np.testing.assert_allclose(
        model_data.desired_end_effector_twist_world,
        np.zeros(6),
    )

    assert control_input.target_prediction is prediction


def test_real_jacobian_control(
    model, reference_builder, robot_state, prediction, robot_kinematics
):
    """Verify a complete prediction-to-command computation."""
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

    orchestrator = make_orchestrator(
        model, reference_builder, controller
    )

    output = orchestrator.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
        **robot_kinematics,
    )

    # Current x = 0, desired x = 2.
    # Selected J = [1, 1, 0]; damping = 1.
    # Command = J.T * 2 / (2 + 1).
    np.testing.assert_allclose(
        output.command,
        [2.0 / 3.0, 2.0 / 3.0, 0.0],
        atol=1e-10,
    )

    assert output.mode == ControlMode.VELOCITY


def test_prediction_selection_by_time(
    model, reference_builder, robot_state, prediction, robot_kinematics
):
    """Select the earliest prediction sample not before query time."""
    controller = RecordingController()
    orchestrator = make_orchestrator(
        model, reference_builder, controller
    )

    orchestrator.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.15,
        **robot_kinematics,
    )

    reference = controller.received[0].reference

    # Query 10.15 selects 10.2: target x = 4, desired x = 3.
    np.testing.assert_allclose(
        reference.position,
        [3.0, 0.0, 0.5],
        atol=1e-10,
    )


def test_configured_lookahead(
    model, reference_builder, robot_state, prediction, robot_kinematics
):
    """Use the configured lookahead rather than a fixed sample index."""
    controller = RecordingController()
    orchestrator = make_orchestrator(
        model,
        reference_builder,
        controller,
        lookahead_time=0.25,
    )

    orchestrator.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
        **robot_kinematics,
    )

    # Query 10.25 selects 10.3: target x = 5, desired x = 4.
    np.testing.assert_allclose(
        controller.received[0].reference.position,
        [4.0, 0.0, 0.5],
        atol=1e-10,
    )


def test_reject_query_beyond_horizon(
    model, reference_builder, robot_state, prediction, robot_kinematics
):
    """Never extrapolate beyond the available prediction horizon."""
    orchestrator = make_orchestrator(
        model, reference_builder, RecordingController()
    )

    with pytest.raises(ValueError, match="horizon"):
        orchestrator.compute(
            robot_state=robot_state,
            prediction=prediction,
            now=10.4,
            **robot_kinematics,
        )


def test_reject_frame_mismatch(
    model, reference_builder, robot_state, prediction, robot_kinematics
):
    """Reject target predictions expressed in another frame."""
    invalid_prediction = PredictedTargetTrajectory(
        states=prediction.states,
        timestamps=prediction.timestamps,
        frame_id="map",
    )

    orchestrator = make_orchestrator(
        model, reference_builder, RecordingController()
    )

    with pytest.raises(ValueError, match="frames"):
        orchestrator.compute(
            robot_state=robot_state,
            prediction=invalid_prediction,
            now=10.0,
            **robot_kinematics,
        )


def test_reject_missing_target_xyz(
    model, reference_builder, robot_state, robot_kinematics
):
    """Require three position coordinates in predicted target states."""
    prediction = PredictedTargetTrajectory(
        states=np.ones((2, 2)),
        timestamps=np.array([10.1, 10.2]),
        frame_id="odom",
    )

    orchestrator = make_orchestrator(
        model, reference_builder, RecordingController()
    )

    with pytest.raises(ValueError, match="3 position components"):
        orchestrator.compute(
            robot_state=robot_state,
            prediction=prediction,
            now=10.0,
            **robot_kinematics,
        )


def test_reject_invalid_query_time(
    model, reference_builder, robot_state, prediction, robot_kinematics
):
    """Reject nonfinite prediction query times."""
    orchestrator = make_orchestrator(
        model, reference_builder, RecordingController()
    )

    with pytest.raises(ValueError, match="now"):
        orchestrator.compute(
            robot_state=robot_state,
            prediction=prediction,
            now=np.nan,
            **robot_kinematics,
        )


def test_reject_negative_lookahead(model, reference_builder):
    """Require a nonnegative prediction lookahead."""
    with pytest.raises(ValueError, match="lookahead_time"):
        make_orchestrator(
            model,
            reference_builder,
            RecordingController(),
            lookahead_time=-0.1,
        )


def test_reject_invalid_robot_state(
    model, reference_builder, prediction, robot_kinematics
):
    """Require the shared RobotState contract."""
    orchestrator = make_orchestrator(
        model, reference_builder, RecordingController()
    )

    with pytest.raises(TypeError, match="RobotState"):
        orchestrator.compute(
            robot_state=None,
            prediction=prediction,
            now=10.0,
            **robot_kinematics,
        )
