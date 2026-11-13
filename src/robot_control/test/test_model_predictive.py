"""Independent finite-horizon references and shared physical-envelope tests."""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from robot_control.controllers.model_predictive import ModelPredictiveController
from robot_control.interfaces.control_data import (
    ControlInput,
    PredictedTargetTrajectory,
    RobotState,
    TrackingReference,
)
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_control.tracking.velocity_task import CartesianVelocityTask, DistancePointingTask
from robot_control.tracking.tracking_reference_builder import TrackingReferenceBuilder
from robot_control.tracking.reference_selector import ReferenceSelector
from robot_control.tracking.radial_direction_policy import RadialDirectionPolicy
from robot_control.tracking.pose_reference_generator import PoseReferenceGenerator
from robot_models.reference.geometry.sphere_reference import SphereReference
from robot_models.constraints.velocity_limits import VelocityLimits
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.interfaces.model_data import ModelData


def fixture(n=1, horizon=2, safety=None, **overrides):
    """Radius-one reference: targets x=2 imply desired robot x=1."""
    builder = TrackingReferenceBuilder(
        ReferenceSelector(SphereReference(), RadialDirectionPolicy(radius=1.0, epsilon=1e-9)),
        PoseReferenceGenerator([1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
    )
    safety = safety or SafeControlPipeline(VelocityLimits(-10 * np.ones(n), 10 * np.ones(n)))
    params = {
        'task': CartesianVelocityTask(np.eye(6)[:1]),
        'safety': safety,
        'reference_builder': builder,
        'horizon': horizon,
        'step_duration': 1.0,
        'task_weights': [1.0],
        'task_scales': [1.0],
        'motion_weight': 0.01,
        'smoothness_weight': 0.0,
        'terminal_weight': 0.0,
        'max_iterations': 100,
        'solver_tolerance': 1e-9,
        'feasibility_tolerance': 1e-7,
    }
    params.update(overrides)
    controller = ModelPredictiveController(**params)
    forecast = PredictedTargetTrajectory(
        np.array([[2.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 4), np.arange(4.0), 'world'
    )
    state = RobotState(np.zeros(n), np.zeros(n), 0.0, 'world')
    inputs = ControlInput(
        state,
        TrackingReference([1.0, 0.0, 0.0]),
        target_prediction=forecast,
        target_state=forecast.states[0],
        dt=1.0,
        previous_velocity=np.zeros(n),
        evaluation_time=0.0,
    )
    spatial = np.zeros((6, n))
    spatial[0] = 1.0
    data = ModelData(np.eye(4), np.zeros(6), np.eye(4), np.zeros(6), spatial)
    return controller, inputs, SimpleNamespace(generalized_velocity_dimension=n), data


def test_scalar_two_step_matches_independent_closed_form():
    """Match hand-derived scalar normal equations, including the error sign."""
    # e1=-1+u0, e2=-1+u0+u1; motion cost=.01*(u/10)^2.
    instance, inputs, model, data = fixture()
    output = instance.compute(inputs, model, data)
    expected = np.linalg.solve([[2.0001, 1.0], [1.0, 1.0001]], [2.0, 1.0])
    np.testing.assert_allclose(output.predicted_commands[:, 0], expected, atol=1e-8)
    np.testing.assert_array_equal(output.command, output.predicted_commands[0])


def test_terminal_smoothness_and_units_match_independent_reference():
    """Preserve terminal, difference and unit-normalization terms independently."""
    instance, inputs, model, data = fixture(
        terminal_weight=2.0, smoothness_weight=0.5, task_scales=[2.0], motion_weight=3.0
    )
    inputs = replace(inputs, previous_velocity=np.array([0.3]))
    # Weighted tracking (.25 e1²+.75 e2²), R=.03, S=.005.
    expected = np.linalg.solve([[1.04, 0.745], [0.745, 0.785]], [1.0015, 0.75])
    output = instance.compute(inputs, model, data)
    np.testing.assert_allclose(output.predicted_commands[:, 0], expected, atol=1e-8)


def test_identical_present_but_different_future_changes_first_command():
    """Future forecast curvature must affect the first applied command."""
    instance, inputs, model, data = fixture()
    still = instance.compute(inputs, model, data).command
    states = inputs.target_prediction.states.copy()
    states[1:, 0] = [3.0, 4.0, 5.0]
    changed = replace(inputs, target_prediction=replace(inputs.target_prediction, states=states))
    future = instance.compute(changed, model, data).command
    assert future[0] > still[0] + 0.5


@pytest.mark.parametrize('n', [1, 2, 5])
def test_dimension_independence_determinism_and_first_command_safety(n):
    """Robot dimension must not change ordering, determinism or final feasibility."""
    instance, inputs, model, data = fixture(n=n)
    a = instance.compute(inputs, model, data)
    b = instance.compute(inputs, model, data)
    np.testing.assert_array_equal(a.command, b.command)
    safe = instance.safety.apply(a, previous_velocity=inputs.previous_velocity, dt=1.0)
    np.testing.assert_allclose(a.command, safe.command, atol=1e-10)
    assert a.predicted_commands.shape == (2, n)


def test_all_horizon_velocity_rate_and_joint_position_constraints():
    """Enforce physical envelopes throughout the whole planned sequence."""
    safety = SafeControlPipeline(
        VelocityLimits(np.array([-0.4]), np.array([0.4])),
        AccelerationLimits(np.array([-0.2]), np.array([0.2])),
        JointLimits(np.array([0.0]), np.array([0.3])),
        np.array([0]),
    )
    instance, inputs, model, data = fixture(safety=safety, horizon=3)
    output = instance.compute(inputs, model, data)
    velocities = output.predicted_commands[:, 0]
    assert np.max(np.abs(velocities)) <= 0.4 + 1e-7
    assert np.max(np.abs(np.diff(np.r_[0.0, velocities]))) <= 0.2 + 1e-7
    positions = np.cumsum(velocities)
    assert np.min(positions) >= -1e-7 and np.max(positions) <= 0.3 + 1e-7
    final = safety.apply(output, inputs.previous_velocity, inputs.robot_state.configuration, 1.0)
    np.testing.assert_allclose(final.command, output.command, atol=1e-7)


def test_infeasible_horizon_is_explicit_failure():
    """Never return a plan when mandatory motion crosses a future joint bound."""
    safety = SafeControlPipeline(
        VelocityLimits(np.array([0.5]), np.array([1.0])),
        joint_limits=JointLimits(np.array([0.0]), np.array([0.75])),
        joint_velocity_indices=np.array([0]),
    )
    instance, inputs, model, data = fixture(safety=safety)
    with pytest.raises(ValueError, match='MPC'):
        instance.compute(inputs, model, data)


@pytest.mark.parametrize('change', ['missing', 'short', 'stale', 'frame', 'future', 'nonfinite'])
def test_forecast_contract_rejects_unsupported_inputs(change):
    """Reject missing, stale, short, incompatible or corrupted forecasts."""
    instance, inputs, model, data = fixture()
    forecast = inputs.target_prediction
    if change == 'missing':
        forecast = None
    if change == 'short':
        forecast = replace(forecast, timestamps=np.array([0.0, 0.5]), states=forecast.states[:2])
    if change == 'stale':
        forecast = replace(forecast, timestamps=forecast.timestamps - 10.0)
    if change == 'frame':
        forecast = replace(forecast, frame_id='camera')
    if change == 'future':
        forecast = replace(forecast, timestamps=forecast.timestamps + 10.0)
    if change == 'nonfinite':
        forecast.states[0, 0] = np.nan
    with pytest.raises(ValueError):
        instance.compute(replace(inputs, target_prediction=forecast), model, data)


def test_solver_failure_and_nonfinite_results_are_rejected(monkeypatch):
    """A failed optimizer or unchecked result must never become a command."""
    import robot_control.controllers.model_predictive as module

    safety = SafeControlPipeline(VelocityLimits(np.array([-0.1]), np.array([0.1])))
    instance, inputs, model, data = fixture(safety=safety)
    for success, values in [(False, [0.0, 0.0]), (True, [np.nan, 0.0]), (True, [10.0, 10.0])]:
        monkeypatch.setattr(
            module,
            'minimize',
            lambda *a, **k: SimpleNamespace(
                success=success, x=np.array(values), message='injected failure', nit=1
            ),
        )
        with pytest.raises(ValueError, match='MPC'):
            instance.compute(inputs, model, data)


def test_distance_pointing_reuses_task_sign_and_forecast_geometry():
    """The shared range error decreases when the robot moves toward the target."""
    instance, inputs, model, data = fixture(
        task=DistancePointingTask(
            radius=1.0, forward_axis_local=[1.0, 0.0, 0.0], tangent_axis_local=[0.0, -1.0, 0.0]
        ),
        task_weights=[1.0, 1.0, 1.0],
        task_scales=[1.0, 1.0, 1.0],
    )
    command = instance.compute(inputs, model, data).command
    assert command[0] > 0  # moving toward target decreases range error.


def test_interpolation_uses_control_time_not_stale_robot_state():
    """Sample at control time with independently computed interpolated optima."""
    instance, inputs, model, data = fixture(horizon=2, step_duration=0.5)
    inputs = replace(inputs, dt=0.5, evaluation_time=0.5)
    states = inputs.target_prediction.states.copy()
    states[:, 0] = [2.0, 4.0, 6.0, 8.0]
    inputs = replace(inputs, target_prediction=replace(inputs.target_prediction, states=states))
    instance.compute(inputs, model, data)
    np.testing.assert_allclose(instance.last_diagnostics['forecast_times'], [0.5, 1.0, 1.5])
    # b1=-3, b2=-4; frozen pose x=0, radius=1, future targets x=4,5.
    # e1=-3+.5u0; e2=-4+.5u0+.5u1.
    expected = np.linalg.solve([[0.5001, 0.25], [0.25, 0.2501]], [3.5, 2.0])
    np.testing.assert_allclose(
        instance.compute(inputs, model, data).predicted_commands[:, 0], expected, atol=1e-7
    )


def test_forecast_velocity_does_not_double_count_target_motion():
    """Forecast positions enter once even when redundant velocities change."""
    instance, inputs, model, data = fixture()
    a = instance.compute(inputs, model, data).command
    states = inputs.target_prediction.states.copy()
    states[:, 3] = 100.0
    b = instance.compute(
        replace(inputs, target_prediction=replace(inputs.target_prediction, states=states)),
        model,
        data,
    ).command
    np.testing.assert_array_equal(
        a, b
    )  # supported times use positions, not duplicate feedforward.


@pytest.mark.parametrize('bounds', [(0.0, 0.0), (0.2, 0.2)])
def test_fixed_actuator_is_preserved(bounds):
    """Fixed physical velocity coordinates stay exact at every horizon step."""
    instance, inputs, model, data = fixture(
        safety=SafeControlPipeline(VelocityLimits(np.array([bounds[0]]), np.array([bounds[1]])))
    )
    result = instance.compute(inputs, model, data)
    np.testing.assert_allclose(result.predicted_commands, bounds[0], atol=1e-10)


def test_zero_acceleration_preserves_previous_command():
    """A locked command-rate envelope holds the previous command."""
    safety = SafeControlPipeline(
        VelocityLimits(-np.ones(1), np.ones(1)), AccelerationLimits(np.zeros(1), np.zeros(1))
    )
    instance, inputs, model, data = fixture(safety=safety)
    result = instance.compute(replace(inputs, previous_velocity=np.array([0.2])), model, data)
    np.testing.assert_allclose(result.predicted_commands, 0.2, atol=1e-8)


def test_antipodal_geometry_is_explicit_local_model_failure():
    """Undefined pointing charts must cause an explicit safe failure."""
    instance, inputs, model, data = fixture(
        task=DistancePointingTask(
            radius=1.0, forward_axis_local=[1.0, 0.0, 0.0], tangent_axis_local=[0.0, -1.0, 0.0]
        ),
        task_weights=[1.0, 1.0, 1.0],
        task_scales=[1.0, 1.0, 1.0],
    )
    states = inputs.target_prediction.states.copy()
    states[:, 0] = -2.0
    with pytest.raises(ValueError, match='antipodal'):
        instance.compute(
            replace(inputs, target_prediction=replace(inputs.target_prediction, states=states)),
            model,
            data,
        )


def test_conditioned_quadratic_converges_with_bounded_iterations():
    """Solve a coupled constrained eight-step problem within three iterations."""
    safety = SafeControlPipeline(VelocityLimits(-0.1 * np.ones(6), 0.1 * np.ones(6)))
    instance, inputs, model, data = fixture(
        n=6, horizon=8, safety=safety, max_iterations=3, step_duration=0.1
    )
    inputs = replace(inputs, dt=0.1)
    out = instance.compute(inputs, model, data)
    assert instance.last_diagnostics['solver_iterations'] <= 3
    np.testing.assert_allclose(out.command, 0.1, atol=1e-6)


@pytest.mark.parametrize('axis', [4, 5])
def test_cartesian_partial_angular_task_rejects_full_so3_antipode(axis):
    """A partial angular selection must not hide a full SO(3) discontinuity."""
    instance, inputs, model, data = fixture(task=CartesianVelocityTask(np.eye(6)[axis:axis + 1]))
    spatial = np.zeros((6, 1))
    spatial[axis] = 1.0
    data = replace(data, whole_body_jacobian_world=spatial)
    states = inputs.target_prediction.states.copy()
    states[:, :3] = [[-2.0, -0.01, 0.0], [-2.0, 0.0, 0.0], [-2.0, 0.01, 0.0], [-2.0, 0.02, 0.0]]
    with pytest.raises(ValueError, match='angular chart'):
        instance.compute(
            replace(inputs, target_prediction=replace(inputs.target_prediction, states=states)),
            model,
            data,
        )


def test_unequal_first_hold_integrates_joint_bounds_with_actual_dt():
    """The first joint increment uses real dt rather than forecast spacing."""
    safety = SafeControlPipeline(
        VelocityLimits(np.array([-0.4]), np.array([0.4])),
        joint_limits=JointLimits(np.array([0.0]), np.array([0.05])),
        joint_velocity_indices=np.array([0]),
    )
    instance, inputs, model, data = fixture(safety=safety, step_duration=0.3)
    result = instance.compute(replace(inputs, dt=0.1), model, data)
    # Reach q=.04 in the actual .1s hold; next .3s can add only .01.
    np.testing.assert_allclose(result.predicted_commands[:, 0], [0.4, 1 / 30], atol=1e-6)


def test_cartesian_angular_task_works_within_a_continuous_chart():
    """Angular Cartesian tasks remain supported away from chart cuts."""
    instance, inputs, model, data = fixture(task=CartesianVelocityTask(np.eye(6)[5:6]))
    spatial = np.zeros((6, 1))
    spatial[5] = 1.0
    data = replace(data, whole_body_jacobian_world=spatial)
    states = inputs.target_prediction.states.copy()
    states[:, 1] = [0.1, 0.2, 0.3, 0.4]
    result = instance.compute(
        replace(inputs, target_prediction=replace(inputs.target_prediction, states=states)),
        model,
        data,
    )
    assert np.all(np.isfinite(result.predicted_commands))
    assert result.command[0] > 0


def test_inactive_coordinate_and_zero_acceleration_are_feasible():
    """Degenerate but consistent velocity and acceleration envelopes work."""
    safety = SafeControlPipeline(
        VelocityLimits(np.array([0.0, -1.0]), np.array([0.0, 1.0])),
        AccelerationLimits(np.zeros(2), np.zeros(2)),
    )
    instance, inputs, model, data = fixture(n=2, safety=safety)
    result = instance.compute(inputs, model, data)
    np.testing.assert_allclose(result.predicted_commands, 0.0, atol=1e-10)
