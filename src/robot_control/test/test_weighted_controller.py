"""Test constrained allocation, task progress and time-sensitive safety bounds."""

import importlib

import numpy as np
import pytest
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits
from robot_models.interfaces.model_data import ModelData

from robot_control.interfaces.control_data import ControlInput, RobotState, TrackingReference
from robot_control.safety.safe_control_pipeline import SafeControlPipeline


def controller(weights, safety=None, **overrides):
    """Configure a synthetic robot independently of the physical four-joint arm."""
    module = 'robot_control.controllers.weighted_least_squares'
    assert importlib.util.find_spec(module), 'Weighted constrained controller is missing'
    task_module = importlib.import_module('robot_control.tracking.velocity_task')
    task = task_module.CartesianVelocityTask(selection_matrix=np.eye(6)[:1])
    if safety is None:
        safety = SafeControlPipeline(
            VelocityLimits(np.array([-1., -1.]), np.array([1., 1.])),
            joint_limits=JointLimits(np.array([-1.]), np.array([1.])),
            joint_velocity_indices=np.array([1]))
    parameters = {
        'task': task, 'safety': safety, 'gains': [1.],
        'task_weights': [1.], 'task_scales': [1.], 'motion_weights': weights,
        'damping': .1, 'posture_weight': 0., 'posture_gain': 0.,
        'smoothness_weight': 0., 'limit_avoidance_weight': 0.,
    }
    parameters.update(overrides)
    return importlib.import_module(module).WeightedLeastSquaresController(**parameters)


def compute(controller_instance, joint=0., desired=.2):
    """Use a two-actuator synthetic translation Jacobian to inspect allocation."""
    from types import SimpleNamespace
    current = np.eye(4)
    target = np.eye(4)
    target[0, 3] = desired
    data = ModelData(
        current, np.zeros(6), target, np.zeros(6),
        np.array([[1., 1.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.]]))
    state = RobotState(np.array([0., joint]), np.zeros(2), 1., 'world')
    inputs = ControlInput(state, TrackingReference(np.array([desired, 0., 0.])),
                          dt=.1, previous_velocity=np.zeros(2))
    return controller_instance.compute(
        inputs, SimpleNamespace(generalized_velocity_dimension=2), data)


def test_motion_weights_change_allocation_without_forcing_motion():
    """Either actuator can be favored while achieving the same task direction."""
    base = compute(controller([1., 100.])).command
    arm = compute(controller([100., 1.])).command
    assert base[0] > 20 * base[1] > 0
    assert arm[1] > 20 * arm[0] > 0


def test_optimizer_uses_next_step_joint_bounds():
    """A costly base still takes motion when the preferred joint hits its limit."""
    instance = controller([100., 1.])
    output = compute(instance, joint=1., desired=3.).command
    assert -1 <= output[0] <= 1
    assert -1 <= output[1] <= 0
    assert np.all(np.isfinite(output))


def test_antipodal_camera_escapes_in_the_feasible_direction_at_a_joint_limit():
    """Either antipodal turning direction is valid, even if one violates a bound."""
    from types import SimpleNamespace
    from robot_control.tracking.velocity_task import DistancePointingTask
    safety = SafeControlPipeline(
        VelocityLimits(np.array([-1.]), np.array([1.])),
        joint_limits=JointLimits(np.array([0.]), np.array([1.])),
        joint_velocity_indices=np.array([0]))
    instance = controller([1.], safety=safety,
                          task=DistancePointingTask(radius=2., forward_axis_local=[0., 0., 1.],
                                                    tangent_axis_local=[0., 1., 0.]),
                          gains=[1., 1., 1.], task_weights=[1., 1., 1.],
                          task_scales=[1., 1., 1.])
    spatial = np.zeros((6, 1))
    spatial[4, 0] = 1.
    data = ModelData(np.eye(4), np.zeros(6), np.eye(4), np.zeros(6), spatial)
    inputs = ControlInput(RobotState(np.zeros(1), np.zeros(1), 1., 'world'),
                          TrackingReference(np.array([0., 0., -2.])),
                          target_state=np.array([0., 0., -2., 0., 0., 0.]),
                          dt=.1, previous_velocity=np.zeros(1))
    output = instance.compute(inputs, SimpleNamespace(generalized_velocity_dimension=1), data)
    assert 0 < output.command[0] <= 1.


def test_optimizer_eliminates_fixed_velocity_component_exactly():
    """A fixed component must contribute its motion to the remaining task."""
    safety = SafeControlPipeline(VelocityLimits(
        np.array([.25, -1.]), np.array([.25, 1.])))
    output = compute(controller([1., 1.], safety=safety), desired=.2)

    # The remaining scalar objective is (dq + .05)^2 + .01*dq^2.
    np.testing.assert_allclose(output.command, [.25, -.05 / 1.01], atol=1e-12)


def test_optimizer_handles_all_components_fixed_by_acceleration():
    """A zero acceleration interval must hold the previous zero velocity."""
    safety = SafeControlPipeline(
        VelocityLimits(-np.ones(2), np.ones(2)),
        acceleration_limits=AccelerationLimits(np.zeros(2), np.zeros(2)))
    instance = controller([1., 1.], safety=safety)
    output = compute(instance, desired=3.)

    np.testing.assert_array_equal(output.command, [0., 0.])
    assert instance.last_diagnostics['solver_success']
    assert instance.last_diagnostics['active_bounds'] == 2


@pytest.mark.parametrize('parameters', [
    {'gains': [0.]},
    {'task_weights': [-1.]},
    {'task_scales': [np.nan]},
    {'task_scales': [1., 1.]},
    {'motion_weights': [1.]},
])
def test_optimizer_rejects_invalid_normalization(parameters):
    """Invalid units or dimensions must fail before reaching the solver."""
    with pytest.raises(ValueError):
        controller([1., 1.], **parameters)


def test_optimizer_rejects_zero_physical_velocity_scale():
    """Actuator normalization requires a nonzero configured speed scale."""
    safety = SafeControlPipeline(VelocityLimits(
        np.array([0., -1.]), np.array([0., 1.])))

    with pytest.raises(ValueError, match='nonzero actuator velocity scales'):
        controller([1., 1.], safety=safety)
