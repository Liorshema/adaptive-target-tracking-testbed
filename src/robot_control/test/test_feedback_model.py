"""Mathematical velocity feedback through the existing task and safety contracts."""

from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from robot_configuration.components import schema_for
from robot_control.factory.component_registry import ComponentRegistry
from robot_control.interfaces.control_data import (
    ControlInput,
    ControlMode,
    ControlOutput,
    RobotState,
    TrackingReference,
)
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_control.tracking.velocity_task import CartesianVelocityTask, DistancePointingTask
from robot_models.base.differential_drive.kinematics import DifferentialDriveKinematics
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.velocity_limits import VelocityLimits
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.interfaces.model_data import ModelData
from robot_models.whole_body.composer import WholeBodyComposer
from robot_simulation.actuation.position_reference import PositionReference

SELECTION = np.eye(6)[:3]
SCHEMA = schema_for(
    {'gain': 2.0},
    units={'gain': 's^-1'},
    constraints={'gain': {'type': 'number', 'exclusiveMinimum': 0.0}},
)


class CartesianArm(ManipulatorModel):
    """Two Cartesian joints, keeping state and task dimensions distinct."""

    @property
    def dof(self):
        """Return two independent joint coordinates."""
        return 2

    def forward_kinematics(self, joint_positions):
        """Translate the end effector along the two joint axes."""
        result = np.eye(4)
        result[:2, 3] = joint_positions
        return result

    def jacobian(self, joint_positions):
        """Map joint speeds to world translation."""
        return np.eye(6)[:, :2]


@pytest.fixture
def inputs():
    """Use a four-command model, five-coordinate state and three-coordinate reference."""
    model = WholeBodyComposer(DifferentialDriveKinematics(0.1, 0.4), CartesianArm())
    desired = np.eye(4)
    desired[:3, 3] = [1.0, -2.0, 0.5]
    jacobian = np.zeros((6, 4))
    jacobian[:3] = [[1, 0, 1, 0], [0, 0, 0, 1], [0, 1, 0, 0]]
    data = ModelData(
        np.eye(4),
        np.zeros(6),
        desired,
        np.array([0.1, 0.2, 0.3, 0, 0, 0]),
        jacobian,
    )
    control_input = ControlInput(
        RobotState(np.zeros(5), np.array([0.1, 0.2, 0.3, 0.4]), 10.0, 'odom'),
        TrackingReference(desired[:3, 3], np.zeros(3)),
    )
    return control_input, model, data


def declaration(callback=None, **kwargs):
    """Declare a feedback law without adding it to production discovery."""
    from robot_control.controllers.feedback_model import velocity_feedback_model

    if callback is None:

        def callback(error, jacobian, target_rate, robot_velocity, params):
            return jacobian.T @ (-params['gain'] * error - target_rate)

    return velocity_feedback_model(
        'test_feedback',
        schema=SCHEMA,
        selection_matrix=SELECTION,
        frame_id='odom',
        **kwargs,
    )(callback)


def build(callback=None):
    """Construct with actual owner registry parameter and dependency validation."""
    declared = declaration(callback)
    registry = ComponentRegistry()
    registry.register(
        declared.identifier,
        declared.builder,
        defaults={'gain': 2.0},
        schema=declared.schema,
        capabilities=declared.capabilities,
        requirements=declared.requirements,
        dependencies=declared.dependencies,
    )
    return registry.create(
        'test_feedback', task=CartesianVelocityTask(SELECTION), safety=make_safety()
    )


def make_safety():
    """Declare test-owner velocity bounds independent of the adapter."""
    return SafeControlPipeline(VelocityLimits(-np.full(4, 4.0), np.full(4, 4.0)))


def test_feedback_uses_task_error_rate_and_model_command_dimension(inputs):
    """Catch sign, feedforward, command sizing and typed conversion regressions."""
    result = build().compute(*inputs)
    # e=[-1,2,-.5], target_rate=[-.1,-.2,-.3], J columns x,z,x,y.
    np.testing.assert_allclose(result.command, [2.1, 1.3, 2.1, -3.8])
    assert result.mode is ControlMode.VELOCITY
    assert inputs[0].robot_state.configuration.size == 5
    assert inputs[0].reference.position.size == 3
    assert inputs[0].robot_state.timestamp == 10.0


def test_raw_feedback_follows_normal_safety_projection_and_actuation(inputs):
    """Catch bypasses of owner velocity/acceleration safety or actuator semantics."""
    output = build().compute(*inputs)
    safety = SafeControlPipeline(
        VelocityLimits(-np.full(4, 4.0), np.full(4, 4.0)),
        AccelerationLimits(-np.full(4, 2.0), np.full(4, 2.0)),
    )
    safe = safety.apply(output, previous_velocity=np.zeros(4), dt=0.1)
    np.testing.assert_allclose(safe.command, [0.2, 0.2, 0.2, -0.2])
    np.testing.assert_allclose(output.command, [2.1, 1.3, 2.1, -3.8])
    actuator = PositionReference([-1, -1], [1, 1], [4, 4], 0.1, 0.1)
    actuator.initialize([0, 0])
    with pytest.raises(ValueError, match='unsafe velocity'):
        actuator.step([5.0, 0.0], [0, 0], 0.1)
    np.testing.assert_allclose(actuator.step(safe.command[2:], [0, 0], 0.1), [0.02, -0.02])


@pytest.mark.parametrize(
    'returned',
    [
        np.zeros(3),
        np.zeros((4, 1)),
        [0, 0, np.inf, 0],
        [0, np.nan, 0, 0],
        [0, 0, 4.01, 0],
        np.array([1j, 0, 0, 0]),
        ControlOutput(np.zeros(4), ControlMode.EFFORT),
    ],
)
def test_invalid_commands_are_rejected_before_safety(inputs, returned):
    """Catch nonfinite, malformed, physically unsafe and incorrectly typed commands."""
    controller = build(lambda *args: returned)
    with pytest.raises((TypeError, ValueError), match='command|finite|shape|array|unsafe'):
        controller.compute(*inputs)


@pytest.mark.parametrize('mode', [ControlMode.EFFORT, ControlMode.ACCELERATION, 'velocity'])
def test_unsupported_actuation_modes_fail_at_declaration(mode):
    """Enforce the supported typed velocity mode before a controller is built."""
    with pytest.raises(ValueError, match='VELOCITY|velocity'):
        declaration(mode=mode)


@pytest.mark.parametrize(
    'task',
    [
        DistancePointingTask(
            radius=1.0, forward_axis_local=[1, 0, 0], tangent_axis_local=[0, 1, 0]
        ),
        CartesianVelocityTask(np.eye(6)[3:]),
        object(),
    ],
)
def test_incompatible_tasks_fail_before_feedback(task):
    """Do not allow a different task class or Cartesian selection into the callback."""
    with pytest.raises((TypeError, ValueError), match='CartesianVelocityTask|selection'):
        declaration().builder(task=task, safety=make_safety(), gain=2.0)


def test_input_frames_dimensions_and_mutable_model_values_are_validated(inputs):
    """Reject incompatible frame/layout and nonfinite model quantities before evaluation."""
    control_input, model, data = inputs
    controller = build()
    with pytest.raises(ValueError, match='frame'):
        controller.compute(
            replace(control_input, robot_state=replace(control_input.robot_state, frame_id='map')),
            model,
            data,
        )
    with pytest.raises(ValueError, match='velocity|dimension'):
        controller.compute(
            replace(
                control_input, robot_state=replace(control_input.robot_state, velocity=np.zeros(3))
            ),
            model,
            data,
        )
    with pytest.raises(ValueError, match='reference'):
        controller.compute(
            replace(control_input, reference=TrackingReference(np.zeros(2))), model, data
        )
    wrong = deepcopy(data)
    wrong.whole_body_jacobian_world = np.zeros((6, 5))
    with pytest.raises(ValueError, match='Jacobian|jacobian'):
        controller.compute(control_input, model, wrong)
    data.desired_end_effector_twist_world[0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        controller.compute(control_input, model, data)


def test_callback_receives_detached_inputs_and_isolated_validated_parameters(inputs):
    """Callback mutation cannot leak into robot/task state or future invocations."""

    def mutate(error, jacobian, target_rate, robot_velocity, params):
        result = np.full(robot_velocity.shape, params['gain'])
        params['gain'] = 99.0
        robot_velocity[:] = 999.0
        jacobian[:] = 0.0
        return result

    controller = build(mutate)
    np.testing.assert_allclose(controller.compute(*inputs).command, [2, 2, 2, 2])
    np.testing.assert_allclose(controller.compute(*inputs).command, [2, 2, 2, 2])
    np.testing.assert_allclose(inputs[0].robot_state.velocity, [0.1, 0.2, 0.3, 0.4])
    assert inputs[2].whole_body_jacobian_world[0, 0] == 1.0


@pytest.mark.parametrize('gain', [-1, True, np.nan, '2'])
def test_direct_builder_does_not_bypass_parameter_schema(gain):
    """Direct construction retains the same finite positive parameter contract."""
    with pytest.raises(ValueError):
        declaration().builder(
            task=CartesianVelocityTask(SELECTION), safety=make_safety(), gain=gain
        )


def test_missing_task_dependency_is_not_hidden_in_registration():
    """A callback cannot silently acquire an implicit global task."""
    with pytest.raises(TypeError, match='task'):
        declaration().builder(gain=2.0)


def test_owner_discovery_reads_defaults_and_isolates_test_plugins(tmp_path, monkeypatch):
    """A trusted local declaration needs no factory switch or production inventory entry."""
    import importlib
    import sys

    import robot_control.controllers as namespace
    import yaml

    from robot_control.configuration import control_configuration_path
    from robot_control.factory.project_components import create_project_registries

    production = create_project_registries()['controller'].names()
    (tmp_path / 'local_feedback.py').write_text("""
import numpy as np
from robot_control.controllers.feedback_model import velocity_feedback_model

@velocity_feedback_model(
    'local_feedback', frame_id='odom', selection_matrix=np.eye(6)[:3],
    schema={'type': 'object', 'additionalProperties': False,
            'properties': {'gain': {'type': 'number', 'exclusiveMinimum': 0, 'unit': 's^-1'}},
            'required': ['gain']})
def feedback(error, jacobian, target_rate, robot_velocity, params):
    return np.zeros_like(robot_velocity)

COMPONENTS = (feedback,)
""")
    data = yaml.safe_load(control_configuration_path().read_text())
    data['controllers']['local_feedback'] = {'parameters': {'gain': 0.25}}
    path = tmp_path / 'control.yaml'
    path.write_text(yaml.safe_dump(data))
    with monkeypatch.context() as context:
        context.setattr(namespace, '__path__', [*namespace.__path__, str(tmp_path)])
        importlib.invalidate_caches()
        try:
            registry = create_project_registries(path)['controller']
            assert registry.describe('local_feedback')['defaults'] == {'gain': 0.25}
            assert registry.describe('local_feedback')['dependencies'] == ('task', 'safety')
            controller = registry.create(
                'local_feedback',
                task=CartesianVelocityTask(SELECTION),
                safety=make_safety(),
            )
            assert controller is not None
            with pytest.raises(ValueError):
                registry.create('local_feedback', {'typo': 1})
        finally:
            sys.modules.pop('robot_control.controllers.local_feedback', None)
    importlib.invalidate_caches()
    assert create_project_registries()['controller'].names() == production
