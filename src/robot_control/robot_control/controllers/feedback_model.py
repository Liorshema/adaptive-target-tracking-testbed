"""
Optional mathematical adapter for a declared selected Cartesian velocity task.

The callback is ``feedback(error, jacobian, target_rate, robot_velocity, params)``
and returns a generalized velocity vector, in the model's coordinate order.
The existing task convention is ``error_dot = jacobian @ u + target_rate``;
``error`` is current minus desired, not a robot configuration minus a reference.
Arrays and parameters passed to the callback are detached per evaluation.

Only CartesianVelocityTask with the exact declared selection is supported. Each
selection row must select translation (m) or rotation (rad), without mixing units.
All spatial quantities use the declared world frame; generalized velocities use
the model's body-base/joint coordinates (m/s or rad/s). No time is advanced and
the existing robot/reference timestamps and downstream freshness checks remain
authoritative. Stateful, alternate-chart, effort and acceleration laws use the
full Controller interface instead.

The injected owner safety envelope rejects raw velocities outside its configured
velocity limits. Independent final safety still projects acceleration and joint
position constraints using the current cycle context before normal actuation.
This adapter neither defines physical limits nor replaces that final projection.
"""

from copy import deepcopy

import numpy as np

from robot_configuration.components import validate_schema
from robot_configuration.model_registration import ModelRegistration
from robot_control.interfaces.control_data import ControlInput, ControlMode, ControlOutput
from robot_control.interfaces.controller import Controller
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_control.tracking.velocity_task import CartesianVelocityTask
from robot_models.interfaces.model_data import ModelData
from robot_models.interfaces.robot_model import RobotModel


def _finite_array(value, shape, name):
    if np.iscomplexobj(value):
        raise ValueError(f'{name} must be a real array')
    result = np.array(value, dtype=float, copy=True)
    if result.shape != shape:
        raise ValueError(f'{name} must have shape {shape}')
    if not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite')
    return result


class _VelocityFeedbackController(Controller):
    """Adapt one trusted function, with explicit task and safety dependencies."""

    def __init__(self, callback, selection, frame_id, schema, *, task, safety, parameters):
        if type(task) is not CartesianVelocityTask:
            raise TypeError('feedback requires CartesianVelocityTask')
        if not np.array_equal(task.task.selection_matrix, selection):
            raise ValueError('task selection must match the declared selection')
        if not isinstance(safety, SafeControlPipeline):
            raise TypeError('feedback requires shared SafeControlPipeline safety')
        self._parameters = validate_schema(parameters, schema)
        self._callback = callback
        self._task = task
        self._selection = selection.copy()
        self._frame_id = frame_id
        self._safety = safety

    def compute(self, control_input, model, model_data):
        """Validate task inputs, evaluate equations, and package a bounded velocity."""
        if not isinstance(control_input, ControlInput):
            raise TypeError('control_input must be ControlInput')
        if not isinstance(model, RobotModel):
            raise TypeError('model must implement RobotModel')
        if not isinstance(model_data, ModelData):
            raise TypeError('model_data must be ModelData')
        if control_input.robot_state.frame_id != self._frame_id:
            raise ValueError('robot state frame must match the declared world frame')
        for trajectory in (control_input.reference_trajectory, control_input.target_prediction):
            if trajectory is not None and trajectory.frame_id != self._frame_id:
                raise ValueError('reference/prediction frame must match the declared world frame')
        dimension = model.generalized_velocity_dimension
        velocity = _finite_array(
            control_input.robot_state.velocity, (dimension,), 'robot velocity'
        )
        configuration = np.asarray(control_input.robot_state.configuration)
        if (
            configuration.ndim != 1
            or not configuration.size
            or not np.all(np.isfinite(configuration))
        ):
            raise ValueError('robot configuration must be a finite vector')
        _finite_array(control_input.reference.position, (3,), 'reference position')
        if control_input.reference.velocity is not None:
            _finite_array(control_input.reference.velocity, (3,), 'reference velocity')
        for name, shape in (
            ('end_effector_transform_world', (4, 4)),
            ('end_effector_twist_world', (6,)),
            ('desired_end_effector_transform_world', (4, 4)),
            ('desired_end_effector_twist_world', (6,)),
            ('whole_body_jacobian_world', (6, dimension)),
        ):
            _finite_array(getattr(model_data, name), shape, name)
        # Task objects and ModelData are mutable: recheck the declared contract per cycle.
        if not np.array_equal(self._task.task.selection_matrix, self._selection):
            raise ValueError('task selection no longer matches the declaration')
        task = self._task.evaluate(control_input, model_data)
        rows = self._selection.shape[0]
        error = _finite_array(task.error, (rows,), 'task error')
        jacobian = _finite_array(task.jacobian, (rows, dimension), 'task jacobian')
        target_rate = _finite_array(task.target_rate, (rows,), 'task target_rate')
        if task.alternative_errors:
            raise ValueError('alternate task charts require the full Controller interface')
        lower = _finite_array(
            self._safety.velocity_limits.lower, (dimension,), 'velocity lower bound'
        )
        upper = _finite_array(
            self._safety.velocity_limits.upper, (dimension,), 'velocity upper bound'
        )
        if np.any(lower > upper):
            raise ValueError('unsafe owner velocity bounds')
        result = self._callback(error, jacobian, target_rate, velocity, deepcopy(self._parameters))
        if isinstance(result, ControlOutput):
            raise TypeError('feedback must return a command array, not ControlOutput')
        command = _finite_array(result, (dimension,), 'feedback command')
        if np.any(command < lower) or np.any(command > upper):
            raise ValueError('unsafe feedback command exceeds owner velocity limits')
        return ControlOutput(command=command, mode=ControlMode.VELOCITY)


def velocity_feedback_model(
    identifier,
    *,
    schema,
    selection_matrix,
    frame_id,
    mode=ControlMode.VELOCITY,
):
    """
    Declare a function as an owner-local ModelRegistration, without registering it.

    Put the resulting declaration in the owning module's ``COMPONENTS`` tuple and
    its parameter defaults in ``control.yaml`` under ``controllers.<id>.parameters``.
    ``schema`` supplies types, constraints and units, with no numerical defaults.
    The normal control factory injects ``task`` and ``safety`` dependencies.
    """
    if mode is not ControlMode.VELOCITY:
        raise ValueError('velocity feedback supports ControlMode.VELOCITY only')
    selection = np.array(selection_matrix, dtype=float, copy=True)
    if (
        selection.ndim != 2
        or selection.shape[1] != 6
        or not selection.shape[0]
        or not np.all(np.isfinite(selection))
    ):
        raise ValueError('selection_matrix must be finite with shape (task_dimension, 6)')
    units = []
    for row in selection:
        translation, rotation = np.any(row[:3] != 0), np.any(row[3:] != 0)
        if translation == rotation:
            raise ValueError(
                'selection rows must select translation or rotation without mixed units'
            )
        units.append('m' if translation else 'rad')
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError('frame_id must declare the world frame')
    if not isinstance(schema, dict) or schema.get('type') != 'object':
        raise ValueError('parameter schema must describe an object')
    parameter_schema = deepcopy(schema)

    def decorate(callback):
        if not callable(callback):
            raise TypeError('feedback must be callable')

        def builder(*, task, safety, **parameters):
            return _VelocityFeedbackController(
                callback,
                selection,
                frame_id,
                parameter_schema,
                task=task,
                safety=safety,
                parameters=parameters,
            )

        return ModelRegistration(
            identifier=identifier,
            builder=builder,
            schema=parameter_schema,
            dependencies=('task', 'safety'),
            capabilities={
                'deployable': True,
                'runtime_validation': 'unit_tested_extension',
                'runtime_checks': (
                    'exact Cartesian task, frame, model dimensions and owner velocity envelope'
                ),
                'task_binding': 'task',
                'task': 'cartesian',
                'task_dimension': selection.shape[0],
                'selection_matrix': selection.tolist(),
                'input': 'CartesianVelocityTask(error,jacobian,target_rate),robot_velocity',
                'input_state_layout': 'model configuration and generalized velocity; 3D reference',
                'error_convention': 'current-minus-desired; error_dot=jacobian@u+target_rate',
                'task_units': units,
                'frame_id': frame_id,
                'command_mode': 'velocity',
                'command_layout': 'body_base_joint_velocity',
                'command_dimension': 'model.generalized_velocity_dimension',
                'command_units': 'model generalized velocity coordinates: m/s or rad/s',
                'safety': 'owner velocity envelope rejection; independent final safety projection',
            },
        )

    return decorate
