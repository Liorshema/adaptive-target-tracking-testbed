"""
Optional mathematical adapter for Cartesian model-local accelerations.

The function receives ``(position, velocity, time, parameters, context)`` and
returns a length-d acceleration in m/s². Positions (m), velocities (m/s), and
all world-context vectors share one fixed world frame; time is the existing
simulation clock in seconds, including intermediate integrator stage times.
The adapter adds the configured environment field exactly once. Control is
unused, as for the built-in target dynamics. Deployment through TargetState
currently supports d=3; other dimensions are only usable through Dynamics.

A decorated function is a ModelRegistration declaration, not a global registry
side effect. Export declarations in the owner module's ``COMPONENTS`` tuple and
put every numerical default in the authoritative owner dynamics YAML.
"""

from collections.abc import Mapping
from copy import deepcopy
from numbers import Integral, Real
from types import MappingProxyType

import numpy as np

from robot_configuration.components import validate_schema
from robot_configuration.model_registration import ModelRegistration

from robot_simulation.interfaces.disturbance import ContextualAccelerationField
from robot_simulation.interfaces.dynamics import Dynamics


class AccelerationDynamics(Dynamics):
    """Execute a validated mathematical acceleration using the existing contract."""

    def __init__(
        self,
        function,
        *,
        dimension,
        parameters,
        schema,
        frame_id,
        required_context,
        environment_field=None,
    ):
        """Detach schema-validated parameters and retain explicit runtime inputs."""
        self._function = function
        self.dimension = dimension
        self.parameters = MappingProxyType(validate_schema(parameters, schema))
        self.frame_id = frame_id
        self.required_context = required_context
        self.environment_field = environment_field

    def derivative(self, state, control, time, context):
        """Return [v, model-local acceleration + environment acceleration]."""
        del control
        if np.iscomplexobj(state):
            raise ValueError('state must be real')
        state = np.asarray(state, dtype=float)
        if state.shape != (2 * self.dimension,) or not np.isfinite(state).all():
            raise ValueError('state must be finite [position, velocity] of declared dimension')
        if isinstance(time, bool) or not isinstance(time, Real) or not np.isfinite(time):
            raise ValueError('time must be a finite scalar in simulation seconds')
        if not isinstance(context, Mapping):
            raise ValueError('context must be a mapping in the declared fixed world frame')
        missing = [
            key for key in self.required_context if key not in context or context[key] is None
        ]
        if missing:
            raise ValueError(f'Missing required context: {missing}')
        if 'frame_id' in context and context['frame_id'] != self.frame_id:
            raise ValueError('context frame_id differs from the declared fixed world frame')
        if 'tracking_position_world' in context:
            if np.iscomplexobj(context['tracking_position_world']):
                raise ValueError('tracking_position_world must be real')
            tracking = np.asarray(context['tracking_position_world'], dtype=float)
            if tracking.shape != (self.dimension,) or not np.isfinite(tracking).all():
                raise ValueError(
                    'tracking_position_world must be a finite declared-dimension vector'
                )
        position = state[: self.dimension].copy()
        velocity = state[self.dimension:].copy()
        position.flags.writeable = False
        velocity.flags.writeable = False
        acceleration = self._acceleration(
            self._function(
                position,
                velocity,
                time,
                MappingProxyType(deepcopy(dict(self.parameters))),
                MappingProxyType(deepcopy(dict(context))),
            ),
            'Model acceleration',
        )
        if self.environment_field is not None:
            field = self.environment_field
            external = (
                field.acceleration_with_context(position, time, context.get('physical_base'))
                if isinstance(field, ContextualAccelerationField)
                else field.acceleration(position, time)
            )
            acceleration = acceleration + self._acceleration(external, 'External acceleration')
        if not np.isfinite(acceleration).all():
            raise ValueError('Combined acceleration must be finite')
        return np.concatenate((velocity, acceleration))

    def _acceleration(self, value, label):
        if np.iscomplexobj(value):
            raise ValueError(f'{label} must be real')
        acceleration = np.asarray(value, dtype=float)
        if acceleration.shape != (self.dimension,) or not np.isfinite(acceleration).all():
            raise ValueError(f'{label} must be a finite declared-dimension vector')
        return acceleration


def acceleration_model(
    identifier, *, dimension, schema, required_context=(), frame_id='world', requirements=None
):
    """
    Declare a local acceleration function with strict schema, without defaults.

    ``parameters`` is a detached read-only mapping. ``context`` is a read-only
    mapping of existing runtime inputs, never an implicit tracker/base source.
    Required context keys must be present and non-null. The existing engine
    supplies world vectors without frame labels; direct callers may additionally
    pass ``context['frame_id']``, which must equal ``frame_id``. No transforms or
    time resampling are performed.
    """
    if isinstance(dimension, bool) or not isinstance(dimension, Integral) or dimension <= 0:
        raise ValueError('dimension must be a positive integer')
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError('frame_id must name the fixed world frame')
    if isinstance(required_context, str):
        raise ValueError('required_context must be a sequence of nonempty keys')
    required_context = tuple(required_context)
    if any(not isinstance(key, str) or not key.strip() for key in required_context):
        raise ValueError('required_context must contain nonempty keys')
    schema = deepcopy(schema)

    def declare(function):
        if not callable(function):
            raise TypeError('Acceleration function must be callable')

        def builder(environment_field=None, **parameters):
            return AccelerationDynamics(
                function,
                dimension=dimension,
                parameters=parameters,
                schema=schema,
                frame_id=frame_id,
                required_context=required_context,
                environment_field=environment_field,
            )

        return ModelRegistration(
            identifier=identifier,
            builder=builder,
            schema=schema,
            capabilities={
                'state_layout': 'position_velocity',
                'state_dimension': 2 * dimension,
                'spatial_dimension': dimension,
                'context_frame': 'same fixed world frame as target',
                'frame_id': frame_id,
                'time_units': 's',
                'position_units': 'm',
                'velocity_units': 'm/s',
                'acceleration_units': 'm/s^2',
                'acceleration_convention': 'model_local_plus_environment',
                'required_context': list(required_context),
                'deployable': dimension == 3,
                'runtime_validation': 'unit_tested_extension',
            },
            requirements={} if requirements is None else requirements,
            dependencies=('environment_field',),
        )

    return declare
