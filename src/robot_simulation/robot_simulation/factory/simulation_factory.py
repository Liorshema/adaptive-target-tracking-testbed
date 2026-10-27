"""Configuration-driven construction of simulation components."""

from collections.abc import Mapping
from typing import Any

import numpy as np

from robot_simulation.disturbances.base_guidance import BaseRelativeGuidance
from robot_simulation.disturbances.composite_field import CompositeAccelerationField
from robot_simulation.disturbances.sinusoidal_field import SinusoidalAccelerationField
from robot_simulation.disturbances.sinusoidal_field import SinusoidalFieldParameters
from robot_simulation.disturbances.vector_field import RestoringVectorField
from robot_simulation.disturbances.vector_field import VectorFieldParameters
from robot_simulation.dynamics.target.fight_flight import FightFlightDynamics
from robot_simulation.dynamics.target.fight_flight import FightFlightParameters
from robot_simulation.dynamics.target.fight_flight import SaturatedFightFlightParameters
from robot_simulation.dynamics.target.fight_flight import TangentialInteractionParameters
from robot_simulation.engine.simulation_engine import SimulationEngine
from robot_simulation.factory.registry import ComponentRegistry
from robot_simulation.integration.rk4 import RK4Integrator


def make_fight_flight(
    environment_field=None,
    tangential=None,
    **parameters: Any,
) -> FightFlightDynamics:
    """Construct Fight/Flight from parameter values."""
    return FightFlightDynamics(
        parameters=FightFlightParameters(**parameters),
        environment_field=environment_field,
        tangential=None if tangential is None else TangentialInteractionParameters(**tangential),
    )


def make_saturated_fight_flight(
    environment_field=None,
    tangential=None,
    **parameters: Any,
) -> FightFlightDynamics:
    """Select bounded radial interaction while reusing the dynamics engine."""
    return FightFlightDynamics(
        parameters=SaturatedFightFlightParameters(**parameters),
        environment_field=environment_field,
        tangential=None if tangential is None else TangentialInteractionParameters(**tangential),
    )


def make_restoring_field(
    center,
    gain_matrix,
) -> RestoringVectorField:
    """Construct a linear restoring acceleration field."""
    return RestoringVectorField(
        parameters=VectorFieldParameters(
            center=np.asarray(center, dtype=float),
            gain_matrix=np.asarray(gain_matrix, dtype=float),
        )
    )


def make_sinusoidal_field(
    amplitude,
    frequency,
    phase,
) -> SinusoidalAccelerationField:
    """Construct a configurable periodic acceleration field."""
    return SinusoidalAccelerationField(
        SinusoidalFieldParameters(
            amplitude=np.asarray(amplitude, dtype=float),
            frequency=np.asarray(frequency, dtype=float),
            phase=np.asarray(phase, dtype=float),
        )
    )


def make_rk4() -> RK4Integrator:
    """Construct an RK4 integrator."""
    return RK4Integrator()


class SimulationFactory:
    """Assemble a simulation from registered component types."""

    def __init__(
        self,
        dynamics_registry: ComponentRegistry | None = None,
        field_registry: ComponentRegistry | None = None,
        integrator_registry: ComponentRegistry | None = None,
    ) -> None:
        self.dynamics_registry = (
            dynamics_registry if dynamics_registry is not None else ComponentRegistry()
        )
        self.field_registry = field_registry if field_registry is not None else ComponentRegistry()
        self.integrator_registry = (
            integrator_registry if integrator_registry is not None else ComponentRegistry()
        )

    @classmethod
    def with_defaults(cls, configuration_path=None) -> 'SimulationFactory':
        """Register the currently implemented built-in components."""
        factory = cls()

        from robot_configuration.components import schema_for
        from robot_configuration.resources import package_config_path
        from robot_configuration.yaml_loader import load_yaml

        factory.dynamics_defaults_path = configuration_path or package_config_path(
            __file__, 'robot_simulation', 'dynamics.yaml'
        )
        data = load_yaml(factory.dynamics_defaults_path)
        if set(data) != {'dynamics'}:
            raise ValueError('Dynamics defaults must contain only mathematical dynamics groups')
        for name, builder in [
            ('fight_flight', make_fight_flight),
            ('saturated_fight_flight', make_saturated_fight_flight),
        ]:
            defaults = data['dynamics'][name]['parameters']
            units = {
                'alpha': 's^-2',
                'a_max': 'm/s^2',
                'transition_length': 'm',
                'preferred_distance': 'm',
                'damping': 's^-1',
                'distance_epsilon': 'm',
            }
            constraints = {key: {'type': 'number', 'minimum': 0.0} for key in defaults}
            for key in ['a_max', 'transition_length', 'preferred_distance', 'distance_epsilon']:
                if key in constraints:
                    constraints[key] = {'type': 'number', 'exclusiveMinimum': 0.0}
            schema = schema_for(defaults, units=units, constraints=constraints)
            schema['properties']['tangential'] = {
                'type': 'object',
                'additionalProperties': False,
                'properties': {
                    'acceleration': {'type': 'number', 'minimum': 0.0, 'unit': 'm/s^2'},
                    'axis_world': {
                        'type': 'array',
                        'minItems': 3,
                        'maxItems': 3,
                        'items': {'type': 'number'},
                    },
                },
                'required': ['acceleration', 'axis_world'],
            }
            factory.dynamics_registry.register(
                name,
                builder,
                defaults=defaults,
                schema=schema,
                dependencies=('environment_field',),
                description=name.replace('_', ' '),
                capabilities={
                    'state_layout': 'position_velocity',
                    'state_dimension': '2*d',
                    'input': 'live tracking_position_world plus optional physical_base',
                    'context_frame': 'same fixed world frame as target',
                    'deployable': True,
                    'runtime_validation': 'validated_contract',
                    'tangential_spatial_dimension': 3,
                },
            )
        factory.field_registry.register('linear_restoring', make_restoring_field)
        factory.field_registry.register('sinusoidal', make_sinusoidal_field)
        factory.field_registry.register('base_relative_guidance', BaseRelativeGuidance)
        factory.integrator_registry.register('rk4', make_rk4)

        return factory

    def create(
        self,
        target_config: Mapping[str, Any],
        environment_config: Mapping[str, Any],
        simulation_config: Mapping[str, Any],
    ) -> tuple[SimulationEngine, float]:
        """Build the simulation engine and timestep from YAML data."""
        target = target_config['target']
        environment = environment_config['environment']
        simulation = simulation_config['simulation']

        if simulation['backend'] != 'numerical':
            raise ValueError('Unsupported simulation backend.')

        fields = [
            self.field_registry.create(field_config)
            for field_config in environment.get('acceleration_fields', [])
            if field_config.get('enabled', True)
        ]

        environment_field = CompositeAccelerationField(fields)

        dynamics = self.dynamics_registry.create(
            target['dynamics'],
            environment_field=environment_field,
        )

        integration_config = simulation['integration']
        integrator = self.integrator_registry.create(
            {
                'type': integration_config['type'],
                'parameters': {
                    key: value
                    for key, value in integration_config.get('parameters', {}).items()
                    if key != 'dt'
                },
            }
        )

        dt = float(integration_config['parameters']['dt'])

        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError('Integration dt must be positive.')

        from robot_simulation.interfaces.dynamics import Dynamics

        if not isinstance(dynamics, Dynamics):
            raise TypeError('Registered dynamics must implement Dynamics')
        engine = SimulationEngine(
            dynamics=dynamics,
            integrator=integrator,
        )

        return engine, dt
