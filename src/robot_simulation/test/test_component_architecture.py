"""Dynamics defaults and extensions use the existing simulation factory."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.interfaces.dynamics import Dynamics


def test_dynamics_owner_defaults_and_strict_validation():
    factory = SimulationFactory.with_defaults()
    info = factory.dynamics_registry.describe('fight_flight')
    assert info['defaults']['damping'] == .5
    assert info['schema']['properties']['alpha']['unit'] == 's^-2'
    assert info['capabilities']['state_layout'] == 'position_velocity'
    for parameters in [{'alpha': float('nan')}, {'alpha': True}, {'unknown': 3.}]:
        with pytest.raises(ValueError):
            factory.dynamics_registry.create({'type': 'fight_flight', 'parameters': parameters})
    first = factory.dynamics_registry.describe('fight_flight')
    first['defaults']['alpha'] = 20.
    assert factory.dynamics_registry.describe('fight_flight')['defaults']['alpha'] == 1.


def test_test_only_dynamics_registers_and_steps_through_existing_factory():
    from robot_configuration.components import schema_for
    from robot_models.target.target_state import TargetState
    from robot_simulation.state.simulation_state import SimulationState

    class ConstantAcceleration(Dynamics):
        def __init__(self, acceleration, environment_field=None):
            self.acceleration = acceleration

        def derivative(self, state, control, time, context):
            return np.r_[state[3:], [self.acceleration, 0., 0.]]

    factory = SimulationFactory.with_defaults()
    factory.dynamics_registry.register(
        'test_only', ConstantAcceleration, defaults={'acceleration': 2.},
        schema=schema_for({'acceleration': 2.}, units={'acceleration': 'm/s^2'}),
        dependencies=('environment_field',), capabilities={'test_only': True, 'deployable': False})
    root = Path(__file__).parents[1] / 'config'
    environment = yaml.safe_load((root / 'environment.yaml').read_text())
    simulation = yaml.safe_load((root / 'simulation.yaml').read_text())
    target = {'target': {'dynamics': {'type': 'test_only', 'parameters': {'acceleration': 3.}}}}
    engine, dt = factory.create(target, environment, simulation)
    state = SimulationState(target=TargetState(np.zeros(3), np.zeros(3)),
                            tracking_position_world=np.zeros(3), time=0.)
    result = engine.step(state, dt)
    np.testing.assert_allclose(result.target.velocity, [3.*dt, 0., 0.])
    assert factory.dynamics_registry.describe('test_only')['capabilities']['test_only']
    target['target']['dynamics']['parameters']['unexpected'] = 1.
    with pytest.raises(ValueError, match='unexpected'):
        factory.create(target, environment, simulation)
