"""Owner registration, atomic model definitions and six-state deployment boundaries."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_models.target.target_state import TargetState
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.state.simulation_state import SimulationState
from robot_simulation.dynamics.equation_model import EquationDynamics

ROOT = Path(__file__).parents[1] / 'config'


def factory_inputs(model=None):
    target = {
        'target': {
            'dynamics': {
                'type': 'equation_state_space',
                'parameters': {} if model is None else {'model': model},
            }
        }
    }
    return (
        target,
        {'environment': {'acceleration_fields': [], 'stochastic_disturbances': []}},
        yaml.safe_load((ROOT / 'simulation.yaml').read_text()),
    )


def test_owner_default_cv_and_explicit_live_initial_override():
    factory = SimulationFactory.with_defaults()
    default = factory.dynamics_registry.describe('equation_state_space')['defaults']['model']
    engine, dt = factory.create(*factory_inputs())
    assert isinstance(engine.dynamics, EquationDynamics)
    # Explicit existing experiment p/v initial overrides the standalone model default.
    x = np.array([2.0, 3.0, 4.0, 0.1, 0.2, 0.3])
    state = SimulationState(TargetState(x[:3], x[3:]), np.zeros(3), 0.0)
    actual = engine.step(state, dt)
    np.testing.assert_allclose(actual.target.position, x[:3] + dt * x[3:])
    np.testing.assert_allclose(actual.target.velocity, x[3:])
    assert default['state'][0]['initial'] == [0.0, 0.0, 1.0]


def test_owner_atomic_replacement_no_stale_equations_or_default_parameters():
    registry = SimulationFactory.with_defaults().dynamics_registry
    m = deepcopy(registry.describe('equation_state_space')['defaults']['model'])
    m.update(
        id='scalar_decay',
        state=[{'name': 'temperature', 'unit': '1', 'shape': [], 'initial': 2.0}],
        parameters={'decay': {'unit': '1/s', 'shape': [], 'value': 0.2}},
        equations={'temperature': '-decay*temperature'},
    )
    m.pop('deployment')
    values = registry.validate('equation_state_space', {'model': m})
    assert values == {'model': m}
    d = registry.create({'type': 'equation_state_space', 'parameters': values})
    np.testing.assert_allclose(d.derivative([2.0], [], 0.0, {}), [-0.4])
    for mutation in (
        lambda n: n['equations'].update(temperature='unknown'),
        lambda n: n.update(bogus=1),
    ):
        bad = deepcopy(m)
        mutation(bad)
        with pytest.raises(ValueError):
            registry.validate('equation_state_space', {'model': bad})
    with pytest.raises(ValueError):
        registry.validate('equation_state_space', {}, use_defaults=False)


def test_factory_rejects_arbitrary_state_or_unprovided_live_inputs_before_start():
    default = SimulationFactory.with_defaults().dynamics_registry.describe('equation_state_space')[
        'defaults'
    ]['model']
    variants = []
    eight = deepcopy(default)
    eight['state'] += [
        {'name': 'theta', 'unit': 'rad', 'shape': [], 'initial': 0.0},
        {'name': 'omega', 'unit': 'rad/s', 'shape': [], 'initial': 1.0},
    ]
    eight['equations'].update(theta='omega', omega='0')
    eight.pop('deployment')
    variants.append(eight)
    for context in ({'unsupported': {'unit': 'm', 'shape': [3], 'frame': 'world'}},):
        m = deepcopy(default)
        m['context'] = context
        variants.append(m)
    for m in variants:
        with pytest.raises(ValueError, match='deploy'):
            SimulationFactory.with_defaults().create(*factory_inputs(m))
    with pytest.raises(ValueError, match='frame'):
        SimulationFactory.with_defaults().create(*factory_inputs(default), reference_frame='body')


def test_constant_zeros_do_not_hide_nonzero_unit_errors():
    m = SimulationFactory.with_defaults().dynamics_registry.describe('equation_state_space')[
        'defaults'
    ]['model']
    m['equations']['v'] = '[0**0,0,0]'
    with pytest.raises(ValueError, match='unit'):
        EquationDynamics(model=m)


def test_external_legacy_owner_file_does_not_require_new_equation_defaults(tmp_path):
    owner = yaml.safe_load((ROOT / 'dynamics.yaml').read_text())
    owner['dynamics'].pop('equation_state_space')
    path = tmp_path / 'legacy_defaults.yaml'
    path.write_text(yaml.safe_dump(owner))
    registry = SimulationFactory.with_defaults(path).dynamics_registry
    assert registry.names() == ('fight_flight', 'saturated_fight_flight')
    d = registry.create({'type': 'fight_flight'})
    assert np.isfinite(
        d.derivative(np.ones(6), [], 0.0, {'tracking_position_world': np.zeros(3)})
    ).all()
