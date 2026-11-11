"""Full-state runtime integration must agree with independent physical references."""

from pathlib import Path

import numpy as np

import pytest

from robot_models.target.target_state import TargetState

from robot_simulation.dynamics.equation_model import EquationDynamics
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.gazebo.target_adapter import GazeboTargetAdapter

import yaml


def circular():
    """Declare the exact acceptance model without a deployment adapter or extra states."""
    return {
        'id': 'prescribed_motion',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [
            {'name': n, 'shape': [], 'unit': 'm', 'initial': v}
            for n, v in zip(['x', 'y', 'z'], [1.0, 0.0, 1.0])
        ],
        'parameters': {'omega': {'shape': [], 'unit': '1/s', 'value': 0.5}},
        'equations': {'x': '-omega*y', 'y': 'omega*x', 'z': '0'},
        'outputs': {'schema_version': 1, 'frame': 'world', 'position': '[x,y,z]'},
    }


def engine_for(model, fields=()):
    """Construct the actual owner engine with explicit environmental field selection."""
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/simulation.yaml').read_text())
    return SimulationFactory.with_defaults().create(
        {'target': {'dynamics': {'type': 'equation_state_space', 'parameters': {'model': model}}}},
        {'environment': {'acceleration_fields': list(fields), 'stochastic_disturbances': []}},
        config,
        reference_frame='world',
    )[0]


def initialize(engine, time=0.0):
    """Supply incompatible legacy p/v to detect incorrect mapped initialization."""
    return engine.initialize(
        TargetState(np.array([9.0, 8.0, 7.0]), np.array([6.0, 5.0, 4.0])), np.zeros(3), time
    )


def test_three_state_live_mapping_preserves_declared_initials_and_nonzero_epoch():
    """Compare every committed state to an independently evaluated analytical circle."""
    engine = engine_for(circular())
    state = initialize(engine, 12.375)
    np.testing.assert_array_equal(state.internal_state, [1, 0, 1])
    np.testing.assert_array_equal(state.target.position, [1, 0, 1])
    np.testing.assert_array_equal(state.target.velocity, [0, 0.5, 0])
    for i in range(1, 501):
        state = engine.step(state, 0.01)
        t = state.time - 12.375
        np.testing.assert_allclose(
            state.target.position, [np.cos(0.5 * t), np.sin(0.5 * t), 1], atol=2e-10, rtol=0
        )
        np.testing.assert_allclose(
            state.target.velocity,
            [-0.5 * np.sin(0.5 * t), 0.5 * np.cos(0.5 * t), 0],
            atol=2e-10,
            rtol=0,
        )
        assert state.internal_state.shape == (3,)
    np.testing.assert_array_equal(
        GazeboTargetAdapter().to_pose(state.target).position, state.target.position
    )
    assert abs(np.linalg.norm(state.target.position[:2]) - 1) < 1e-10
    reset = initialize(engine, 0.0)
    np.testing.assert_array_equal(reset.internal_state, [1, 0, 1])


def test_reordered_arbitrary_states_and_additional_vector_variables_are_retained():
    """Prove hidden vector variables evolve without relying on state order or names."""
    d = circular()
    d['state'] = [
        {'name': 'hidden', 'shape': [2], 'unit': '1', 'initial': [7, 8]},
        {'name': 'c', 'shape': [], 'unit': 'm', 'initial': 1},
        {'name': 'a', 'shape': [], 'unit': 'm', 'initial': 1},
        {'name': 'b', 'shape': [], 'unit': 'm', 'initial': 0},
    ]
    d['parameters']['rate'] = {'shape': [], 'unit': '1/s', 'value': 2}
    d['equations'] = {'hidden': '[rate,-rate]', 'c': '0', 'a': '-omega*b', 'b': 'omega*a'}
    d['outputs']['position'] = '[a,b,c]'
    e = engine_for(d)
    s = initialize(e)
    for _ in range(100):
        s = e.step(s, 0.01)
    np.testing.assert_allclose(s.internal_state[:2], [9, 6], atol=1e-12)
    np.testing.assert_allclose(s.target.position, [np.cos(0.5), np.sin(0.5), 1], atol=1e-10)
    assert len(s.internal_state) == 5
    snapshot = s.internal_state.copy()
    with pytest.raises(ValueError):
        s.internal_state[0] = 0
    np.testing.assert_array_equal(s.internal_state, snapshot)


def test_vector_states_and_explicit_time_mapping_use_same_simulation_time():
    """Verify explicit time and state derivative contributions at a nonzero epoch."""
    d = circular()
    d.update(
        state=[{'name': 'q', 'shape': [3], 'unit': 'm', 'initial': [1, 2, 3]}],
        parameters={'speed': {'shape': [3], 'unit': 'm/s', 'value': [0.1, 0.2, 0.3]}},
        equations={'q': 'speed'},
    )
    d['outputs']['position'] = 'q+speed*t'
    e = engine_for(d)
    s = initialize(e, 2.0)
    np.testing.assert_allclose(s.target.position, [1.2, 2.4, 3.6])
    s = e.step(s, 0.5)
    np.testing.assert_allclose(s.internal_state, [1.05, 2.1, 3.15])
    np.testing.assert_allclose(s.target.position, [1.3, 2.6, 3.9])
    np.testing.assert_allclose(s.target.velocity, [0.2, 0.4, 0.6])


def test_scaled_nonlinear_model_uses_same_engine_without_name_or_motion_branches():
    """Compare scaled Lorenz state and physical outputs against independent Python RK4."""
    d = circular()
    d['state'] = [{'name': n, 'shape': [], 'unit': '1', 'initial': 1.0} for n in ['a', 'b', 'c']]
    d['parameters'] = {
        n: {'shape': [], 'unit': u, 'value': v}
        for n, u, v in [
            ('sigma', '1', 10),
            ('rho', '1', 28),
            ('beta', '1', 8 / 3),
            ('rate', '1/s', 1),
            ('scale', 'm', 0.1),
        ]
    }
    d['equations'] = {'a': 'rate*sigma*(b-a)', 'b': 'rate*(a*(rho-c)-b)', 'c': 'rate*(a*b-beta*c)'}
    d['outputs']['position'] = 'scale*[a,b,c]'
    e = engine_for(d)
    s = initialize(e)

    # Independently evaluated RK4 reference, not PositionOutput or D3 evaluation.
    def f(x):
        a, b, c = x
        return np.array([10 * (b - a), a * (28 - c) - b, a * b - (8 / 3) * c])

    x = np.ones(3)
    for _ in range(50):
        h = 0.01
        k1 = f(x)
        k2 = f(x + h * k1 / 2)
        k3 = f(x + h * k2 / 2)
        k4 = f(x + h * k3)
        x = x + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        s = e.step(s, h)
    np.testing.assert_allclose(s.internal_state, x, atol=1e-12)
    np.testing.assert_allclose(s.target.position, 0.1 * x, atol=1e-12)
    np.testing.assert_allclose(s.target.velocity, 0.1 * f(x), atol=1e-12)
    d['outputs']['position'] = '[a,b,c]'
    with pytest.raises(ValueError, match='unit'):
        engine_for(d)


def test_active_fields_rejected_and_explicit_disabled_field_is_not_coupled():
    """Reject active coupling while accepting the owner's explicit disabled-field policy."""
    field = {
        'type': 'linear_restoring',
        'parameters': {'center': [0, 0, 1], 'gain_matrix': np.eye(3).tolist()},
    }
    with pytest.raises(ValueError, match='(?i)prescribed.*environment|environment.*prescribed'):
        engine_for(circular(), [field])
    with pytest.raises(ValueError, match='Prescribed mapped dynamics'):
        EquationDynamics(model=circular(), environment_field=object())
    field['enabled'] = False
    e = engine_for(circular(), [field])
    s = initialize(e)
    s = e.step(s, 0.1)
    np.testing.assert_allclose(
        s.target.velocity, [-0.5 * np.sin(0.05), 0.5 * np.cos(0.05), 0], atol=2e-8
    )


def test_mapping_domain_and_unbound_live_inputs_fail_closed():
    """Reject singular initial outputs and unsupported live bindings."""
    d = circular()
    d['parameters']['length'] = {'shape': [], 'unit': 'm', 'value': 1}
    d['outputs']['position'] = 'length*[x,y,z]/y'
    with pytest.raises(ValueError):
        initialize(engine_for(d))
    d = circular()
    d['context'] = {'unknown': {'shape': [], 'unit': 'm', 'frame': 'world'}}
    with pytest.raises(ValueError, match='context'):
        engine_for(d)
    d = circular()
    d['control'] = [{'name': 'u', 'shape': [], 'unit': 'm/s', 'frame': 'world'}]
    with pytest.raises(ValueError, match='control'):
        engine_for(d)
    d = circular()
    d['outputs']['frame'] = 'map'
    with pytest.raises(ValueError, match='frame'):
        engine_for(d)
