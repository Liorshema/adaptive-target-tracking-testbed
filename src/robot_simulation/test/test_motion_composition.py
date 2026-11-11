"""Composition is a validated transformation, not another dynamics executor."""

from copy import deepcopy
import numpy as np
import pytest

from robot_simulation.dynamics.motion_composition import compile_definition
from robot_simulation.dynamics.equation_model import EquationDynamics


def scalar(value, unit):
    return {'value': value, 'shape': [], 'unit': unit}


def definition():
    return {
        'schema_version': 1,
        'type': 'cartesian_composition',
        'id': 'spiral_avoidance',
        'frame': 'world',
        'state': [
            {'name': 'p', 'shape': [3], 'unit': 'm', 'initial': [1.0, 0.0, 1.0]},
            {'name': 'v', 'shape': [3], 'unit': 'm/s', 'initial': [0.1, 0.2, 0.3]},
        ],
        'base': {
            'id': 'spiral',
            'frame': 'world',
            'output': {'shape': [3], 'unit': 'm/s^2'},
            'parameters': {
                'k': scalar(0.2, '1/s^2'),
                'c': scalar(0.1, '1/s'),
                'rate': scalar(0.4, '1/s'),
                'axis': {'value': [0.0, 0.0, 1.0], 'shape': [3], 'unit': '1'},
            },
            'expression': '-k*p-c*v+rate*cross(axis,v)',
        },
        'interactions': [
            {
                'id': 'avoidance',
                'frame': 'world',
                'output': {'shape': [3], 'unit': 'm/s^2'},
                'parameters': {
                    'a': scalar(0.15, 'm/s^2'),
                    'r0': scalar(1.0, 'm'),
                    'L': scalar(0.15, 'm'),
                    'epsilon': scalar(1e-6, 'm'),
                },
                'context': {
                    'tracking_position_world': {'shape': [3], 'unit': 'm', 'frame': 'world'}
                },
                'intermediates': {'delta': 'p-tracking_position_world', 'r': 'norm(delta)'},
                'expression': 'a*tanh((r-r0)/L)*delta/sqrt(dot(delta,delta)+epsilon**2)',
            }
        ],
    }


@pytest.mark.parametrize(
    'interaction,environment', [(False, False), (True, False), (False, True), (True, True)]
)
def test_composed_derivative_matches_independent_contributions(interaction, environment):
    source = definition()
    source['interactions'][0]['enabled'] = interaction
    compiled = compile_definition(source)

    class Field:
        calls = 0

        def acceleration(self, p, t):
            self.calls += 1
            return np.array([0.1, -0.2, t * 0.03])

    field = Field()
    d = EquationDynamics(model=compiled['model'], environment_field=field if environment else None)
    x = np.array([1.0, 0.2, 0.7, 0.1, -0.3, 0.2])
    tracking = np.array([0.1, 0.2, 0.3])
    expected = -0.2 * x[:3] - 0.1 * x[3:] + 0.4 * np.cross([0, 0, 1], x[3:])
    if interaction:
        delta = x[:3] - tracking
        r = np.linalg.norm(delta)
        expected += 0.15 * np.tanh((r - 1.0) / 0.15) * delta / np.sqrt(r * r + 1e-12)
    if environment:
        expected += np.array([0.1, -0.2, 0.021])
    actual = d.derivative(x, [], 0.7, {'tracking_position_world': tracking} if interaction else {})
    np.testing.assert_allclose(actual, np.r_[x[3:], expected], atol=1e-14)
    assert field.calls == int(environment)


def test_disable_remove_and_parameter_namespaces_are_independent_and_deterministic():
    a = definition()
    a['interactions'][0]['enabled'] = False
    b = deepcopy(a)
    b['interactions'] = []
    x = [1.0, 2.0, 3.0, 0.2, 0.3, 0.4]
    da = EquationDynamics(model=compile_definition(a)['model']).derivative(x, [], 0.0, {})
    db = EquationDynamics(model=compile_definition(b)['model']).derivative(x, [], 0.0, {})
    np.testing.assert_array_equal(da, db)
    a = definition()
    a['interactions'][0]['parameters']['k'] = scalar(99.0, '1/s^2')
    compiled = compile_definition(a)
    mapping = compiled['symbol_map']
    assert mapping['spiral']['parameters']['k'] != mapping['avoidance']['parameters']['k']
    assert compile_definition(a) == compiled
    assert a['base']['parameters']['k']['value'] == 0.2


@pytest.mark.parametrize(
    'mutation',
    [
        lambda x: x.update(environment={'type': 'wind'}),
        lambda x: x.update(stochastic=True),
        lambda x: x['base']['output'].update(unit='m/s'),
        lambda x: x['base']['output'].update(shape=[2]),
        lambda x: x['base'].update(frame='body'),
        lambda x: x['interactions'].append(deepcopy(x['interactions'][0])),
        lambda x: x['base'].update(expression='c1_p0*p'),
        lambda x: x['base'].update(expression="__import__('os')"),
        lambda x: x['base'].update(expression='tracking_position_world'),
        lambda x: x['base']['parameters'].update(p=scalar(1.0, '1/s')),
        lambda x: x['base'].update(enabled=1),
        lambda x: x['interactions'][0].update(expression='unknown'),
    ],
)
def test_invalid_combination_and_cross_namespace_injection_are_rejected(mutation):
    x = definition()
    mutation(x)
    with pytest.raises(ValueError):
        compile_definition(x)


def test_general_state_space_passthrough_preserves_all_states():
    model = {
        'id': 'general',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [{'name': 'q', 'unit': '1', 'shape': [8], 'initial': [0.0] * 8}],
        'parameters': {},
        'equations': {'q': '[0,0,0,0,0,0,0,0] / tau'},
    }
    model['parameters']['tau'] = scalar(1.0, 's')
    result = compile_definition(model)
    assert result['model'] == model and result['symbol_map'] == {}
    d = EquationDynamics(model=result['model'])
    assert d.state_dimension == 8 and not d.capabilities['deployable']
