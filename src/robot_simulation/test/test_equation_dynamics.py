"""Independent mathematical and adversarial checks for declarative Dynamics."""

from copy import deepcopy
import math

import numpy as np
import pytest

from robot_simulation.dynamics.equation_model import EquationDynamics
from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.dynamics.target.fight_flight import (
    FightFlightDynamics,
    SaturatedFightFlightParameters,
)


def variable(name, unit, initial, shape=None):
    return {
        'name': name,
        'unit': unit,
        'shape': [] if shape is None else shape,
        'initial': initial,
    }


def parameter(value, unit, **bounds):
    return {
        'value': value,
        'unit': unit,
        'shape': [len(value)] if isinstance(value, list) else [],
        **bounds,
    }


def model():
    return {
        'id': 'test_motion',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [
            variable('p', 'm', [1.0, 0.0, 0.0], [3]),
            variable('v', 'm/s', [0.2, 0.0, 0.0], [3]),
        ],
        'parameters': {},
        'equations': {'p': 'v', 'v': '[0, 0, 0]'},
        'deployment': {'adapter': 'cartesian_target', 'position': 'p', 'velocity': 'v'},
    }


def integrate(dynamics, steps=100, dt=0.01, context=None):
    x = dynamics.initial_state.copy()
    for i in range(steps):
        x = RK4Integrator().step(dynamics, x, np.empty(0), i * dt, dt, context or {})
    return x


def test_constant_velocity_analytic_and_detached_definition():
    definition = model()
    d = EquationDynamics(model=definition)
    definition['equations']['p'] = '[0,0,0]'
    np.testing.assert_allclose(integrate(d), [1.2, 0, 0, 0.2, 0, 0], atol=1e-14)
    assert d.state_dimension == 6
    d.validate_deployment('world')


def test_spring_damper_against_independent_analytic_solution():
    m = model()
    m['state'][1]['initial'] = [0.0, 0.0, 0.0]
    m['parameters'] = {
        'k': parameter(2.0, 's^-2', minimum=0.0),
        'c': parameter(0.5, '1/s', minimum=0.0),
    }
    m['equations']['v'] = '-k*p-c*v'
    actual = integrate(EquationDynamics(model=m))
    w = math.sqrt(2.0 - 0.25**2)
    expected = [
        math.exp(-0.25) * (math.cos(w) + 0.25 / w * math.sin(w)),
        0,
        0,
        -2.0 / w * math.exp(-0.25) * math.sin(w),
        0,
        0,
    ]
    np.testing.assert_allclose(actual, expected, atol=1e-9)
    changed = deepcopy(m)
    changed['parameters']['k']['value'] = 3.0
    assert not np.allclose(actual, integrate(EquationDynamics(model=changed)))


def test_eight_states_order_and_rk4_without_false_deployment():
    m = model()
    m.pop('deployment')
    m['state'] += [variable('theta', 'rad', 0.3), variable('omega', 'rad/s', 0.7)]
    m['parameters']['gamma'] = parameter(0.1, '1/s', minimum=0.0)
    # Deliberately reverse equation mapping order: only state list determines layout.
    m['equations'] = {'omega': '-gamma*omega', 'theta': 'omega', 'v': '[0,0,0]', 'p': 'v'}
    d = EquationDynamics(model=m)
    assert d.state_dimension == 8
    assert d.capabilities['mathematically_valid']
    assert not d.capabilities['deployable']
    np.testing.assert_allclose(
        d.derivative(d.initial_state, [], 0.0, {}), [0.2, 0, 0, 0, 0, 0, 0.7, -0.07]
    )
    expected = [0.3 + 7 * (1 - math.exp(-0.1)), 0.7 * math.exp(-0.1)]
    np.testing.assert_allclose(integrate(d)[6:], expected, atol=1e-12)
    with pytest.raises(ValueError, match='deploy'):
        d.validate_deployment('world')
    m['deployment'] = model()['deployment']
    with pytest.raises(ValueError, match='six|6|deploy'):
        EquationDynamics(model=m).validate_deployment('world')


def saturated():
    m = model()
    m['parameters'] = {
        'a_max': parameter(0.15, 'm/s^2', exclusiveMinimum=0.0),
        'transition_length': parameter(0.15, 'm', exclusiveMinimum=0.0),
        'preferred_distance': parameter(1.0, 'm', exclusiveMinimum=0.0),
        'damping': parameter(0.5, '1/s', minimum=0.0),
        'epsilon': parameter(1e-6, 'm', exclusiveMinimum=0.0),
    }
    m['context'] = {'tracking_position_world': {'unit': 'm', 'shape': [3], 'frame': 'world'}}
    # Forward reference proves topological compilation, not mapping-order execution.
    m['intermediates'] = {
        'accel': 'a_max*tanh((distance-preferred_distance)/transition_length)*direction-damping*v',
        'direction': 'delta/sqrt(dot(delta,delta)+epsilon**2)',
        'distance': 'norm(delta)',
        'delta': 'p-tracking_position_world',
    }
    m['equations']['v'] = 'accel'
    return m


class Field:
    def acceleration(self, position, time):
        return np.array([0.1, -0.2, 0.03 * math.sin(time)])


@pytest.mark.parametrize(
    'position', [[0.0, 0.0, 0.0], [1e-10, 0.0, 0.0], [1.0, 0.2, 0.7], [-2.0, 1.0, 0.3]]
)
def test_saturated_derivatives_and_regularization_match_trusted_python(position):
    m = saturated()
    p = SaturatedFightFlightParameters(0.15, 0.15, 1.0, 0.5, 1e-6)
    old = FightFlightDynamics(p, environment_field=Field())
    new = EquationDynamics(model=m, environment_field=Field())
    state = np.r_[position, [0.1, -0.3, 0.2]]
    context = {'tracking_position_world': np.zeros(3)}
    np.testing.assert_allclose(
        new.derivative(state, [], 0.7, context),
        old.derivative(state, [], 0.7, context),
        atol=1e-14,
    )
    state_old = state.copy()
    for i in range(50):
        state_old = RK4Integrator().step(old, state_old, [], i * 0.002, 0.002, context)
        state = RK4Integrator().step(new, state, [], i * 0.002, 0.002, context)
    np.testing.assert_allclose(state, state_old, atol=1e-13)


def test_control_time_vectors_dot_cross_index_and_functions():
    m = model()
    m.pop('deployment')
    m['parameters'] = {
        'rate': parameter(0.4, '1/s'),
        'axis': parameter([0.0, 0.0, 1.0], '1'),
        'period': parameter(2.0, 's'),
    }
    m['control'] = [{'name': 'a', 'shape': [3], 'unit': 'm/s^2', 'frame': 'world'}]
    m['equations']['v'] = 'rate*cross(axis,v) + a*cos(t/period)'
    d = EquationDynamics(model=m)
    np.testing.assert_allclose(
        d.derivative(d.initial_state, [1, 2, 3], 0.0, {}), [0.2, 0, 0, 1, 2.08, 3]
    )
    m['equations']['v'] = '[a[0],a[1],a[2]] * exp(-t/period) * sin(pi/2)'
    np.testing.assert_allclose(
        EquationDynamics(model=m).derivative(d.initial_state, [1, 2, 3], 2.0, {})[3:],
        np.exp(-1) * np.array([1, 2, 3]),
    )


@pytest.mark.parametrize(
    'expression',
    [
        "__import__('os').system('true')",
        'p.__class__',
        'open(1)',
        '[x for x in p]',
        '(lambda: 1)()',
        'p[0:2]',
        'p[v[0]]',
        'p[-1]',
        'sum(p)',
        'not p',
        'p if t else v',
        'missing',
        'sqrt(-1)',
        '1/0',
        '2**99999',
        'True',
        "'text'",
    ],
)
def test_expression_security_and_constant_domains(expression):
    m = model()
    m['equations']['v'] = expression
    with pytest.raises(ValueError):
        EquationDynamics(model=m)


@pytest.mark.parametrize(
    'expression',
    [
        'p',
        'v+p',
        'sin(p)',
        'exp(t)',
        'sqrt(v)',
        'dot(p,v)',
        '[v[0],v[1]]',
        'cross(p,[1,2])',
        'p**t',
        'p+p[0]',
    ],
)
def test_wrong_units_shapes_and_powers_reject_before_execution(expression):
    m = model()
    m['equations']['v'] = expression
    with pytest.raises(ValueError):
        EquationDynamics(model=m)


@pytest.mark.parametrize(
    'mutation',
    [
        lambda m: m.update(unknown=True),
        lambda m: m.update(schema_version=True),
        lambda m: m['state'].append(deepcopy(m['state'][0])),
        lambda m: m['equations'].update(extra='v'),
        lambda m: m['equations'].pop('p'),
        lambda m: m['state'][0].update(initial=[0, 0]),
        lambda m: m['state'][0].update(unit='cm'),
        lambda m: m['state'][0].update(shape=[0]),
        lambda m: m['state'][0].update(initial=[0, np.nan, 0]),
        lambda m: m['parameters'].update(k=parameter(-1.0, '1/s', minimum=0.0)),
        lambda m: m['parameters'].update(k=parameter(np.inf, '1/s')),
        lambda m: m['parameters'].update(k=parameter(1.0, '1/s', typo=2.0)),
        lambda m: m.update(intermediates={'a': 'b', 'b': 'a'}),
        lambda m: m.update(context={'r': {'shape': [3], 'unit': 'm', 'frame': 'body'}}),
        lambda m: m['equations'].update(v='+'.join(['v'] * 400)),
    ],
)
def test_schema_constraints_dependencies_and_cost_bounds(mutation):
    m = model()
    mutation(m)
    with pytest.raises(ValueError):
        EquationDynamics(model=m)


@pytest.mark.parametrize(
    'state,control,time,context',
    [
        ([0] * 5, [], 0.0, {}),
        ([0] * 6, [1], 0.0, {}),
        ([0] * 6, [], True, {}),
        ([0] * 5 + [np.inf], [], 0.0, {}),
        ([0j] * 6, [], 0.0, {}),
        ([0] * 6, [], 0.0, None),
        ([0] * 6, [], 0.0, {'frame_id': 'body'}),
    ],
)
def test_runtime_state_inputs_frames_and_time_are_validated(state, control, time, context):
    with pytest.raises(ValueError):
        EquationDynamics(model=model()).derivative(state, control, time, context)


def test_required_context_domain_errors_and_no_silent_clipping():
    m = saturated()
    d = EquationDynamics(model=m)
    for context in (
        {},
        {'tracking_position_world': [1, 2]},
        {'tracking_position_world': [0, np.inf, 0]},
    ):
        with pytest.raises(ValueError):
            d.derivative(d.initial_state, [], 0.0, context)
    m['intermediates']['direction'] = 'delta/norm(delta)'
    singular = EquationDynamics(model=m)
    with pytest.raises(ValueError):
        singular.derivative(np.zeros(6), [], 0.0, {'tracking_position_world': np.zeros(3)})


def test_circular_velocity_has_independent_analytic_trajectory():
    m = model()
    m['parameters'] = {'rate': parameter(0.4, '1/s'), 'axis': parameter([0.0, 0.0, 1.0], '1')}
    m['equations']['v'] = 'rate*cross(axis,v)'
    actual = integrate(EquationDynamics(model=m))
    np.testing.assert_allclose(
        actual,
        [
            1.0 + 0.2 * math.sin(0.4) / 0.4,
            0.2 * (1.0 - math.cos(0.4)) / 0.4,
            0.0,
            0.2 * math.cos(0.4),
            0.2 * math.sin(0.4),
            0.0,
        ],
        atol=1e-12,
    )


@pytest.mark.parametrize(
    'expression,state',
    [
        ('sqrt(q)*rate', [-1.0]),
        ('q**0.5*rate', [-1.0]),
        ('rate/q', [0.0]),
        ('exp(q)*rate', [1000.0]),
    ],
)
def test_runtime_domain_failures_are_explicit(expression, state):
    m = {
        'id': 'domain_test',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [variable('q', '1', 1.0)],
        'parameters': {'rate': parameter(1.0, '1/s')},
        'equations': {'q': expression},
    }
    d = EquationDynamics(model=m)
    with pytest.raises(ValueError):
        d.derivative(state, [], 0.0, {})


def test_cartesian_pipeline_passes_actual_tracking_context_and_contextual_field_once():
    from robot_models.target.target_state import TargetState
    from robot_simulation.engine.simulation_engine import SimulationEngine
    from robot_simulation.state.simulation_state import SimulationState
    from robot_simulation.state.physical_context import PhysicalBaseContext

    class ContextField:
        def acceleration(self, p, t):
            raise AssertionError('Use the existing physical-context contract')

        def acceleration_with_context(self, p, t, base):
            return 0.1 * base.position_at(t, 'world')

    context = {
        'tracking_position_world': np.array([0.3, 0.2, 0.1]),
        'physical_base': PhysicalBaseContext(
            np.array([0.1, 0.2, 0.3]), 0.0, 'world', 'test measured source', 1.0
        ),
    }
    d = EquationDynamics(model=saturated(), environment_field=ContextField())
    old = FightFlightDynamics(
        SaturatedFightFlightParameters(0.15, 0.15, 1.0, 0.5, 1e-6),
        environment_field=ContextField(),
    )
    s = SimulationState(
        TargetState(np.array([1.0, 0.2, 1.0]), np.array([0.1, 0.2, 0.3])),
        context['tracking_position_world'],
        0.0,
        context['physical_base'],
    )
    actual = SimulationEngine(d, RK4Integrator()).step(s, 0.01)
    expected = SimulationEngine(old, RK4Integrator()).step(s, 0.01)
    np.testing.assert_allclose(actual.target.position, expected.target.position, atol=1e-14)
    np.testing.assert_allclose(actual.target.velocity, expected.target.velocity, atol=1e-14)


def test_dimensionless_fractional_power_has_real_domain_without_unit_ambiguity():
    m = {
        'id': 'fractional',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [variable('q', '1', 1.0)],
        'parameters': {'rate': parameter(1.0, '1/s')},
        'equations': {'q': 'q**0.3*rate'},
    }
    d = EquationDynamics(model=m)
    np.testing.assert_allclose(d.derivative([2.0], [], 0.0, {}), [2.0**0.3])
    with pytest.raises(ValueError):
        d.derivative([-2.0], [], 0.0, {})


def test_generic_integrator_rejects_nonfinite_final_combination():
    m = {
        'id': 'large_rate',
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': 'world',
        'state': [variable('q', '1', 0.0)],
        'parameters': {'rate': parameter(1e308, '1/s')},
        'equations': {'q': 'rate'},
    }
    d = EquationDynamics(model=m)
    # Each derivative and RK4 stage is finite; the final weighted sum can overflow.
    with np.errstate(over='ignore'):
        with pytest.raises(ValueError, match='finite'):
            RK4Integrator().step(d, d.initial_state, [], 0.0, 1e-6, {})
