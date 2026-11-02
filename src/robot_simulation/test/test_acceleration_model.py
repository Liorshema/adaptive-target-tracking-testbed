"""Mathematical contracts for the optional Cartesian acceleration adapter."""

import importlib
from pathlib import Path

import numpy as np

import pytest

from robot_models.target.target_state import TargetState

from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.state.simulation_state import SimulationState

import yaml

SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'stiffness': {'type': 'number', 'minimum': 0.0, 'unit': 's^-2'},
        'damping': {'type': 'number', 'minimum': 0.0, 'unit': 's^-1'},
    },
    'required': ['stiffness', 'damping'],
}
PARAMETERS = {'stiffness': 2.0, 'damping': 0.5}


def declaration(callback=None, **metadata):
    """Keep test-only mathematical models out of the production namespace."""
    acceleration_model = importlib.import_module(
        'robot_simulation.dynamics.acceleration_model'
    ).acceleration_model
    if callback is None:

        def callback(position, velocity, time, parameters, context):
            return -parameters['stiffness'] * position - parameters['damping'] * velocity

    return acceleration_model('test_spring_damper', dimension=3, schema=SCHEMA, **metadata)(
        callback
    )


def build(callback=None, environment_field=None, **metadata):
    """Construct the test-only model from explicit owner-equivalent parameters."""
    return declaration(callback, **metadata).builder(
        environment_field=environment_field, **PARAMETERS
    )


def test_spring_damper_derivative_uses_cartesian_layout():
    """The acceleration law follows position and velocity without transposition."""
    dynamics = build()
    result = dynamics.derivative([1.0, -2.0, 3.0, 0.2, -0.4, 0.6], [], 1.5, {})
    np.testing.assert_allclose(result, [0.2, -0.4, 0.6, -2.1, 4.2, -6.3])


def test_spring_damper_integration_matches_independent_damped_oscillator():
    """Unchanged RK4 integration agrees with the analytic damped oscillator."""
    # p'' + 0.5 p' + 2 p = 0; p(0)=1, p'(0)=0.
    dynamics = build()
    state = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    integrator = RK4Integrator()
    for step in range(100):
        state = integrator.step(dynamics, state, np.array([]), step * 0.01, 0.01, {})
    omega = np.sqrt(2.0 - 0.25**2)
    expected_p = np.exp(-0.25) * (np.cos(omega) + 0.25 / omega * np.sin(omega))
    expected_v = -2.0 / omega * np.exp(-0.25) * np.sin(omega)
    np.testing.assert_allclose(state, [expected_p, 0.0, 0.0, expected_v, 0.0, 0.0], atol=1e-9)


def test_callback_receives_unchanged_stage_time_and_declared_context():
    """The callback receives the runtime time and live world context."""

    def acceleration(position, velocity, time, parameters, context):
        return context['tracking_position_world'] + time * np.ones(3)

    dynamics = build(acceleration, required_context=('tracking_position_world',))
    result = dynamics.derivative(
        np.zeros(6), [], 2.5, {'tracking_position_world': np.array([1.0, 2.0, 3.0])}
    )
    np.testing.assert_allclose(result, [0.0, 0.0, 0.0, 3.5, 4.5, 5.5])


class ConstantField:
    """Ordinary position/time field with a hand-computable contribution."""

    def acceleration(self, position, time):
        """Supply an independent external acceleration."""
        return np.array([1.0, 2.0, time])


class ContextField:
    """Field whose explicit contextual method must be selected."""

    def acceleration(self, position, time):
        """Reject accidental use of the context-free dispatch branch."""
        raise AssertionError('Contextual dispatch must use the explicit context method')

    def acceleration_with_context(self, position, time, context):
        """Add the physical-source context and current simulation time."""
        return context + np.array([time, 0.0, 0.0])


@pytest.mark.parametrize(
    'field,context,expected',
    [
        (ConstantField(), {}, [1.0, 2.0, 3.0]),
        (ContextField(), {'physical_base': np.array([1.0, 2.0, 3.0])}, [4.0, 2.0, 3.0]),
    ],
)
def test_environment_acceleration_is_added_exactly_once(field, context, expected):
    """Both field interfaces add one contribution after model-local acceleration."""
    result = build(environment_field=field).derivative(np.ones(6), [], 3.0, context)
    np.testing.assert_allclose(result[3:], np.array(expected) - 2.5)


@pytest.mark.parametrize(
    'state', [np.zeros(4), np.zeros((2, 3)), [0.0] * 5, [0.0, 0.0, 0.0, 0.0, 0.0, np.nan]]
)
def test_rejects_wrong_dimension_and_nonfinite_state(state):
    """Invalid Cartesian states are rejected before the callback."""
    with pytest.raises(ValueError, match='state'):
        build().derivative(state, [], 0.0, {})


@pytest.mark.parametrize('time', [np.nan, np.inf, True, '1', np.array([1.0])])
def test_rejects_invalid_time(time):
    """Only finite real scalar simulation times reach the callback."""
    with pytest.raises(ValueError, match='time'):
        build().derivative(np.zeros(6), [], time, {})


@pytest.mark.parametrize('output', [np.zeros(2), np.zeros((1, 3)), [0.0, np.inf, 0.0]])
def test_rejects_invalid_model_acceleration(output):
    """Wrong-shaped or nonfinite mathematical outputs cannot become derivatives."""

    def acceleration(*args):
        return output

    with pytest.raises(ValueError, match='acceleration'):
        build(acceleration).derivative(np.zeros(6), [], 0.0, {})


@pytest.mark.parametrize('output', [np.zeros(2), [0.0, np.nan, 0.0]])
def test_rejects_invalid_environment_acceleration(output):
    """The environment cannot introduce invalid acceleration vectors."""

    class InvalidField:
        def acceleration(self, position, time):
            return output

    with pytest.raises(ValueError, match='acceleration'):
        build(environment_field=InvalidField()).derivative(np.zeros(6), [], 0.0, {})


def test_rejects_missing_context_and_mismatched_frame():
    """Missing declared inputs and explicit frame mismatches fail early."""
    dynamics = build(required_context=('tracking_position_world',))
    with pytest.raises(ValueError, match='tracking_position_world'):
        dynamics.derivative(np.zeros(6), [], 0.0, {})
    with pytest.raises(ValueError, match='frame'):
        build().derivative(np.zeros(6), [], 0.0, {'frame_id': 'moving_body'})
    with pytest.raises(ValueError, match='context'):
        build().derivative(np.zeros(6), [], 0.0, None)


@pytest.mark.parametrize('tracking', [np.zeros(2), [0.0, np.nan, 0.0]])
def test_rejects_invalid_known_world_position_context(tracking):
    """Runtime tracking vectors respect the declared spatial dimension."""
    with pytest.raises(ValueError, match='tracking_position_world'):
        build().derivative(np.zeros(6), [], 0.0, {'tracking_position_world': tracking})


@pytest.mark.parametrize(
    'parameters',
    [
        {'stiffness': True, 'damping': 0.5},
        {'stiffness': 1.0, 'damping': -0.1},
        {'stiffness': 1.0, 'damping': 0.5, 'hidden': 1},
        {'stiffness': 1.0},
    ],
)
def test_direct_builder_preserves_strict_schema_validation(parameters):
    """Even direct construction rejects unknown, missing and invalid parameters."""
    with pytest.raises(ValueError):
        declaration().builder(**parameters)


def test_callback_cannot_mutate_state_or_parameter_mapping():
    """Mathematical callbacks cannot overwrite shared states or parameter keys."""

    def acceleration(position, velocity, time, parameters, context):
        with pytest.raises(ValueError):
            position[0] = 100.0
        with pytest.raises(ValueError):
            velocity[0] = 100.0
        with pytest.raises(TypeError):
            parameters['stiffness'] = 100.0
        return np.zeros(3)

    state = np.ones(6)
    build(acceleration).derivative(state, [], 0.0, {})
    np.testing.assert_array_equal(state, np.ones(6))


def test_declaration_has_no_duplicate_defaults_and_declares_runtime_contract():
    """Registration describes the mathematical contract without another default source."""
    model = declaration()
    assert model.dependencies == ('environment_field',)
    assert model.capabilities['state_layout'] == 'position_velocity'
    assert model.capabilities['spatial_dimension'] == 3
    assert model.capabilities['runtime_validation'] == 'unit_tested_extension'
    assert not hasattr(model, 'defaults')


def test_factory_discovers_test_only_declaration_and_uses_owner_yaml(tmp_path, monkeypatch):
    """Trusted discovery constructs models with owner defaults without inventory leaks."""
    import robot_simulation.factory.simulation_factory as factory_module
    from robot_configuration.components import ComponentRegistry
    from robot_configuration.model_registration import discover_models

    namespace = tmp_path / 'test_dynamics_plugins'
    namespace.mkdir()
    (namespace / '__init__.py').write_text('')
    (namespace / 'spring.py').write_text(
        'from robot_simulation.dynamics.acceleration_model import acceleration_model\n'
        f'SCHEMA = {SCHEMA!r}\n'
        '@acceleration_model("test_spring_damper", dimension=3, schema=SCHEMA)\n'
        'def spring(position, velocity, time, parameters, context):\n'
        '    return -parameters["stiffness"]*position-parameters["damping"]*velocity\n'
        'COMPONENTS = (spring,)\n'
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    monkeypatch.setattr(factory_module, 'DYNAMICS_MODEL_NAMESPACE', 'test_dynamics_plugins')
    source_config = Path(__file__).resolve().parents[1] / 'config/dynamics.yaml'
    data = yaml.safe_load(source_config.read_text())
    data['dynamics']['test_spring_damper'] = {'parameters': PARAMETERS}
    config = tmp_path / 'dynamics.yaml'
    config.write_text(yaml.safe_dump(data))
    factory = SimulationFactory.with_defaults(config)
    dynamics = factory.dynamics_registry.create({'type': 'test_spring_damper'})
    np.testing.assert_allclose(
        dynamics.derivative(np.ones(6), [], 0.0, {}), [1.0, 1.0, 1.0, -2.5, -2.5, -2.5]
    )
    engine, dt = factory.create(
        {'target': {'dynamics': {'type': 'test_spring_damper'}}},
        {'environment': {'acceleration_fields': []}},
        {
            'simulation': {
                'backend': 'numerical',
                'integration': {'type': 'rk4', 'parameters': {'dt': 0.01}},
            }
        },
    )
    state = SimulationState(TargetState(np.zeros(3), np.ones(3)), np.zeros(3), 0.0)
    assert engine.step(state, dt).time == pytest.approx(0.01)
    # Repeated discovery constructs independent registries; nothing self-registers.
    registry = ComponentRegistry()
    discover_models(registry, 'test_dynamics_plugins', data['dynamics'])
    assert registry.names() == ('test_spring_damper',)
    monkeypatch.setattr(
        factory_module, 'DYNAMICS_MODEL_NAMESPACE', 'robot_simulation.dynamics.target'
    )
    assert SimulationFactory.with_defaults().dynamics_registry.names() == (
        'equation_state_space',
        'fight_flight',
        'saturated_fight_flight',
    )


def test_time_dependent_acceleration_preserves_rk4_stage_times():
    """An acceleration equal to time integrates to the expected cubic position."""

    def acceleration(position, velocity, time, parameters, context):
        return np.full(3, time)

    result = RK4Integrator().step(build(acceleration), np.zeros(6), [], 0.0, 1.0, {})
    np.testing.assert_allclose(result, [1.0 / 6.0] * 3 + [0.5] * 3)


@pytest.mark.parametrize('dimension', [0, -1, True, 3.5])
def test_rejects_invalid_dimension_declaration(dimension):
    """A declaration cannot silently coerce invalid spatial dimensions."""
    acceleration_model = importlib.import_module(
        'robot_simulation.dynamics.acceleration_model'
    ).acceleration_model
    with pytest.raises(ValueError, match='dimension'):
        acceleration_model('test_invalid', dimension=dimension, schema=SCHEMA)


def test_other_mathematical_dimension_is_explicitly_not_deployable():
    """Other Dynamics dimensions do not claim supported TargetState deployment."""
    acceleration_model = importlib.import_module(
        'robot_simulation.dynamics.acceleration_model'
    ).acceleration_model

    @acceleration_model('test_planar', dimension=2, schema=SCHEMA)
    def acceleration(position, velocity, time, parameters, context):
        return np.zeros(2)

    assert acceleration.capabilities['deployable'] is False
    assert acceleration.capabilities['spatial_dimension'] == 2
    dynamics = acceleration.builder(**PARAMETERS)
    np.testing.assert_array_equal(
        dynamics.derivative(np.ones(4), [], 0.0, {}), [1.0, 1.0, 0.0, 0.0]
    )


def test_complex_acceleration_is_rejected_without_discarding_imaginary_part():
    dynamics = build(lambda *args: np.ones(3, dtype=complex) * (1.0 + 2.0j))
    with pytest.raises(ValueError, match='real'):
        dynamics.derivative(np.zeros(6), [], 0.0, {})


def test_complex_state_is_rejected_and_context_callback_cannot_mutate_source():
    with pytest.raises(ValueError, match='real'):
        build().derivative(np.ones(6, dtype=complex), [], 0.0, {})
    tracking = np.array([1.0, 2.0, 3.0])

    def mutate(position, velocity, time, parameters, context):
        context['tracking_position_world'][:] = 0
        return np.zeros(3)

    build(mutate).derivative(np.zeros(6), [], 0.0, {'tracking_position_world': tracking})
    np.testing.assert_array_equal(tracking, [1.0, 2.0, 3.0])
