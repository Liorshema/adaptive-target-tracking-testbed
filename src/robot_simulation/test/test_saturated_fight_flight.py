"""Verify selectable bounded radial interaction with the shared dynamics engine."""

import numpy as np
import pytest

from robot_simulation.disturbances.vector_field import (
    RestoringVectorField,
    VectorFieldParameters,
)
from robot_simulation.dynamics.target.fight_flight import (
    FightFlightDynamics,
    FightFlightParameters,
)
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.integration.rk4 import RK4Integrator


def model(field=None, **overrides):
    """Construct through the same registered selector used by simulation YAML."""
    parameters = {'a_max': 1.5, 'transition_length': .5, 'preferred_distance': 1.,
                  'damping': .15, 'distance_epsilon': 1e-6}
    parameters.update(overrides)
    return SimulationFactory.with_defaults().dynamics_registry.create(
        {'type': 'saturated_fight_flight', 'parameters': parameters}, environment_field=field)


def derivative(instance, position, velocity=None, reference=None):
    """Evaluate an explicitly six-dimensional position/velocity target state."""
    return instance.derivative(
        np.r_[position, np.zeros(3) if velocity is None else velocity], np.empty(0), 0.,
        {'tracking_position_world': np.zeros(3) if reference is None else reference})


@pytest.mark.parametrize('radius,sign', [(.5, -1), (1., 0), (2., 1)])
def test_interaction_sign_and_zero_at_preferred_distance(radius, sign):
    """Preserve attraction inside D and repulsion outside D."""
    result = derivative(model(damping=0.), [radius, 0., 0.])[3:]
    assert np.sign(result[0]) == sign
    np.testing.assert_allclose(result[1:], [0., 0.])


@pytest.mark.parametrize('direction', [[1., 0., 0.], [0., -1., 0.], [0., 0., 1.], [1., 2., -3.]])
def test_interaction_is_bounded_and_rotationally_consistent(direction):
    """The same regularized scalar law applies in every 3D direction."""
    direction = np.array(direction) / np.linalg.norm(direction)
    instance = model(damping=0.)
    for radius in [1e-12, .1, .7, 1., 2., 10., 1e6]:
        acceleration = derivative(instance, radius * direction)[3:]
        assert np.linalg.norm(acceleration) <= 1.5 + 1e-12
        expected = 1.5 * np.tanh((radius - 1.) / .5) * (
            radius / np.sqrt(radius ** 2 + 1e-12)) * direction
        np.testing.assert_allclose(acceleration, expected, atol=1e-12)


def test_coincident_position_and_near_zero_distance_are_finite():
    """Coincident points contribute no undefined radial direction."""
    np.testing.assert_allclose(derivative(model(), [0., 0., 0.]), np.zeros(6))
    assert np.isfinite(derivative(model(), [1e-15, -1e-15, 1e-15])).all()


def test_damping_and_position_derivative_have_physical_units():
    """At D only damping remains, and position rate equals velocity."""
    velocity = np.array([.2, -.4, .6])
    result = derivative(model(), [1., 0., 0.], velocity)
    np.testing.assert_allclose(result[:3], velocity)
    np.testing.assert_allclose(result[3:], -.15 * velocity)


def test_local_slope_matches_configured_acceleration_over_transition_length():
    """The transition has the analytical local gradient a_max/length."""
    instance = model(damping=0.)
    h = 1e-6
    slope = (derivative(instance, [1. + h, 0., 0.])[3] -
             derivative(instance, [1. - h, 0., 0.])[3]) / (2 * h)
    assert slope == pytest.approx(3., rel=1e-9)


def test_environment_is_added_separately_and_reference_changes_interaction():
    """Fields compose additively and moving camera position affects the force."""
    field = RestoringVectorField(VectorFieldParameters(np.zeros(3), .5 * np.eye(3)))
    result = derivative(model(field, a_max=2., damping=0.), [2., 0., 0.])[3:]
    np.testing.assert_allclose(result, [2. * np.tanh(2.) * 2. / np.sqrt(4. + 1e-12) - 1., 0., 0.])
    assert derivative(model(damping=0.), [2., 0., 0.], reference=[1.5, 0., 0.])[3] < 0.


@pytest.mark.parametrize('key,value', [
    ('a_max', 0.), ('a_max', -1.), ('transition_length', 0.),
    ('preferred_distance', 0.), ('damping', -.1), ('distance_epsilon', 0.),
    ('a_max', float('nan')), ('damping', float('inf')), ('transition_length', float('inf')),
])
def test_invalid_saturated_parameters_are_rejected(key, value):
    """Nonphysical coefficients fail before numerical integration."""
    with pytest.raises(ValueError, match='finite|positive|non-negative'):
        model(**{key: value})


def test_linear_derivative_remains_the_original_equation():
    """Adding selection must retain the original linear vector field."""
    instance = FightFlightDynamics(FightFlightParameters(1., 1., .5, 1e-6))
    p, v, reference = np.array([2., -.3, .9]), np.array([.1, .2, -.4]), np.array([.5, 0., .4])
    displacement = p - reference
    radius = np.linalg.norm(displacement)
    expected = (radius - 1.) * displacement / np.sqrt(radius ** 2 + 1e-12) - .5 * v
    np.testing.assert_allclose(
        derivative(instance, p, v, reference), np.r_[v, expected], atol=1e-15)


def test_saturated_rk4_integration_is_repeatable():
    """Repeated deterministic RK4 integration returns identical state vectors."""
    instance = model()

    def integrate():
        """Advance the same fixed initial value problem for one second."""
        state = np.array([2., .3, .9, .1, -.2, .15])
        for step in range(100):
            state = RK4Integrator().step(instance, state, np.empty(0), step * .01, .01,
                                         {'tracking_position_world': np.array([.5, 0., .6])})
        return state
    np.testing.assert_array_equal(integrate(), integrate())
