"""Optional spatial circulation preserves existing Fight/Flight dynamics."""

import numpy as np

import pytest

from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.integration.rk4 import RK4Integrator

from scipy.spatial.transform import Rotation


def model(tangential=None):
    """Use the production registry and external parameter schema."""
    parameters = {'a_max': .15, 'transition_length': .15, 'preferred_distance': 1.,
                  'damping': .5, 'distance_epsilon': 1e-6}
    if tangential is not None:
        parameters['tangential'] = tangential
    return SimulationFactory.with_defaults().dynamics_registry.create(
        {'type': 'saturated_fight_flight', 'parameters': parameters})


def evaluate(instance, position, velocity=(0., 0., 0.), reference=(0., 0., 0.)):
    """Evaluate a six-state initial value problem."""
    return instance.derivative(np.r_[position, velocity], np.empty(0), 0.,
                               {'tracking_position_world': np.asarray(reference)})


def test_tangent_is_perpendicular_bounded_and_spatial():
    """An inclined configurable axis supplies both lateral and vertical force."""
    baseline = model()
    spatial = model({'acceleration': .06, 'axis_world': [.3, -.8, .5]})
    for position in ([1., 0., 0.], [2., -.3, .6], [1e6, 2e6, -3e6]):
        tangent = (evaluate(spatial, position) - evaluate(baseline, position))[3:]
        assert np.dot(tangent, position) == pytest.approx(0., abs=1e-9)
        assert np.linalg.norm(tangent) <= .06 + 1e-12
    acceleration = evaluate(spatial, [1., 0., 0.])[3:]
    assert acceleration[1] > 0. and acceleration[2] > 0.


def test_rotation_covariance_includes_configured_axis_and_translation():
    """The preferred axis is explicit anisotropy and rotates with the geometry."""
    rotation = Rotation.from_rotvec([.8, -.3, .6]).as_matrix()
    axis, position, velocity, reference = map(np.array, (
        [.3, -.8, .5], [2., .3, .8], [.1, -.2, .3], [.2, -.1, .5]))
    shift = np.array([3., -2., 1.])
    first = evaluate(model({'acceleration': .06, 'axis_world': axis}),
                     position, velocity, reference)
    second = evaluate(model({'acceleration': .06, 'axis_world': rotation @ axis}),
                      rotation @ position + shift, rotation @ velocity,
                      rotation @ reference + shift)
    np.testing.assert_allclose(second, np.r_[rotation @ first[:3], rotation @ first[3:]],
                               atol=1e-14)


def test_zero_velocity_coincidence_and_axis_poles_are_finite():
    """No random fallback or undefined direction is used at a pole."""
    spatial = model({'acceleration': .06, 'axis_world': [1., 2., 3.]})
    baseline = model()
    for position in ([0., 0., 0.], [1., 2., 3.], [1e-14, 2e-14, 3e-14]):
        result = evaluate(spatial, position)
        assert np.isfinite(result).all()
        np.testing.assert_allclose(result, evaluate(baseline, position), atol=1e-12)
    assert np.linalg.norm(evaluate(spatial, [1., 0., 0.])[3:]) > .05


def test_zero_strength_is_bitwise_legacy_and_axis_scale_does_not_matter():
    """Defaults and explicitly disabled circulation retain the old equation."""
    state = ([2., -.3, .9], [.1, -.2, .3])
    np.testing.assert_array_equal(evaluate(model(), *state), evaluate(
        model({'acceleration': 0., 'axis_world': [.3, -.8, .5]}), *state))
    np.testing.assert_allclose(
        evaluate(model({'acceleration': .06, 'axis_world': [.3, -.8, .5]}), *state),
        evaluate(model({'acceleration': .06, 'axis_world': [3., -8., 5.]}), *state))


def test_optional_circulation_composes_with_the_linear_model_too():
    """The interaction is reusable and does not replace the configured radial law."""
    instance = SimulationFactory.with_defaults().dynamics_registry.create({
        'type': 'fight_flight', 'parameters': {
            'alpha': 1., 'preferred_distance': 1., 'damping': .5, 'distance_epsilon': 1e-6,
            'tangential': {'acceleration': .06, 'axis_world': [0., -.8, .5]},
        }})
    result = evaluate(instance, [1., 0., 0.])[3:]
    np.testing.assert_allclose(result, .06 * np.array([0., .5, .8]) /
                               np.sqrt(.89 * (1. + 1e-12)))


@pytest.mark.parametrize('overrides', [
    {'acceleration': -.1}, {'acceleration': float('nan')},
    {'axis_world': [0., 0., 0.]}, {'axis_world': [1., 2.]},
    {'axis_world': [float('inf'), 0., 1.]},
])
def test_invalid_circulation_configuration_fails_early(overrides):
    """Bad parameters cannot reach a numerical integration step."""
    parameters = {'acceleration': .06, 'axis_world': [.3, -.8, .5]}
    parameters.update(overrides)
    with pytest.raises(ValueError):
        model(parameters)


def test_spatial_integration_is_deterministic_and_bounded_with_existing_field():
    """Use the existing field and RK4; require spatial motion and containment."""
    from robot_simulation.disturbances.vector_field import (
        RestoringVectorField, VectorFieldParameters)

    instance = model({'acceleration': .06, 'axis_world': [.3, -.8, .5]})
    center = np.array([2.2, 0., .85])
    instance.environment_field = RestoringVectorField(
        VectorFieldParameters(center, .5 * np.eye(3)))

    def integrate():
        state = np.array([2., .25, 1., 0., .1, -.12])
        rows = []
        for step in range(1000):
            state = RK4Integrator().step(instance, state, np.empty(0), step * .02, .02,
                                         {'tracking_position_world': np.array([1.2, 0., .85])})
            rows.append(state.copy())
        return np.asarray(rows)

    rows = integrate()
    np.testing.assert_array_equal(rows, integrate())
    assert np.isfinite(rows).all()
    assert np.max(np.linalg.norm(rows[:, :3] - center, axis=1)) < 1.
    assert np.all(np.ptp(rows[:, :3], axis=0) > .02)
    assert np.linalg.matrix_rank(rows[:, :3] - rows[:, :3].mean(axis=0), tol=1e-6) == 3
