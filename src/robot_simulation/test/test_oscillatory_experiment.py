"""Verify existing periodic-field units and deterministic spatial experiments."""

from pathlib import Path

import numpy as np

import pytest

from robot_models.types import TargetState

from robot_simulation.disturbances.sinusoidal_field import (
    SinusoidalAccelerationField, SinusoidalFieldParameters)
from robot_configuration.yaml_loader import load_yaml
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.state.simulation_state import SimulationState

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def test_periodic_field_uses_hertz_and_radian_phase():
    """A one-second evaluation must match the configured physical periods."""
    field = SinusoidalAccelerationField(SinusoidalFieldParameters(
        np.array([0., .4, .3]), np.array([0., .25, .5]), np.array([0., 0., np.pi / 2])))
    np.testing.assert_allclose(field.acceleration(np.zeros(3), 0.), [0., 0., .3], atol=1e-14)
    np.testing.assert_allclose(field.acceleration(np.ones(3), 1.), [0., .4, -.3], atol=1e-14)


@pytest.mark.parametrize('frequency', [[0., -.1, .2], [0., np.nan, .2], [0., .2]])
def test_periodic_field_rejects_invalid_frequencies(frequency):
    """Invalid field geometry/rates fail before target integration."""
    with pytest.raises(ValueError):
        SinusoidalFieldParameters(np.ones(3), np.array(frequency), np.zeros(3))


@pytest.mark.parametrize('scenario', [
    'reactive_target_oscillatory.yaml', 'reactive_target_oscillatory_aggressive_reference.yaml'])
def test_configured_motion_remains_spatial_bounded_and_repeatable(scenario):
    """Both selected fields must sustain post-transient Y/Z reversals with RK4."""
    scene = load_yaml(CONFIG / 'scenarios' / scenario)
    target = load_yaml(CONFIG.parent / scene['target']['config'])
    environment = load_yaml(CONFIG.parent / scene['simulation']['environment_config'])
    integration = load_yaml(CONFIG.parent / scene['simulation']['config'])
    engine, dt = SimulationFactory.with_defaults().create(target, environment, integration)
    center = np.array(environment['environment']['acceleration_fields'][0]['parameters']['center'])

    def integrate():
        state = SimulationState(TargetState(
            np.array(scene['target']['initial_position']),
            np.array(scene['target']['initial_velocity'])), np.array([1.2, 0., .85]), 0.)
        rows = []
        for _ in range(round(20. / dt)):
            state = engine.step(state, dt)
            rows.append(np.r_[state.target.position, state.target.velocity])
        return np.asarray(rows)

    rows = integrate()
    assert np.isfinite(rows).all()
    assert np.linalg.norm(rows[:, :3] - center, axis=1).max() < 1.
    settled = rows[round(10. / dt):]
    assert np.ptp(settled[:, 1]) > .3 and np.ptp(settled[:, 2]) > .1
    for index in [4, 5]:
        signs = np.sign(settled[np.abs(settled[:, index]) > .001, index])
        assert np.count_nonzero(np.diff(signs)) >= 3
    if scenario.endswith('_aggressive_reference.yaml'):
        np.testing.assert_array_equal(rows, integrate())
