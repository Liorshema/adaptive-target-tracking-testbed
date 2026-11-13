"""Independent posterior references and causal online acceleration contracts."""

from pathlib import Path

import numpy as np

import pytest

from robot_prediction.baseline.gaussian_process import GaussianProcessPredictor, RBFRegression
from robot_prediction.interfaces.prediction_data import PredictionInput

import yaml


def parameters(**overrides):
    """Parameters."""
    path = Path(__file__).parents[1] / 'config/prediction.yaml'
    values = yaml.safe_load(path.read_text())['predictors']['gaussian_process']['parameters']
    return {**values, **overrides}


def model(**overrides):
    """Model."""
    return GaussianProcessPredictor(**parameters(**overrides))


def query(state, time=0.0, dt=0.1, horizon=4):
    """Query."""
    return PredictionInput(np.asarray(state), time, dt, horizon)


def test_posterior_matches_independent_dense_reference():
    """Posterior matches independent dense reference."""
    X = np.array([[0.0, 1.0], [0.5, -0.2], [1.0, 0.0]])
    Y = np.array([[1.0, -1.0], [2.0, 0.5], [0.0, 1.0]])
    point = np.array([0.3, 0.2])
    length, signal, noise, jitter = 1.3, 2.0, 0.2, 1e-8
    gp = RBFRegression(X, Y, length, signal, noise, jitter, 1e10)
    K = np.array([[signal * np.exp(-sum((a - b) ** 2) / (2 * length**2)) for b in X] for a in X])
    C = K + (noise + jitter) * np.eye(3)
    k = np.array([signal * np.exp(-sum((a - point) ** 2) / (2 * length**2)) for a in X])
    mean, variance = gp.posterior(point)
    np.testing.assert_allclose(mean, k @ np.linalg.solve(C, Y), atol=1e-13)
    np.testing.assert_allclose(variance, signal - k @ np.linalg.solve(C, k), atol=1e-13)
    assert mean.shape == variance.shape == (2,)
    np.testing.assert_allclose(gp.kernel(X, X), K)


def test_duplicate_inputs_and_positive_definite_guard():
    """Duplicate inputs and positive definite guard."""
    X = np.ones((10, 4))
    gp = RBFRegression(X, np.ones((10, 2)), 1.0, 1.0, 0.01, 1e-9, 1e8)
    mean, var = gp.posterior(X[0] + 1e-14)
    assert np.all(np.isfinite(mean)) and np.all(var >= 0)
    with pytest.raises(ValueError, match='condition'):
        RBFRegression(X, np.ones((10, 2)), 1.0, 1.0, 1e-15, 1e-16, 10.0)


@pytest.mark.parametrize('dimension', [1, 2, 3, 5])
def test_cv_and_accelerating_recursive_propagation(dimension):
    """Cv and accelerating recursive propagation."""
    for acceleration in [0.0, 0.3]:
        gp = model(
            min_training_samples=4, refit_interval=1, length_scale=1e4, observation_noise=1e-7
        )
        for k in range(9):
            t = 0.1 * k
            state = np.r_[
                np.full(dimension, 2 + t + 0.5 * acceleration * t * t),
                np.full(dimension, 1 + acceleration * t),
            ]
            if k:
                gp.update_at(state, 0.1, t)
            output = gp.predict(query(state, t))
        assert output.model_name == 'gaussian_process'
        future = 0.8 + 0.1 * np.arange(1, 5)
        np.testing.assert_allclose(
            output.states[:, :dimension],
            np.repeat((2 + future + 0.5 * acceleration * future**2)[:, None], dimension, axis=1),
            atol=1e-6,
        )
        np.testing.assert_allclose(
            output.states[:, dimension:],
            np.repeat((1 + acceleration * future)[:, None], dimension, axis=1),
            atol=1e-6,
        )
        np.testing.assert_allclose(output.timestamps, future)
        assert output.covariance is None and output.confidence is None


def test_causality_irregular_samples_bounded_history_and_refits():
    """Causality irregular samples bounded history and refits."""
    gp = model(history_capacity=6, min_training_samples=3, refit_interval=2)
    x = np.array([1.0, 0.1])
    first = gp.predict(query(x))
    assert first.model_name == 'gaussian_process/cv_warmup'
    gp.predict(query(x))
    assert gp.diagnostics['training_pairs'] == 0
    times = [0.07, 0.18, 0.27, 0.39, 0.50, 0.61, 0.72, 0.81]
    previous = 0.0
    for count, t in enumerate(times, 1):
        state = np.array([1 + 0.1 * t, 0.1 + 0.2 * t])
        gp.update_at(state, t - previous, t)
        assert gp.diagnostics['training_pairs'] == min(count, 6)
        output = gp.predict(query(state, t))
        assert np.all(np.isfinite(output.states))
        previous = t
    assert gp.diagnostics['fit_count'] == 3  # transitions 3, 5, 7
    assert gp.diagnostics['last_training_end'] == 0.81
    assert model().diagnostics['training_pairs'] == 0
    gp.reset()
    assert gp.diagnostics['fit_count'] == gp.diagnostics['training_pairs'] == 0


@pytest.mark.parametrize('kind', ['duplicate', 'backwards', 'dt', 'dimension', 'nan'])
def test_bad_observations_clear_learned_state(kind):
    """Bad observations clear learned state."""
    gp = model(min_training_samples=2)
    gp.predict(query([0.0, 1.0]))
    gp.update_at([0.1, 1.0], 0.1, 0.1)
    state, dt, timestamp = [0.0, 1.0], 0.1, 0.2
    if kind == 'duplicate':
        timestamp = 0.1
    elif kind == 'backwards':
        timestamp = 0.0
    elif kind == 'dt':
        dt = 0.3
    elif kind == 'dimension':
        state = [1.0, 2.0, 3.0, 4.0]
    else:
        state = [float('nan'), 1.0]
    with pytest.raises(ValueError):
        gp.update_at(state, dt, timestamp)
    assert gp.diagnostics['training_pairs'] == 0
    assert gp.predict(query([0.0, 1.0])).model_name.endswith('/cv_warmup')


def test_gap_and_clock_reset_and_work_budgets():
    """Gap and clock reset and work budgets."""
    gp = model(min_training_samples=2)
    gp.predict(query([0.0, 1.0]))
    gp.update_at([1.0, 1.0], 1.0, 1.0)
    assert gp.diagnostics['training_pairs'] == 0
    with pytest.raises(ValueError):
        gp.predict(query([0.0, 1.0], 0.0))
    assert gp.diagnostics['last_observation_time'] is None
    for invalid in [
        query([0.0, 1.0], horizon=1001),
        query([0.0, 1.0], dt=0.0),
        query([0.0, 1.0], time=1e300),
        query([0.0, 1.0], horizon=True),
        query([0.0, np.inf]),
    ]:
        with pytest.raises(ValueError):
            model().predict(invalid)


def test_rejected_fit_is_explicit_cv_and_noise_is_finite():
    """Rejected fit is explicit cv and noise is finite."""
    gp = model(min_training_samples=2, refit_interval=1, max_condition=1.0)
    gp.predict(query([0.0, 1.0]))
    for k in range(1, 6):
        state = np.array([0.1 * k, 1.0 + 0.01 * (-1) ** k])
        gp.update_at(state, 0.1, 0.1 * k)
        out = gp.predict(query(state, 0.1 * k))
    assert out.model_name == 'gaussian_process/cv_conditioning'
    np.testing.assert_allclose(out.states[:, 0], state[0] + out.timestamps.__sub__(0.5) * state[1])
    assert gp.diagnostics['fit_failures'] > 0
    with pytest.raises(ValueError):
        gp.acceleration_posterior(state)


@pytest.mark.parametrize(
    'override',
    [
        {'kernel': 'bad'},
        {'observation_noise': 0},
        {'jitter': -1},
        {'history_capacity': 257},
        {'min_training_samples': 65},
        {'normalize_features': 'yes'},
    ],
)
def test_configuration_guards(override):
    """Configuration guards."""
    with pytest.raises(ValueError):
        model(**override)


def test_real_pipeline_updates_beyond_previous_horizon_and_freezes_forecasts():
    """Real pipeline updates beyond previous horizon and freezes forecasts."""
    from robot_prediction.configuration.prediction_config import PredictionConfig
    from robot_prediction.factory.system_factory import PredictionSystemFactory

    config = PredictionConfig(Path(__file__).parents[1] / 'config/prediction.yaml')
    system = PredictionSystemFactory(config).create(
        predictor_type='gaussian_process', adaptation_type='none', fusion_type='none'
    )
    state = np.array([0.0, 1.0])
    original = system.predict(query(state, horizon=1))
    saved = original.states.copy()
    for k in range(1, 25):
        t = k * 0.2
        state = np.array([t, 1.0 + 0.02 * np.sin(k)])
        assert system.update(state, observation_timestamp=t) is None
        current = system.predict(query(state, t, horizon=1))
    assert current.model_name == 'gaussian_process'
    assert system._predictor.diagnostics['training_pairs'] == 24
    np.testing.assert_array_equal(original.states, saved)
    mean, var = system._predictor.acceleration_posterior(state)
    assert mean.shape == var.shape == (1,) and np.all(var >= 0)


def test_recursive_nonlinear_forecast_uses_new_state_at_each_step():
    """Recursive nonlinear forecast uses new state at each step."""
    gp = model(min_training_samples=3, refit_interval=1)
    for k in range(12):
        t = k * 0.1
        state = np.array([np.cos(t), -np.sin(t)])
        if k:
            gp.update_at(state, 0.1, t)
        result = gp.predict(query(state, t, 0.1, 5))
    expected = []
    x = state.copy()
    for _ in range(5):
        a, _ = gp.acceleration_posterior(x)
        x = np.r_[x[:1] + 0.1 * x[1:] + 0.005 * a, x[1:] + 0.1 * a]
        expected.append(x)
    np.testing.assert_allclose(result.states, expected, atol=1e-14)
