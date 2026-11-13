"""Independent EDMD references and causal online lifecycle checks."""

import numpy as np

import pytest

from robot_prediction.baseline.koopman_edmd import (
    KoopmanEDMDPredictor,
    PolynomialDictionary,
    fit_edmd,
)
from robot_prediction.evaluation.performance_monitor import PerformanceMonitor
from robot_prediction.interfaces.prediction_data import PredictionInput
from robot_prediction.trajectory_prediction import TrajectoryPredictionSystem


def predictor(**changes):
    """Use small explicit numerical fixtures without duplicating owner defaults."""
    parameters = {
        'basis': 'polynomial',
        'polynomial_degree': 2,
        'regularization': 1e-8,
        'training_interval': 0.1,
        'history_capacity': 60,
        'min_training_samples': 8,
        'refit_interval': 2,
        'max_condition': 1e12,
        'minimum_scale': 1e-3,
        'max_observation_gap': 0.5,
        'max_features': 128,
        'max_transition_steps': 1000,
        'max_spectral_radius': 1.05,
    }
    parameters.update(changes)
    return KoopmanEDMDPredictor(**parameters)


def request(state=(0.0, 1.0), timestamp=0.0, dt=0.1, horizon=4):
    """Create a generic one-dimensional position/velocity input."""
    return PredictionInput(np.array(state), timestamp, dt, horizon)


def test_polynomial_dictionary_reconstructs_physical_states_and_cross_terms():
    """Affine normalization must preserve original coordinates and polynomial order."""
    dictionary = PolynomialDictionary(np.array([2.0, -1.0]), np.array([2.0, 4.0]), 2, 20)
    lifted = dictionary.lift(np.array([4.0, 7.0]))
    np.testing.assert_array_equal(lifted, [1.0, 1.0, 2.0, 1.0, 2.0, 4.0])
    np.testing.assert_array_equal(dictionary.decoder @ lifted, [4.0, 7.0])


def test_ridge_edmd_matches_independent_augmented_least_squares():
    """Check ridge orientation/transpose independently of the production fit."""
    X = np.array([[1.0, 1.0, 1.0, 1.0], [-2.0, -0.5, 1.0, 3.0]])
    Y = np.array([[1.0, 1.0, 1.0, 1.0], [-3.0, 0.0, 3.0, 7.0]])
    ridge = 0.2
    expected = np.linalg.lstsq(
        np.vstack([X.T, np.sqrt(ridge) * np.eye(2)]),
        np.vstack([Y.T, np.zeros((2, 2))]),
        rcond=None,
    )[0].T
    actual, condition = fit_edmd(X, Y, ridge, 1e9)
    np.testing.assert_allclose(actual, expected, atol=1e-12)
    assert 1 < condition < 20


def test_known_linear_system_matches_independent_matrix_powers():
    """Nontrivial state transition predictions must match a separately given A."""
    A = np.array([[0.9, 0.1], [-0.2, 0.8]])
    instance = predictor(
        basis='linear',
        polynomial_degree=1,
        min_training_samples=15,
        regularization=1e-12,
        refit_interval=1,
    )
    state = np.array([2.0, 1.0])
    instance.predict(request(state))
    for k in range(1, 31):
        state = A @ state
        instance.update_at(state, 0.1, k * 0.1)
    result = instance.predict(request(state, 3.0))
    expected = np.array([np.linalg.matrix_power(A, h) @ state for h in range(1, 5)])
    assert result.model_name == 'koopman_edmd'
    np.testing.assert_allclose(result.states, expected, atol=2e-6)
    assert result.covariance is None and result.confidence is None


def test_nonlinear_lift_has_closed_prediction_subspace_that_linear_fit_lacks():
    """A quadratic state dependency needs a genuine nonlinear observable."""
    rng = np.random.default_rng(7)
    states = rng.uniform(-1.0, 1.0, size=(2, 200))
    following = np.vstack([0.9 * states[0], 0.8 * states[1] + 0.2 * states[0] ** 2])
    errors = []
    for degree in [1, 2]:
        dictionary = PolynomialDictionary(np.zeros(2), np.ones(2), degree, 20)
        K, _ = fit_edmd(dictionary.lift(states), dictionary.lift(following), 1e-10, 1e14)
        physical = np.array([0.7, -0.4])
        z = dictionary.lift(physical)
        error = []
        for _ in range(5):
            physical = np.array([0.9 * physical[0], 0.8 * physical[1] + 0.2 * physical[0] ** 2])
            z = K @ z
            error.append(np.linalg.norm(dictionary.decoder @ z - physical))
        errors.append(max(error))
    assert errors[0] > 0.01
    assert errors[1] < 1e-8


def test_warmup_is_explicit_and_repeated_forecasts_do_not_train():
    """Predicting twice must not invent a new observation or a trained model."""
    instance = predictor()
    a = instance.predict(request())
    b = instance.predict(request())
    assert a.model_name == 'koopman_edmd/cv_warmup'
    np.testing.assert_array_equal(a.states, b.states)
    np.testing.assert_allclose(a.states[:, 0], [0.1, 0.2, 0.3, 0.4])
    assert instance.diagnostics['training_pairs'] == 0
    assert instance.diagnostics['fit_count'] == 0


def test_irregular_observations_form_only_completed_causal_grid_pairs():
    """A later bracketing estimate is used only after it has actually arrived."""
    instance = predictor()
    instance.predict(request())
    instance.update_at(np.array([0.06, 1.0]), 0.06, 0.06)
    assert instance.diagnostics['training_pairs'] == 0
    instance.update_at(np.array([0.15, 1.0]), 0.09, 0.15)
    assert instance.diagnostics['training_pairs'] == 1
    assert instance.diagnostics['last_grid_time'] == pytest.approx(0.1)
    instance.update_at(np.array([0.31, 1.0]), 0.16, 0.31)
    assert instance.diagnostics['training_pairs'] == 3
    assert instance.diagnostics['last_grid_time'] == pytest.approx(0.3)


def test_bounded_history_refit_interval_and_run_isolation():
    """Window eviction and refit cadence must not leak to another instance."""
    instance = predictor(history_capacity=10, refit_interval=3)
    instance.predict(request())
    for k in range(1, 31):
        instance.update_at(np.array([k * 0.1, 1.0]), 0.1, k * 0.1)
    assert instance.diagnostics['training_pairs'] == 10
    assert instance.diagnostics['fit_count'] == 8  # pairs 8,11,...,29
    other = predictor()
    assert other.predict(request()).model_name.endswith('/cv_warmup')
    assert other.diagnostics['training_pairs'] == 0
    instance.reset()
    assert instance.predict(request(timestamp=0.0)).model_name.endswith('/cv_warmup')


@pytest.mark.parametrize('stamp', [0.1, 0.0])
def test_invalid_time_sequence_clears_the_learned_epoch(stamp):
    """Duplicate/decreasing updates are explicit errors and cannot retain a model."""
    instance = predictor()
    instance.predict(request())
    instance.update_at(np.array([0.1, 1.0]), 0.1, 0.1)
    with pytest.raises(ValueError, match='time'):
        instance.update_at(np.array([1.0, 1.0]), 0.1, stamp)
    assert instance.diagnostics['training_pairs'] == 0


def test_old_prediction_origin_cannot_use_later_training_observations():
    """Never predict the past using data that became available later."""
    instance = predictor()
    instance.predict(request())
    instance.update_at(np.array([0.2, 1.0]), 0.2, 0.2)
    with pytest.raises(ValueError, match='time|future'):
        instance.predict(request(timestamp=0.1))


def test_gap_reset_never_interpolates_an_unobserved_long_interval():
    """A missing-data gap starts a fresh bounded epoch instead of inventing pairs."""
    instance = predictor()
    instance.predict(request())
    instance.update_at(np.array([2.0, 1.0]), 2.0, 2.0)
    assert instance.diagnostics['training_pairs'] == 0
    assert instance.predict(request((2.0, 1.0), 2.0)).model_name.endswith('/cv_warmup')


def test_conditioning_failure_is_an_explicit_cv_state():
    """A rejected fit must not be tagged as a learned Koopman forecast."""
    instance = predictor(max_condition=1.01)
    instance.predict(request())
    for k in range(1, 11):
        instance.update_at(np.array([k * 0.1, 1.0]), 0.1, k * 0.1)
    assert instance.predict(request((1.0, 1.0), 1.0)).model_name == 'koopman_edmd/cv_conditioning'
    assert instance.diagnostics['fit_failures'] > 0


@pytest.mark.parametrize('dimension', [2, 4, 6])
def test_generic_dimensions_future_grid_and_fractional_transition_times(dimension):
    """Outputs use the caller grid and generic position/velocity ordering."""
    spatial = dimension // 2
    state = np.r_[np.zeros(spatial), np.ones(spatial)]
    instance = predictor(regularization=1e-8)
    instance.predict(request(state))
    for k in range(1, 21):
        state[:spatial] = k * 0.1
        instance.update_at(state.copy(), 0.1, k * 0.1)
    result = instance.predict(request(state, 2.0, dt=0.15, horizon=3))
    assert result.model_name == 'koopman_edmd'
    assert result.states.shape == (3, dimension)
    np.testing.assert_allclose(result.timestamps, [2.15, 2.3, 2.45])
    np.testing.assert_allclose(
        result.states[:, :spatial],
        np.array([2.15, 2.3, 2.45])[:, None] * np.ones((1, spatial)),
        atol=1e-5,
    )


def test_real_system_update_hook_trains_without_future_data():
    """Learning is driven by TrajectoryPredictionSystem's normal observation hook."""
    instance = predictor()
    system = TrajectoryPredictionSystem(instance, PerformanceMonitor(10, 0.5), fusion=None)
    system.predict(request())
    for k in range(1, 15):
        system.update(np.array([k * 0.1, 1.0]), observation_timestamp=k * 0.1)
        output = system.predict(request((k * 0.1, 1.0), k * 0.1))
    assert output.model_name == 'koopman_edmd'
    assert instance.diagnostics['last_observation_time'] == pytest.approx(1.4)


@pytest.mark.parametrize(
    'change',
    [
        {'regularization': 0.0},
        {'history_capacity': 5},
        {'refit_interval': 0},
        {'training_interval': float('nan')},
        {'polynomial_degree': 4},
        {'max_condition': 0.5},
    ],
)
def test_invalid_parameter_domains_are_rejected(change):
    """Bad training/conditioning settings must fail before runtime."""
    with pytest.raises(ValueError):
        predictor(**change)


def test_dictionary_size_is_bounded_and_nonfinite_samples_rejected():
    """Feature explosion or nonfinite estimates must never reach a solve."""
    with pytest.raises(ValueError):
        PolynomialDictionary(np.zeros(10), np.ones(10), 3, 20)
    instance = predictor()
    instance.predict(request())
    with pytest.raises(ValueError):
        instance.update_at(np.array([np.nan, 1.0]), 0.1, 0.1)


def test_correction_fusion_preserves_visible_warmup_identity():
    """Applying an optional correction must not disguise untrained CV as Koopman."""
    from robot_prediction.fusion.weighted_correction import WeightedCorrection

    baseline = predictor().predict(request())
    result = WeightedCorrection().apply(baseline, np.zeros_like(baseline.states))
    assert result.model_name == 'koopman_edmd/cv_warmup+weighted_correction'


def test_historical_owner_snapshot_without_edmd_still_constructs_cv(tmp_path):
    """Adding a trusted module must not require new parameters in frozen old Runs."""
    from pathlib import Path
    import yaml
    from robot_prediction.configuration.prediction_config import PredictionConfig
    from robot_prediction.factory.system_factory import PredictionSystemFactory

    original = Path(__file__).parents[1] / 'config/prediction.yaml'
    data = yaml.safe_load(original.read_text())
    data['predictors'].pop('koopman_edmd', None)
    path = tmp_path / 'old_prediction.yaml'
    path.write_text(yaml.safe_dump(data))
    system = PredictionSystemFactory(PredictionConfig(path)).create(
        predictor_type='constant_velocity', adaptation_type='none', fusion_type='none'
    )
    assert system.predict(request()).model_name == 'constant_velocity'


def test_lifted_rollout_does_not_relift_reconstructed_states():
    """Repeated K propagation must preserve independent lifted coordinates."""
    instance = predictor()
    instance.predict(request((1.0, 1.0)))
    instance._dictionary = PolynomialDictionary(np.zeros(2), np.ones(2), 2, 20)
    K = np.eye(6)
    K[1] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    K[3, 3] = 2.0
    instance._operator = K
    result = instance.predict(request((1.0, 1.0), horizon=3))
    np.testing.assert_array_equal(result.states, [[1.0, 1.0], [2.0, 1.0], [4.0, 1.0]])


def test_invalid_learned_rollout_is_rejected_not_silently_cv():
    """Nonfinite learned propagation must never masquerade as a valid forecast."""
    instance = predictor()
    instance.predict(request())
    instance._dictionary = PolynomialDictionary(np.zeros(2), np.ones(2), 2, 20)
    instance._operator = np.full((6, 6), 1e308)
    with pytest.raises(ValueError, match='overflow|Nonfinite'):
        instance.predict(request(horizon=10))


def test_transition_work_budget_rejects_before_forecasting():
    """Finite grids can still require excessive work; fail before any propagation."""
    instance = predictor(max_transition_steps=3)
    with pytest.raises(ValueError, match='budget|transition'):
        instance.predict(request(dt=0.5, horizon=1))


def test_transition_work_budget_resets_without_dense_resampling():
    """Bound retained data and actual interpolation work independently."""
    instance = predictor(max_transition_steps=3)
    instance.predict(request(horizon=1))
    with pytest.raises(ValueError, match='budget|transition'):
        instance.update_at(np.array([0.5, 1.0]), 0.5, 0.5)
    assert instance.diagnostics['training_pairs'] == 0


def test_nonfinite_cv_warmup_is_rejected_by_the_public_predictor():
    """Fallback has the same finite-output contract as a learned forecast."""
    instance = predictor()
    with pytest.raises(ValueError, match='finite|overflow'):
        instance.predict(request((1e308, 1e308), dt=1.0, horizon=2))


def test_excessive_learned_operator_growth_is_rejected_explicitly():
    """Well-conditioned least squares alone does not certify a usable rollout."""
    instance = predictor(basis='linear', polynomial_degree=1, max_spectral_radius=1.05)
    state = np.array([1.0, 0.3])
    instance.predict(request(state))
    for k in range(1, 21):
        state = state * np.array([1.2, 1.1])
        instance.update_at(state, 0.1, k * 0.1)
    result = instance.predict(request(state, 2.0))
    assert result.model_name == 'koopman_edmd/cv_unstable'
    assert instance.diagnostics['spectral_radius'] > 1.1
    assert 'spectral radius' in instance.diagnostics['fit_error']


def test_huge_output_count_is_rejected_before_allocation():
    """Validate the work budget before converting a huge integer horizon to float."""
    with pytest.raises(ValueError, match='budget'):
        predictor().predict(request(horizon=10**400))


def test_extremely_fine_identification_grid_is_bounded():
    """A valid positive interval must not cause billions of interpolation steps."""
    instance = predictor(training_interval=1e-12)
    instance.predict(request(dt=1e-12, horizon=1))
    with pytest.raises(ValueError, match='budget'):
        instance.update_at(np.array([.1, 1.]), .1, .1)
    assert instance.diagnostics['training_pairs'] == 0
