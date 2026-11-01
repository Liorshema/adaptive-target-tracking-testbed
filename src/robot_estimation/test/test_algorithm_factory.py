"""Role-free construction uses the existing numerical KF/EKF implementations."""

from importlib import import_module

import numpy as np
import pytest

from robot_estimation.filters.extended_kalman_filter import ExtendedKalmanFilter
from robot_estimation.filters.kalman_filter import KalmanFilter
from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate


def registry(configuration=None):
    module = import_module('robot_estimation.factory.algorithm_factory')
    return module.create_algorithm_registry(
        configuration
        or {
            'estimators': {'kf': {'parameters': {}}, 'ekf': {'parameters': {}}},
        }
    )


def linear_dependencies(dimension=2):
    return {
        'initial_state': StateEstimate(np.zeros(dimension), np.eye(dimension)),
        'state_transition': lambda dt: np.eye(dimension),
        'process_covariance': lambda dt: dt * np.eye(dimension),
        'measurement_matrix': np.eye(dimension),
    }


def nonlinear_dependencies(dimension=2):
    return {
        'initial_state': StateEstimate(np.zeros(dimension), np.eye(dimension)),
        'process_function': lambda x, u, dt: x.copy(),
        'process_jacobian': lambda x, u, dt: np.eye(dimension),
        'process_covariance': lambda dt: dt * np.eye(dimension),
    }


def test_generic_algorithms_expose_contracts_without_role_or_fixed_state_layout():
    algorithms = registry()
    assert algorithms.names() == ('ekf', 'kf')
    for name, expected_kind in [('kf', 'linear'), ('ekf', 'nonlinear')]:
        metadata = algorithms.describe(name)
        assert metadata['defaults'] == {}
        assert metadata['capabilities']['process_kind'] == expected_kind
        assert not {'role', 'state_layout', 'state_dimension'} & metadata['capabilities'].keys()
    assert algorithms.describe('ekf')['capabilities']['dynamic_measurements']
    assert algorithms.describe('ekf')['capabilities']['state_constraints']


@pytest.mark.parametrize('dimension', [1, 3])
def test_kf_reuses_existing_class_for_different_state_and_measurement_bindings(dimension):
    dependencies = linear_dependencies(dimension)
    dependencies['measurement_matrix'] = np.ones((1, dimension))
    algorithm = registry().create('kf', **dependencies)
    assert type(algorithm) is KalmanFilter
    prediction = algorithm.predict(1.0)
    np.testing.assert_array_equal(prediction.mean, np.zeros(dimension))
    np.testing.assert_array_equal(prediction.covariance, 2 * np.eye(dimension))
    result = algorithm.update(Measurement(np.array([2.0]), np.array([[2.0]]), 4.0))
    np.testing.assert_allclose(result.mean, np.full(dimension, 2 / (dimension + 1)))
    np.testing.assert_allclose(
        result.covariance,
        2 * np.eye(dimension) - 2 / (dimension + 1) * np.ones((dimension, dimension)),
    )
    assert result.timestamp == 4.0


@pytest.mark.parametrize('dimension', [1, 4])
def test_ekf_reuses_existing_class_with_nonlinear_models_and_dynamic_measurements(dimension):
    dependencies = nonlinear_dependencies(dimension)
    dependencies['initial_state'] = StateEstimate(np.ones(dimension), np.eye(dimension), 2.0)
    dependencies['process_function'] = lambda x, u, dt: x**2 + dt
    dependencies['process_jacobian'] = lambda x, u, dt: np.diag(2 * x)
    algorithm = registry().create('ekf', **dependencies)
    assert type(algorithm) is ExtendedKalmanFilter
    prediction = algorithm.predict(1.0)
    np.testing.assert_array_equal(prediction.mean, np.full(dimension, 2.0))
    np.testing.assert_array_equal(prediction.covariance, 5 * np.eye(dimension))
    assert prediction.timestamp == 3.0
    result = algorithm.update(
        Measurement(np.array([5.0]), np.array([[1.0]]), 4.0),
        measurement_function=lambda x: x[:1] ** 2,
        measurement_jacobian=lambda x: np.array([[4.0] + [0.0] * (dimension - 1)]),
    )
    expected = np.full(dimension, 2.0)
    expected[0] += 20 / 81
    np.testing.assert_allclose(result.mean, expected)
    expected_covariance = 5 * np.eye(dimension)
    expected_covariance[0, 0] = 5 / 81
    np.testing.assert_allclose(result.covariance, expected_covariance)


def test_algorithm_parameters_are_closed_and_unknown_algorithms_rejected():
    algorithms = registry()
    with pytest.raises(ValueError, match='unknown'):
        algorithms.create('kf', {'target_spatial_dimension': 3}, **linear_dependencies())
    with pytest.raises(KeyError, match='Unknown component'):
        algorithms.create('robot_ekf')
    with pytest.raises(ValueError, match='unknown'):
        registry(
            {'estimators': {'kf': {'parameters': {'tolerance': 1.0}}, 'ekf': {'parameters': {}}}}
        )


@pytest.mark.parametrize(
    'name,key',
    [
        ('kf', 'state_transition'),
        ('kf', 'process_covariance'),
        ('ekf', 'process_function'),
        ('ekf', 'process_jacobian'),
        ('ekf', 'process_covariance'),
    ],
)
def test_required_model_dependencies_must_be_callable(name, key):
    dependencies = linear_dependencies() if name == 'kf' else nonlinear_dependencies()
    dependencies[key] = None
    with pytest.raises(ValueError, match=key):
        registry().create(name, **dependencies)


@pytest.mark.parametrize(
    'mean,covariance',
    [
        ([np.nan, 0.0], np.eye(2)),
        ([np.inf, 0.0], np.eye(2)),
        ([0.0, 0.0], [[1.0, 2.0], [0.0, 1.0]]),
        ([0.0, 0.0], [[1.0, 0.0], [0.0, -1.0]]),
        ([0.0, 0.0], [[1.0, 0.0], [0.0, np.nan]]),
    ],
)
def test_initial_distribution_must_be_finite_symmetric_positive_semidefinite(mean, covariance):
    dependencies = linear_dependencies()
    dependencies['initial_state'] = StateEstimate(mean, covariance)
    with pytest.raises(ValueError, match='initial_state'):
        registry().create('kf', **dependencies)


@pytest.mark.parametrize(
    'matrix', [np.ones((1, 3)), [[np.nan, 0.0]], [[1j, 0.0]], np.empty((0, 2))]
)
def test_linear_measurement_matrix_contract_is_validated_before_construction(matrix):
    dependencies = linear_dependencies()
    dependencies['measurement_matrix'] = matrix
    with pytest.raises(ValueError, match='measurement_matrix'):
        registry().create('kf', **dependencies)


@pytest.mark.parametrize(
    'dependencies',
    [
        {'measurement_function': lambda x: x},
        {'measurement_jacobian': lambda x: np.eye(len(x))},
        {'measurement_function': 1.0, 'measurement_jacobian': lambda x: np.eye(len(x))},
        {'state_constraint': 1.0},
    ],
)
def test_ekf_rejects_incomplete_or_noncallable_measurement_and_constraint_bindings(dependencies):
    with pytest.raises(ValueError, match='measurement|state_constraint'):
        registry().create('ekf', **{**nonlinear_dependencies(), **dependencies})


def test_model_providers_are_not_evaluated_before_filter_closure_is_initialized():
    holder = {}
    dependencies = nonlinear_dependencies()
    dependencies['process_covariance'] = lambda dt: dt * holder['filter'].state.covariance
    holder['filter'] = registry().create('ekf', **dependencies)
    np.testing.assert_array_equal(holder['filter'].predict(1.0).covariance, 2 * np.eye(2))


@pytest.mark.parametrize(
    'name,key,provider',
    [
        ('kf', 'state_transition', lambda dt: np.ones((3, 3))),
        ('kf', 'process_covariance', lambda dt: [[1.0, 0.0], [0.0, -1.0]]),
        ('kf', 'process_covariance', lambda dt: [[1.0, 1.0], [0.0, 1.0]]),
        ('ekf', 'process_function', lambda x, u, dt: [np.inf, 0.0]),
        ('ekf', 'process_jacobian', lambda x, u, dt: np.ones((3, 3))),
        ('ekf', 'process_covariance', lambda dt: np.eye(3)),
    ],
)
def test_deferred_model_outputs_are_checked_when_the_real_predict_step_evaluates_them(
    name, key, provider
):
    dependencies = linear_dependencies() if name == 'kf' else nonlinear_dependencies()
    dependencies[key] = provider
    algorithm = registry().create(name, **dependencies)
    with pytest.raises(ValueError, match=key):
        algorithm.predict(1.0)


@pytest.mark.parametrize('dt', [float('nan'), float('inf'), True])
@pytest.mark.parametrize('name', ['kf', 'ekf'])
def test_registered_algorithm_rejects_invalid_time_even_for_constant_callbacks(name, dt):
    deps = linear_dependencies() if name == 'kf' else nonlinear_dependencies()
    deps['process_covariance'] = lambda dt: np.eye(2)
    algorithm = registry().create(name, **deps)
    with pytest.raises(ValueError, match='time|dt'):
        algorithm.predict(dt)
