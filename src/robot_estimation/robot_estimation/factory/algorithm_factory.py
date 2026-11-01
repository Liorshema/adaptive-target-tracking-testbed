"""
Role-independent construction of the existing numerical estimation cores.

Only intrinsic algorithm settings belong in ``estimators.<id>.parameters``.
Initial distributions and model callables are explicit injected dependencies.
Construction checks distributions, callability, and static measurement matrices.
Callable output checks are deferred to their actual use: a covariance provider
may close over a filter that has not yet been assigned during construction.
These checks return the original model output without numerical modification.
Measurement covariances and dynamically supplied EKF measurement models remain
the responsibility of the measurement/role binding and existing update contract.
"""

from collections.abc import Mapping
from functools import wraps
from numbers import Real

import numpy as np

from robot_configuration.components import ComponentRegistry
from robot_configuration.model_registration import discover_models
from robot_configuration.model_registration import ModelRegistration
from robot_configuration.model_registration import register_models
from robot_estimation.filters.extended_kalman_filter import ExtendedKalmanFilter
from robot_estimation.filters.kalman_filter import KalmanFilter
from robot_estimation.state import StateEstimate


def _finite_array(value, name, shape=None):
    array = np.asarray(value)
    if array.dtype.kind not in 'iuf' or not np.isfinite(array).all():
        raise ValueError(f'{name} must contain finite real numbers')
    if shape is not None and array.shape != shape:
        raise ValueError(f'{name} must have shape {shape}, got {array.shape}')
    return array


def _covariance(value, name, dimension):
    array = _finite_array(value, name, (dimension, dimension))
    # Match the existing EKF PSD validation tolerance; never repair the matrix.
    if (
        not np.allclose(array, array.T, atol=1e-12, rtol=0.0)
        or np.linalg.eigvalsh(array).min() < -1e-12
    ):
        raise ValueError(f'{name} must be symmetric positive semidefinite')


def _initial_dimension(initial_state):
    if not isinstance(initial_state, StateEstimate):
        raise ValueError('initial_state must be a StateEstimate')
    mean = _finite_array(initial_state.mean, 'initial_state.mean')
    if mean.ndim != 1 or mean.size == 0:
        raise ValueError('initial_state.mean must be a nonempty vector')
    _covariance(initial_state.covariance, 'initial_state.covariance', mean.size)
    timestamp = initial_state.timestamp
    if timestamp is not None and (
        not isinstance(timestamp, Real)
        or isinstance(timestamp, bool)
        or not np.isfinite(timestamp)
    ):
        raise ValueError('initial_state.timestamp must be finite real seconds or None')
    return mean.size


def _callable(value, name, *, optional=False):
    if not (optional and value is None) and not callable(value):
        raise ValueError(f'{name} must be callable')


def _provider(provider, name, check):
    _callable(provider, name)

    @wraps(provider)
    def checked(*args, **kwargs):
        dt = kwargs.get('dt', args[-1] if args else None)
        if isinstance(dt, bool) or not isinstance(dt, Real) or not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive finite real time in seconds')
        result = provider(*args, **kwargs)
        check(result)
        return result

    return checked


def _matrix(value, name, rows=None, columns=None):
    array = _finite_array(value, name)
    if (
        array.ndim != 2
        or min(array.shape) == 0
        or (rows is not None and array.shape[0] != rows)
        or (columns is not None and array.shape[1] != columns)
    ):
        raise ValueError(
            f'{name} must be a nonempty matrix with '
            f'{rows if rows is not None else "compatible"} rows and '
            f'{columns if columns is not None else "compatible"} columns; got {array.shape}'
        )


def _build_kf(
    initial_state, state_transition, process_covariance, measurement_matrix, control_matrix=None
):
    dimension = _initial_dimension(initial_state)
    _matrix(measurement_matrix, 'measurement_matrix', columns=dimension)
    _callable(control_matrix, 'control_matrix', optional=True)
    return KalmanFilter(
        initial_state=initial_state,
        state_transition=_provider(
            state_transition,
            'state_transition',
            lambda value: _finite_array(value, 'state_transition', (dimension, dimension)),
        ),
        process_covariance=_provider(
            process_covariance,
            'process_covariance',
            lambda value: _covariance(value, 'process_covariance', dimension),
        ),
        measurement_matrix=measurement_matrix,
        control_matrix=(
            None
            if control_matrix is None
            else _provider(
                control_matrix,
                'control_matrix',
                lambda value: _matrix(value, 'control_matrix', rows=dimension),
            )
        ),
    )


def _build_ekf(
    initial_state,
    process_function,
    process_jacobian,
    process_covariance,
    measurement_function=None,
    measurement_jacobian=None,
    state_constraint=None,
):
    dimension = _initial_dimension(initial_state)
    if (measurement_function is None) != (measurement_jacobian is None):
        raise ValueError('measurement_function and measurement_jacobian must be supplied together')
    _callable(measurement_function, 'measurement_function', optional=True)
    _callable(measurement_jacobian, 'measurement_jacobian', optional=True)
    _callable(state_constraint, 'state_constraint', optional=True)
    return ExtendedKalmanFilter(
        initial_state=initial_state,
        process_function=_provider(
            process_function,
            'process_function',
            lambda value: _finite_array(value, 'process_function', (dimension,)),
        ),
        process_jacobian=_provider(
            process_jacobian,
            'process_jacobian',
            lambda value: _finite_array(value, 'process_jacobian', (dimension, dimension)),
        ),
        process_covariance=_provider(
            process_covariance,
            'process_covariance',
            lambda value: _covariance(value, 'process_covariance', dimension),
        ),
        measurement_function=measurement_function,
        measurement_jacobian=measurement_jacobian,
        state_constraint=state_constraint,
    )


# Explicit trusted declarations, without duplicated owner configuration defaults.
COMPONENTS = (
    ModelRegistration(
        identifier='kf',
        builder=_build_kf,
        schema={'type': 'object', 'properties': {}, 'additionalProperties': False},
        dependencies=(
            'initial_state',
            'state_transition',
            'process_covariance',
            'measurement_matrix',
            'control_matrix',
        ),
        capabilities={
            'process_kind': 'linear',
            'dynamic_measurements': False,
            'state_constraints': False,
            'deployable': False,
            'runtime_validation': 'model outputs validated when evaluated; adapter required',
        },
        description='Generic linear Kalman filter; requires compatible model bindings',
    ),
    ModelRegistration(
        identifier='ekf',
        builder=_build_ekf,
        schema={'type': 'object', 'properties': {}, 'additionalProperties': False},
        dependencies=(
            'initial_state',
            'process_function',
            'process_jacobian',
            'process_covariance',
            'measurement_function',
            'measurement_jacobian',
            'state_constraint',
        ),
        capabilities={
            'process_kind': 'nonlinear',
            'dynamic_measurements': True,
            'state_constraints': True,
            'deployable': False,
            'runtime_validation': 'model outputs validated when evaluated; adapter required',
        },
        description='Generic extended Kalman filter; requires process and measurement Jacobians',
    ),
)


def create_algorithm_registry(configuration=None):
    """
    Build a fresh registry from owner defaults and trusted filter declarations.

    ``configuration`` may be the full already loaded owner mapping, avoiding
    circular loading while composing role adapters. The default owner path is
    resolved lazily. YAML never controls a module namespace or import path.
    """
    if configuration is None:
        from robot_configuration.yaml_loader import load_yaml
        from robot_estimation.configuration import estimation_configuration_path

        configuration = load_yaml(estimation_configuration_path())
    if not isinstance(configuration, Mapping) or not isinstance(
        configuration.get('estimators'), Mapping
    ):
        raise ValueError('estimators: expected owner algorithm parameter groups')
    registry = ComponentRegistry()
    register_models(registry, COMPONENTS, configuration['estimators'])
    discover_models(registry, 'robot_estimation.filters', configuration['estimators'])
    return registry
