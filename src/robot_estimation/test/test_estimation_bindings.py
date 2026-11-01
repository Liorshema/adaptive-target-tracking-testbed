"""Canonical algorithms require actual compatible model and deployment bindings."""

from pathlib import Path

import numpy as np
import pytest
from robot_configuration.yaml_loader import load_yaml
from robot_estimation.configuration import load_estimation_parameters
from robot_estimation.factory.estimator_factory import (
    create_estimation_registries,
    create_target_estimator,
)

CONFIG = Path(__file__).parents[1] / 'config/estimation.yaml'


def test_owner_sections_and_legacy_projection_have_one_numerical_owner():
    data = load_yaml(CONFIG)
    assert set(data['estimators']) == {'kf', 'ekf'}
    assert all(x['parameters'] == {} for x in data['estimators'].values())
    assert not {'robot_estimators', 'target_estimators'} & data.keys()
    robot = load_estimation_parameters(CONFIG, 'robot_state')
    target = load_estimation_parameters(CONFIG, 'target_state')
    assert robot == load_estimation_parameters(CONFIG, 'robot_ekf')
    assert target == load_estimation_parameters(CONFIG, 'kalman')
    assert robot['estimation_node']['ros__parameters']['linear_acceleration_noise_std'] == 0.1
    assert target['estimation_node']['ros__parameters']['target_acceleration_noise_std'] == 0.1


def test_canonical_selection_constructs_shared_algorithm_with_legacy_meaning():
    registries = create_estimation_registries()
    assert {'kf', 'kalman'} <= set(registries['target_estimator'].names())
    assert {'ekf', 'robot_ekf'} <= set(registries['robot_estimator'].names())
    first = create_target_estimator({}, 'kf')
    second = create_target_estimator({}, 'kalman')
    assert type(first._filter) is type(second._filter)
    assert registries['target_estimator'].describe('kalman')['capabilities']['alias_of'] == 'kf'
    np.testing.assert_array_equal(first.predict(0.1).covariance, second.predict(0.1).covariance)


@pytest.mark.parametrize(
    'changes',
    [
        {'target_process_model': 'missing'},
        {'target_measurement_model': 'robot_sensors'},
        {'target_process_model': 'robot_motion'},
        {'target_deployment_adapter': 'robot_state'},
        {'target_initial_state': [0.0] * 5},
        {'target_position_variance': [float('nan')] * 3},
    ],
)
def test_invalid_model_adapter_bindings_reject(changes):
    with pytest.raises((ValueError, TypeError, KeyError)):
        create_target_estimator(changes, 'kf')


def test_unknown_and_unsupported_role_algorithms_fail():
    for name in ['unknown', 'ekf']:
        with pytest.raises((ValueError, KeyError)):
            create_target_estimator({}, name)


def test_missing_binding_model_rejects_before_construction(tmp_path):
    data = load_yaml(CONFIG)
    del data['bindings']['target_state']['process_model']
    path = tmp_path / 'missing.yaml'
    import yaml

    path.write_text(yaml.safe_dump(data))
    with pytest.raises((ValueError, KeyError), match='process_model|binding'):
        create_estimation_registries(path)


@pytest.mark.parametrize(
    'identifier,key',
    [
        ('robot_motion', 'linear_acceleration_noise_std'),
        ('target_cv', 'target_acceleration_noise_std'),
    ],
)
def test_model_registration_preserves_legacy_float_noise_coercion(identifier, key):
    # A valid integer noise value may square beyond NumPy int64. Old owner factories
    # converted these three robot/one target amplitudes to float before model construction.
    registry = create_estimation_registries()['estimation_process']
    model = registry.create(identifier, {key: 10_000_000_000})
    covariance = model.process_covariance(0.1)
    assert covariance.dtype.kind == 'f'
    assert np.isfinite(covariance).all()


def _parameter_node(overrides):
    from types import SimpleNamespace
    from rclpy.parameter import Parameter
    from robot_estimation.nodes.estimation_node import EstimationNode

    parameters = {}
    for group in ['robot_state', 'target_state']:
        parameters.update(
            load_estimation_parameters(CONFIG, group)['estimation_node']['ros__parameters']
        )
    parameters.update(robot_estimator_type='ekf', target_estimator_type='kf', **overrides)
    node = object.__new__(EstimationNode)
    node.declared = []

    def declare(name, default):
        node.declared.append(name)
        if name not in parameters:
            parameters[name] = None if isinstance(default, Parameter.Type) else default
        return SimpleNamespace(value=parameters[name])

    node.declare_parameter = declare
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node._robot_buffer = SimpleNamespace(reset=lambda: None)
    node.get_logger = lambda: SimpleNamespace(warning=lambda text: None)
    return node


def test_ros_boundary_declares_and_forwards_explicit_model_binding():
    node = _parameter_node({'target_process_model': 'missing'})
    node._declare_parameters()
    assert 'target_process_model' in node.declared
    assert node._model_bindings == {'target_process_model': 'missing'}
    with pytest.raises((ValueError, KeyError), match='missing'):
        node._create_target_estimator()


def test_clock_reset_reconstruction_revalidates_explicit_binding():
    node = _parameter_node({'target_deployment_adapter': 'robot_state'})
    node._declare_parameters()
    with pytest.raises((ValueError, KeyError), match='adapter|incompatible'):
        node._on_time_jump(None)


def test_intrinsic_algorithm_settings_use_frozen_role_transport():
    from robot_configuration.components import schema_for
    from robot_estimation.factory.algorithm_factory import create_algorithm_registry

    algorithms = create_algorithm_registry()
    metadata = algorithms.describe('kf')
    from robot_estimation.factory.algorithm_factory import COMPONENTS

    original = next(item.builder for item in COMPONENTS if item.identifier == 'kf')

    def configured(tuning, **dependencies):
        result = original(**dependencies)
        result.tuning = tuning
        return result

    algorithms.register(
        'configured_filter',
        configured,
        defaults={'tuning': 1.0},
        schema=schema_for({'tuning': 1.0}),
        dependencies=metadata['dependencies'],
        capabilities=metadata['capabilities'],
    )
    role = create_estimation_registries(algorithms=algorithms)['target_estimator']
    assert role.describe('configured_filter')['capabilities']['deployable']
    assert role.create('configured_filter')._filter.tuning == 1.0
    with pytest.raises(ValueError, match='frozen'):
        create_target_estimator({}, 'configured_filter', registry=role, require_frozen=True)
    from robot_estimation.factory.algorithm_parameters import encode_algorithm_parameters

    frozen = {'target_algorithm_parameters': encode_algorithm_parameters(
        'configured_filter', {'tuning': 0.25})}
    actual = create_target_estimator(
        frozen, 'configured_filter', registry=role, require_frozen=True)
    assert actual._filter.tuning == 0.25
