"""Estimator discovery, strict contracts, and the legacy transport boundary."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_estimation.factory import estimator_factory as factory


CONFIG = Path(__file__).parents[1] / 'config' / 'estimation.yaml'


def test_canonical_configuration_and_group_adapter_preserve_values(tmp_path):
    from robot_estimation.configuration import load_estimation_parameters
    document = yaml.safe_load(CONFIG.read_text())
    assert 'robot_estimators' in document and 'transport' in document
    robot = load_estimation_parameters(CONFIG, 'robot_ekf')['estimation_node']['ros__parameters']
    assert robot['linear_acceleration_noise_std'] == .1
    assert robot['wheel_odometry_topic'] == '/wheel/odometry'
    assert robot['robot_initial_state'] == [0.] * 13
    assert 'wheel_odometry_topic' not in document['robot_estimators']['robot_ekf']['parameters']
    aided = load_estimation_parameters(CONFIG, ['robot_ekf', 'pose_aiding'])['estimation_node']['ros__parameters']
    assert aided['reorder_holdback'] == .08 and aided['linear_acceleration_noise_std'] == .3
    legacy = tmp_path / 'legacy.yaml'
    legacy.write_text(yaml.safe_dump({'estimation_node': {'ros__parameters': robot}}))
    assert load_estimation_parameters(legacy) == {'estimation_node': {'ros__parameters': robot}}
    robot['robot_initial_state'][0] = 99.
    assert load_estimation_parameters(CONFIG, 'robot_ekf')['estimation_node']['ros__parameters']['robot_initial_state'][0] == 0.


def test_discovery_defaults_isolation_and_numerical_construction():
    registries = factory.create_estimation_registries()
    registry = registries['target_estimator']
    assert registry.names() == ('kalman',)
    info = registry.describe('kalman')
    assert info['capabilities']['state_layout'] == 'position_velocity'
    assert info['schema']['properties']['target_acceleration_noise_std']['unit'] == 'm/s^2'
    info['defaults']['target_initial_state'][0] = 99.
    first = registry.create('kalman')
    first.state.mean[0] = 99.
    second = registry.create('kalman')
    np.testing.assert_array_equal(second.state.mean, np.zeros(6))
    np.testing.assert_array_equal(second.state.covariance, np.eye(6))


@pytest.mark.parametrize('parameters', [
    {'unknown': 1.}, {'target_acceleration_noise_std': -1.},
    {'target_acceleration_noise_std': float('nan')},
    {'target_spatial_dimension': True}, {'target_spatial_dimension': 0},
    {'target_initial_state': [0.]}, {'target_initial_covariance_diagonal': [-1.] * 6},
    {'target_position_variance': [0.1]},
])
def test_target_rejects_invalid_component_parameters(parameters):
    with pytest.raises((ValueError, TypeError)):
        factory.create_estimation_registries()['target_estimator'].create('kalman', parameters)


def test_arbitrary_library_dimension_and_deployment_rejection():
    values = {'target_spatial_dimension': 2, 'target_initial_state': [0.] * 4,
              'target_initial_covariance_diagonal': [1.] * 4,
              'target_position_variance': [.01] * 2}
    registry = factory.create_estimation_registries()['target_estimator']
    assert registry.create('kalman', values).state.dimension == 4
    assert not registry.compatibility('kalman', {'target_measurement_adapter': 'bearing'})['compatible']
    with pytest.raises(ValueError, match='dimension'):
        factory.create_target_estimator(values, context={
            'target_state_layout': 'position_velocity',
            'target_measurement_adapter': 'position', 'spatial_dimension': 3})


def test_missing_physical_description_is_informative():
    with pytest.raises(ValueError, match='physical|description'):
        factory.create_robot_estimator({})


def test_test_only_estimator_registers_and_constructs_through_public_factory():
    from robot_estimation.interfaces.estimator_interface import EstimatorInterface
    from robot_estimation.state import StateEstimate

    class ExampleEstimator(EstimatorInterface):
        def __init__(self, offset):
            self._state = StateEstimate(np.array([offset]), np.eye(1))

        @property
        def state(self):
            return self._state

        def predict(self, dt, control=None):
            return self.state

        def update(self, measurement):
            return self.state

    registry = factory.create_estimation_registries()['target_estimator']
    registry.register('test_only_offset', ExampleEstimator, defaults={'offset': 2.},
                      schema={'type': 'object', 'additionalProperties': False,
                              'properties': {'offset': {'type': 'number'}},
                              'required': ['offset']},
                      capabilities={'deployable': False, 'test_only': True})
    assert 'test_only_offset' in registry.names()
    estimate = factory.create_target_estimator({'offset': 3.}, 'test_only_offset', registry=registry)
    np.testing.assert_array_equal(estimate.state.mean, [3.])
    assert registry.describe('test_only_offset')['capabilities']['deployable'] is False
    with pytest.raises(ValueError, match='unknown'):
        factory.create_target_estimator({'offset': 3., 'unexpected': 0.}, 'test_only_offset', registry=registry)


def test_public_factories_reject_registrations_without_domain_contracts():
    from robot_configuration.components import ComponentRegistry, schema_for
    registry = ComponentRegistry()
    registry.register('invalid', lambda: object(), schema=schema_for({}))
    with pytest.raises(TypeError, match='EstimatorInterface'):
        factory.create_target_estimator({}, 'invalid', registry=registry)
    with pytest.raises(TypeError, match='robot|Robot'):
        factory.create_robot_estimator({}, estimator_type='invalid', registry=registry)


def test_custom_target_does_not_need_builtin_parameter_names_in_deployment_context():
    from robot_configuration.components import ComponentRegistry, schema_for
    from robot_estimation.interfaces.estimator_interface import EstimatorInterface
    from robot_estimation.state import StateEstimate

    class CustomTarget(EstimatorInterface):
        def __init__(self):
            self._state = StateEstimate(np.zeros(6), np.eye(6))

        @property
        def state(self):
            return self._state

        def predict(self, dt, control=None):
            return self.state

        def update(self, measurement):
            return self.state

    registry = ComponentRegistry()
    registry.register('custom', CustomTarget, schema=schema_for({}),
                      requirements={'target_state_layout': 'position_velocity'})
    result = factory.create_target_estimator({}, 'custom', registry=registry, context={
        'target_state_layout': 'position_velocity', 'spatial_dimension': 3})
    assert result.state.dimension == 6


def test_custom_robot_can_use_the_sensor_contract_without_physical_dependency():
    from robot_configuration.components import ComponentRegistry, schema_for

    def build():
        return factory.create_robot_estimator({'joint_bounds_enabled': False})

    registry = ComponentRegistry()
    registry.register('custom', build, schema=schema_for({}))
    result, constraint = factory.create_robot_estimator({}, estimator_type='custom', registry=registry)
    assert result.state.dimension == 13 and constraint is None
