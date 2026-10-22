"""Validate the current numerical/Gazebo deployment's configuration contracts."""

import numpy as np

from robot_simulation.factory.physics_configuration import validate_physics_engine

from robot_simulation.measurements.synthetic_measurement import SyntheticMeasurementGenerator


def _keys(value, allowed, path):
    if not isinstance(value, dict) or set(value) - set(allowed):
        raise ValueError(f'{path}: expected mapping with supported keys {sorted(allowed)}')


def _positive(value, path):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
        raise ValueError(f'{path}: expected finite positive number, got {value!r}')


def validate_deployment_configuration(deployment, target, environment, simulation, measurements):
    """Fail before startup for ignored keys and unsupported six-state target wiring."""
    layout = {
        'paths': {'base'},
        'world': {'source', 'terrain_config', 'output_name', 'physics', 'system_plugins'},
        'target': {'config', 'entity_name', 'initial_position', 'initial_velocity'},
        'robot': {'enabled', 'config', 'joint_position_hold'},
        'tracking': {'source', 'position', 'reference_frame', 'tracking_frame', 'max_age', 'future_tolerance', 'pose_source'},
        'simulation': {'config', 'environment_config', 'measurement_config'},
        'gazebo': {'gui', 'start_running', 'clock_topic', 'pose_service_template', 'pose_service_type'},
    }
    _keys(deployment, set(layout) | {'physical_context', 'localization_sensor_config'}, 'simulation deployment')
    if 'localization_sensor_config' in deployment and (not isinstance(deployment['localization_sensor_config'], str) or not deployment['localization_sensor_config'].strip()):
        raise ValueError('localization_sensor_config must be an explicit resource path')
    for key, allowed in layout.items():
        _keys(deployment[key], allowed, 'simulation.' + key)
    _keys(deployment['world'].get('physics', {}), {'max_step_size', 'real_time_factor', 'engine'}, 'world.physics')
    for key, value in deployment['world'].get('physics', {}).items():
        if key == 'engine':
            validate_physics_engine(value)
        else:
            _positive(value, 'world.physics.' + key)
    for key in ['initial_position', 'initial_velocity']:
        array = np.asarray(deployment['target'][key], dtype=float)
        if array.shape != (3,) or not np.all(np.isfinite(array)):
            raise ValueError(f'target.{key}: requires finite position/velocity 3-vector')
    if deployment['tracking']['source'] not in {'tf', 'static'}:
        raise ValueError('tracking.source: supported tf, static')
    _positive(deployment['tracking']['max_age'], 'tracking.max_age')
    pose_source = deployment['tracking'].get('pose_source', 'wheel_odometry')
    if pose_source not in {'wheel_odometry', 'physical'}:
        raise ValueError('tracking.pose_source: supported wheel_odometry, physical')
    needs_context = any(f.get('enabled', True) and f.get('type') == 'base_relative_guidance'
                        for f in environment['environment'].get('acceleration_fields', []))
    if needs_context:
        physical = deployment.get('physical_context')
        _keys(physical, {'base_topic', 'reference_frame', 'max_age'}, 'physical_context')
        if set(physical) != {'base_topic', 'reference_frame', 'max_age'}:
            raise ValueError('physical_context requires base_topic/reference_frame/max_age')
        if (not isinstance(physical['base_topic'], str) or not physical['base_topic'].strip()
                or physical['reference_frame'] != deployment['tracking']['reference_frame']
                or pose_source != 'physical' or deployment['tracking']['source'] != 'tf'):
            raise ValueError('guidance requires physical tracking TF in its reference frame')
        _positive(physical['max_age'], 'physical_context.max_age')
        for field in environment['environment']['acceleration_fields']:
            if field.get('type') == 'base_relative_guidance' and field.get('enabled', True):
                if field['parameters']['reference_frame'] != physical['reference_frame']:
                    raise ValueError('guidance/context reference frames must agree')
    elif 'physical_context' in deployment:
        raise ValueError('physical_context configured without a contextual field')

    _keys(target, {'target'}, 'target document')
    _keys(target['target'], {'geometry', 'appearance', 'physics', 'dynamics'}, 'target')
    _keys(target['target']['dynamics'], {'type', 'parameters'}, 'target.dynamics')
    _keys(environment, {'environment'}, 'environment document')
    _keys(environment['environment'], {'acceleration_fields', 'stochastic_disturbances'}, 'environment')
    if environment['environment'].get('stochastic_disturbances'):
        raise ValueError('environment.stochastic_disturbances: no deployed implementation; use acceleration_fields')
    for field in environment['environment'].get('acceleration_fields', []):
        _keys(field, {'type', 'parameters', 'enabled'}, 'environment.acceleration_fields')
        if 'enabled' in field and type(field['enabled']) is not bool:
            raise ValueError('environment.acceleration_fields.enabled: expected boolean')
    _keys(simulation, {'simulation'}, 'integration document')
    _keys(simulation['simulation'], {'backend', 'integration', 'ground_truth'}, 'simulation')
    integration = simulation['simulation']['integration']
    _keys(integration, {'type', 'parameters', 'max_steps'}, 'simulation.integration')
    if type(integration['max_steps']) is not int or integration['max_steps'] <= 0:
        raise ValueError('simulation.integration.max_steps: expected positive integer')
    _keys(measurements, {'measurements'}, 'measurements document')
    _keys(measurements['measurements'], {'target_position'}, 'measurements')
    sensor = measurements['measurements']['target_position']
    _keys(sensor, {'enabled', 'topic', 'observation', 'noise', 'random_seed', 'rate_hz'}, 'measurements.target_position')
    _keys(sensor['observation'], {'components'}, 'measurements.observation')
    _keys(sensor['noise'], {'covariance'}, 'measurements.noise')
    _positive(sensor['rate_hz'], 'measurements.rate_hz')
    if type(sensor['enabled']) is not bool:
        raise ValueError('measurements.enabled: expected boolean')
    # Reuse the real observation constructor's shape/symmetry/PSD validation.
    SyntheticMeasurementGenerator(np.eye(3, 6), np.asarray(sensor['noise']['covariance']),
                                  np.random.default_rng(sensor['random_seed']))
