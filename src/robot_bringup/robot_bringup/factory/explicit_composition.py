"""Compile schema-2 deployment selection into the existing M07 profile interface."""

from copy import deepcopy

from robot_configuration.parameter_validation import validate_parameters
from robot_configuration.resources import resolve_resource
from robot_configuration.yaml_loader import load_yaml


REFERENCES = ('estimation.calibration', 'simulation.robot', 'simulation.actuation',
              'simulation.localization_sensor')


def compile_profile(manifest, source, share, overrides):
    """Select actual owners without introducing a second launch or validation engine.

    The manifest owns actuation, calibration and TF policy. The simulation owns
    its deployment slots; duplicate robot/localization selectors must name the
    same canonical resources. Existing M07 owner/factory checks still run after
    compilation. A legacy profile override has no meaning in this new schema.
    """
    if overrides.get('config'):
        raise ValueError('schema 2 cannot select a legacy profile with config; '
                         'select simulation.config in the manifest')
    selected = manifest['simulation']
    deployment_path = selected['config']
    deployment = load_yaml(deployment_path)
    required = {'paths', 'world', 'target', 'robot', 'tracking', 'simulation', 'gazebo'}
    if not required <= deployment.keys():
        raise ValueError('simulation.config must select a direct simulation deployment')
    # The existing simulation launch consumes these as strings, not manifest mappings.
    for section, key in [('world', 'source'), ('world', 'terrain_config'),
                         ('target', 'config'), ('robot', 'config'),
                         ('robot', 'joint_position_hold'), ('simulation', 'config'),
                         ('simulation', 'environment_config'),
                         ('simulation', 'measurement_config')]:
        value = deployment[section][key]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'simulation deployment resource {section}.{key} '
                             'must be a string path')
    if 'localization_sensor_config' in deployment:
        value = deployment['localization_sensor_config']
        if not isinstance(value, str) or not value.strip():
            raise ValueError('simulation deployment resource localization_sensor_config '
                             'must be a string path')
    mode = deployment['paths']['base']
    if mode not in {'package', 'config'}:
        raise ValueError('simulation.paths.base: supported package, config')
    base = share('robot_simulation') if mode == 'package' else deployment_path.parent
    deployed_robot = resolve_resource(deployment['robot']['config'], base, share)
    if deployed_robot != selected['robot']:
        raise ValueError('simulation.robot conflicts with simulation deployment robot selector')
    configured_sensor = deployment.get('localization_sensor_config')
    sensor = resolve_resource(configured_sensor, base, share) if configured_sensor else None
    if sensor != selected.get('localization_sensor'):
        raise ValueError(
            'simulation.localization_sensor conflicts with deployment sensor selector')
    keys = {'rate_hz', 'max_age', 'max_skew', 'future_tolerance'}
    tracking = validate_parameters(
        selected['tracking_tf'], dict.fromkeys(keys, 'DOUBLE'), required=keys,
        path='simulation.tracking_tf')
    for key, value in tracking.items():
        if value < 0 or (key in {'rate_hz', 'max_age'} and value == 0):
            raise ValueError(f'simulation.tracking_tf.{key}: invalid finite timing value')
    profile = {
        'simulation_config': str(deployment_path), 'actuation_config': str(selected['actuation']),
        'control_config': manifest['control']['config'],
        'robot_estimator_config': manifest['estimation']['robot']['config'],
        'target_estimator_config': manifest['estimation']['target']['config'],
        'tracking_tf': deepcopy(tracking)}
    calibration = manifest['estimation'].get('calibration')
    if calibration is not None:
        profile['estimation_overrides'] = str(calibration)
    provenance = {
        'schema_version': 2, 'profile_source_kind': 'system_manifest',
        'simulation': deployment_path, 'robot': deployed_robot,
        'actuation': selected['actuation'], 'localization_sensor': sensor,
        'calibration': calibration, 'tracking_tf': deepcopy(tracking)}
    return profile, provenance
