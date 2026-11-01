"""
Project owner defaults into the existing merged ROS parameter boundary.

Canonical files separate estimator mathematics, transport, timing and optional
profiles. Flat/grouped historical ROS snapshots remain readable here; consumers
do not need a second set of default values or a second model-selection policy.
"""

from copy import deepcopy

from robot_configuration.parameter_validation import validate_parameters
from robot_configuration.resources import package_config_path
from robot_configuration.yaml_loader import load_yaml
from robot_configuration.yaml_loader import load_yaml_group
from robot_estimation.parameter_schema import OPTIONAL_PARAMETERS
from robot_estimation.parameter_schema import PARAMETERS


def estimation_configuration_path():
    """Locate canonical owner data in source and copied installations."""
    return package_config_path(__file__, 'robot_estimation', 'estimation.yaml')


# Historical selectors are explicit compatibility contracts, never package selections.
LEGACY_ESTIMATORS = {
    'robot_ekf': {'algorithm': 'ekf', 'binding': 'robot_state'},
    'kalman': {'algorithm': 'kf', 'binding': 'target_state'},
}

# Parameter ownership for bounded migration of pre-Task-E owner documents.
_PROCESS_KEYS = {
    'robot_motion': (
        'joint_count',
        'linear_acceleration_noise_std',
        'angular_acceleration_noise_std',
        'joint_acceleration_noise_std',
        'planar_slip_noise_std',
        'yaw_slip_noise_std',
    ),
    'target_cv': ('target_spatial_dimension', 'target_acceleration_noise_std'),
}
_MEASUREMENT_KEYS = {
    'robot_sensors': (
        'wheel_linear_velocity_variance',
        'wheel_angular_velocity_variance',
        'imu_yaw_rate_variance',
        'joint_position_variance',
        'joint_velocity_variance',
    ),
    'target_position': ('target_position_variance',),
}


def canonical_estimation(data):
    """Read current owner data or translate the bounded historical owner representation."""
    data = deepcopy(data)
    if 'robot_estimators' not in data:
        required = {
            'schema_version',
            'estimators',
            'process_models',
            'measurement_models',
            'initialization',
            'deployment_adapters',
            'bindings',
            'transport',
            'runtime',
            'profiles',
        }
        if (
            set(data) != required
            or type(data['schema_version']) is not int
            or data['schema_version'] != 1
        ):
            raise ValueError('canonical estimation: invalid sections or schema_version')
        return data
    expected = {'robot_estimators', 'target_estimators', 'transport', 'runtime', 'profiles'}
    if set(data) != expected:
        raise ValueError('historical canonical estimation: invalid sections')
    result = {key: data[key] for key in ('transport', 'runtime', 'profiles')}
    result.update(
        schema_version=1,
        estimators={'kf': {'parameters': {}}, 'ekf': {'parameters': {}}},
        process_models={},
        measurement_models={},
        initialization={},
        deployment_adapters={},
        bindings={},
    )
    for role, legacy, process, measurement in [
        ('robot', 'robot_ekf', 'robot_motion', 'robot_sensors'),
        ('target', 'kalman', 'target_cv', 'target_position'),
    ]:
        owner = data[role + '_estimators'][legacy]
        remaining = deepcopy(owner['parameters'])
        optional = deepcopy(owner.get('optional_parameters', {}))
        for section, name, keys in [
            ('process_models', process, _PROCESS_KEYS[process]),
            ('measurement_models', measurement, _MEASUREMENT_KEYS[measurement]),
        ]:
            group = {'parameters': {k: remaining.pop(k) for k in keys if k in remaining}}
            extra = {k: optional.pop(k) for k in keys if k in optional}
            if extra:
                group['optional_parameters'] = extra
            result[section][name] = group
        initial_keys = (role + '_initial_state', role + '_initial_covariance_diagonal')
        name = LEGACY_ESTIMATORS[legacy]['binding']
        result['initialization'][name] = {
            'parameters': {k: remaining.pop(k) for k in initial_keys}
        }
        result['deployment_adapters'][name] = {'parameters': remaining}
        if optional:
            result['deployment_adapters'][name]['optional_parameters'] = optional
        result['bindings'][name] = {
            'role': role,
            'process_model': process,
            'measurement_model': measurement,
            'initialization': name,
            'adapter': name,
        }
    return result


def binding_parameters(data, name, *, include_optional=False):
    """Flatten one explicitly selected, algorithm-free deployment resource."""
    try:
        binding = data['bindings'][name]
    except KeyError as error:
        raise ValueError(f'unknown estimation binding {name!r}') from error
    expected = {'role', 'process_model', 'measurement_model', 'initialization', 'adapter'}
    if not isinstance(binding, dict) or set(binding) != expected:
        raise ValueError(f'bindings.{name}: requires {sorted(expected)}')
    parameters = {}
    for section, key in [
        ('process_models', 'process_model'),
        ('measurement_models', 'measurement_model'),
        ('initialization', 'initialization'),
        ('deployment_adapters', 'adapter'),
    ]:
        try:
            group = data[section][binding[key]]
        except (KeyError, TypeError) as error:
            raise ValueError(f'bindings.{name}.{key}: unknown model/resource') from error
        if not isinstance(group, dict) or set(group) - {'parameters', 'optional_parameters'}:
            raise ValueError(f'{section}.{binding[key]}: invalid parameter group')
        values = deepcopy(group['parameters'])
        if include_optional:
            values.update(deepcopy(group.get('optional_parameters', {})))
        overlap = parameters.keys() & values.keys()
        if overlap:
            raise ValueError(f'bindings.{name}: competing parameter owners {sorted(overlap)}')
        parameters.update(values)
    return parameters


def role_binding(data, role):
    """Resolve the unique existing deployment resource for a ROS output contract."""
    names = [name for name, value in data['bindings'].items() if value.get('role') == role]
    if len(names) != 1:
        raise ValueError(f'{role}: requires one explicit deployment binding resource')
    binding_parameters(data, names[0])
    return names[0], deepcopy(data['bindings'][names[0]])


def load_estimation_parameters(path=None, group=None):
    """Project named bindings/profiles; historical owner and flat ROS snapshots remain readable."""
    path = path or estimation_configuration_path()
    data = load_yaml(path)
    if 'robot_estimators' not in data and 'estimators' not in data:
        return load_yaml_group(path, group)
    data = canonical_estimation(data)
    if isinstance(group, list):
        if (
            not group
            or any(not isinstance(n, str) or not n for n in group)
            or len(group) != len(set(group))
        ):
            raise ValueError(f'{path}: group list must contain unique nonempty names')
        parameters = {}
        for name in group:
            parameters.update(
                load_estimation_parameters(path, name)['estimation_node']['ros__parameters']
            )
        return {'estimation_node': {'ros__parameters': parameters}}
    if group is None:
        raise ValueError(f'{path}: canonical estimation requires an explicit parameter group')
    name = LEGACY_ESTIMATORS.get(group, {}).get('binding', group)
    if name in data['bindings']:
        role = data['bindings'][name]['role']
        parameters = binding_parameters(data, name)
        parameters.update(data['transport'][role])
        parameters.update(data['runtime'][role])
    elif group in data['profiles']:
        profile = data['profiles'][group]
        parameters = {
            **profile.get('robot_parameters', {}),
            **profile.get('target_parameters', {}),
            **profile.get('transport', {}),
            **profile.get('runtime', {}),
        }
    else:
        raise ValueError(f'{path}: unknown estimation parameter group {group!r}')
    parameters = validate_parameters(
        parameters, {**PARAMETERS, **OPTIONAL_PARAMETERS}, path=str(path)
    )
    return {'estimation_node': {'ros__parameters': parameters}}
