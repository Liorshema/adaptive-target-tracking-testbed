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


def load_estimation_parameters(path=None, group=None):
    """
    Return a detached legacy ROS document for an explicit group or group list.

    ``robot_ekf`` and ``kalman`` select defaults, never the runtime implementation.
    ``pose_aiding`` supplies the existing optional calibration delta. Ordered lists
    merge these deltas; runtime model selection still belongs to the manifest.
    """
    path = path or estimation_configuration_path()
    data = load_yaml(path)
    if 'robot_estimators' not in data:
        return load_yaml_group(path, group)
    expected = {'robot_estimators', 'target_estimators', 'transport', 'runtime', 'profiles'}
    if set(data) != expected:
        raise ValueError(f'{path}: canonical estimation keys must be {sorted(expected)}')
    if isinstance(group, list):
        if (
            not group
            or any(not isinstance(name, str) or not name for name in group)
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
    parameters = None
    for family, role in [('robot_estimators', 'robot'), ('target_estimators', 'target')]:
        if group in data[family]:
            parameters = deepcopy(data[family][group]['parameters'])
            parameters.update(data['transport'][role])
            parameters.update(data['runtime'][role])
            break
    if parameters is None and group in data['profiles']:
        profile = data['profiles'][group]
        parameters = {
            **profile.get('robot_parameters', {}),
            **profile.get('target_parameters', {}),
            **profile.get('transport', {}),
            **profile.get('runtime', {}),
        }
    if parameters is None:
        raise ValueError(f'{path}: unknown estimation parameter group {group!r}')
    parameters = validate_parameters(
        parameters, {**PARAMETERS, **OPTIONAL_PARAMETERS}, path=str(path)
    )
    return {'estimation_node': {'ros__parameters': parameters}}
