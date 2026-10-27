"""Render canonical controller/task defaults at the existing ROS/strategy boundary."""

from copy import deepcopy

from robot_configuration.resources import package_config_path
from robot_configuration.yaml_loader import load_yaml
from robot_configuration.yaml_loader import load_yaml_group


def control_configuration_path():
    """Locate the single owner resource in source or copied installation."""
    return package_config_path(__file__, 'robot_control', 'control.yaml')


def load_control_parameters(path=None, group=None):
    """Keep saved ROS/strategy files readable and project selected named defaults."""
    path = path or control_configuration_path()
    data = load_yaml(path)
    if 'controllers' not in data:
        return load_yaml_group(path, group)
    if set(data) != {
        'controllers',
        'tasks',
        'tracking',
        'references',
        'operating_bounds',
        'transport',
        'runtime',
    }:
        raise ValueError(f'{path}: unsupported/missing canonical control sections')
    if group == 'parameters':
        dls = data['controllers']['jacobian']['parameters']
        tracking = data['tracking']['parameters']
        parameters = {
            **data['transport'],
            **data['runtime'],
            **data['operating_bounds'],
            'controller_damping': dls['damping'],
            'controller_task_gains': dls['gains'],
            'controller_task_axes': data['tasks']['cartesian']['parameters']['axes'],
            'tracking_radius': tracking['radius'],
            'tracking_forward_axis': tracking['forward_axis_local'],
            'tracking_up_axis': tracking['up_axis_local'],
            'world_up_axis': tracking['world_up'],
            'tracking_lookahead_time': tracking['lookahead_time'],
        }
        return deepcopy({'control_node': {'ros__parameters': parameters}})
    if group == 'distance_pointing':
        return deepcopy(
            {
                'controller': {
                    'uses_safety_bounds': True,
                    'task': {'type': group, 'parameters': data['tasks'][group]['parameters']},
                    'parameters': data['controllers']['weighted_least_squares']['parameters'],
                },
                'tracking': data['tracking'][group],
            }
        )
    raise ValueError(f'{path}: canonical control requires an explicit supported group, got {
            group!r}')
