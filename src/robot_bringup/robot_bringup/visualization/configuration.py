"""Strict flat visualization configuration, independent of ROS and robot geometry."""

import re
from copy import deepcopy
from numbers import Real

import numpy as np


LAYERS = ('target_truth', 'target_estimate', 'target_prediction', 'robot_truth',
          'robot_estimate', 'camera_axis', 'line_of_sight', 'tracking_radius', 'joint_limits',
          'target_truth_current', 'target_estimate_current')

DEFAULTS = {
    'schema_version': 1, 'enabled': True, 'frame_id': 'world', 'topic': '/tracking/markers',
    'target_topic': '/simulation/target_position', 'paths_topic': '/tracking/paths',
    'model_frame_prefix': 'visual/', 'rate_hz': 5., 'max_age': .2, 'max_skew': .08,
    'future_tolerance': .01, 'lifetime': .3, 'axis_length': .3,
    'forward_length': .8, 'width': .012, 'origin_size': .035,
    'history_seconds': 8., 'max_points': 500, 'prediction_max_age': .3,
    'history_max_gap': .5, 'history_max_step': 1., 'show_all_tf': False,
    'path_widths': {'target_truth': .025, 'target_estimate': .015,
                    'target_prediction': .035, 'robot_truth': .012, 'robot_estimate': .012},
    'point_sizes': {'truth': .065, 'estimate': .085},
    'overlay_widths': {'line_of_sight': .008, 'tracking_radius': .004},
    'view': {'distance': 3.8, 'pitch': .6, 'yaw': .8, 'focal_point': [1., 0., .6]},
    'layers': {key: key not in ('robot_truth', 'robot_estimate') for key in LAYERS},
    'colors': {
        'origin': [1., 1., 1., 1.], 'x': [1., 0., 0., 1.],
        'y': [0., 1., 0., 1.], 'z': [0., 0., 1., 1.],
        'optical': [0., 1., 1., 1.], 'target': [1., 0., 1., .8],
        'truth': [0., 1., 0., 1.], 'estimate': [1., .5, 0., 1.],
        'prediction': [0., .35, 1., 1.], 'robot_truth': [.6, .6, .6, 1.],
        'robot_estimate': [1., .5, 0., 1.], 'safety': [1., 0., 0., 1.],
        'radius': [0., 1., 1., .25],
    },
}


def validate_configuration(configuration):
    """Return complete independent styling defaults; reject unknown or invalid fields."""
    if not isinstance(configuration, dict):
        raise ValueError('Visualization configuration must be a mapping')
    unknown = set(configuration) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown visualization keys: {sorted(unknown, key=str)}')
    result = deepcopy(DEFAULTS)
    for key, value in configuration.items():
        if isinstance(DEFAULTS[key], dict):
            if not isinstance(value, dict) or set(value) - set(DEFAULTS[key]):
                raise ValueError(f'Invalid visualization {key} keys')
            result[key].update(deepcopy(value))
        else:
            result[key] = deepcopy(value)
    if type(result['schema_version']) is not int or result['schema_version'] != 1:
        raise ValueError('Visualization schema_version must be 1')
    if type(result['enabled']) is not bool:
        raise ValueError('Visualization enabled must be boolean')
    if type(result['show_all_tf']) is not bool:
        raise ValueError('Visualization show_all_tf must be boolean')
    if type(result['max_points']) is not int or result['max_points'] < 1:
        raise ValueError('Visualization max_points must be a positive integer')
    for key in ('rate_hz', 'max_age', 'max_skew', 'lifetime', 'axis_length',
                'forward_length', 'width', 'origin_size', 'history_seconds',
                'prediction_max_age', 'future_tolerance', 'history_max_gap', 'history_max_step'):
        value = result[key]
        if (isinstance(value, bool) or not isinstance(value, Real) or
                not np.isfinite(value) or value < 0 or
                (key != 'future_tolerance' and value == 0)):
            raise ValueError(f'Visualization {key} must be finite and positive')
        result[key] = float(value)
    for group in ('path_widths', 'point_sizes', 'overlay_widths', 'view'):
        for key, value in result[group].items():
            if key == 'focal_point':
                if (not isinstance(value, (list, tuple)) or len(value) != 3 or
                        any(isinstance(x, bool) or not isinstance(x, Real) or
                            not np.isfinite(x) for x in value)):
                    raise ValueError('Visualization view focal_point must be a finite 3-vector')
                result[group][key] = [float(x) for x in value]
            else:
                if (isinstance(value, bool) or not isinstance(value, Real) or
                        not np.isfinite(value) or
                        (key not in ('pitch', 'yaw') and value <= 0)):
                    raise ValueError(f'Invalid visualization {group}.{key}')
                result[group][key] = float(value)
    for key in ('frame_id', 'model_frame_prefix'):
        value = result[key]
        if (not isinstance(value, str) or
                not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', value) or '//' in value or
                (key == 'frame_id' and value.endswith('/'))):
            raise ValueError(f'Visualization {key} must be a nonempty relative frame name')
    if not result['model_frame_prefix'].endswith('/'):
        raise ValueError('Visualization model_frame_prefix must end in /')
    for key in ('topic', 'target_topic', 'paths_topic'):
        value = result[key]
        if (not isinstance(value, str) or not re.fullmatch(r'/?[A-Za-z_][A-Za-z0-9_/]*', value)
                or value.endswith('/') or '//' in value):
            raise ValueError(f'Visualization {key} must be a topic name')
    for key, value in result['layers'].items():
        if type(value) is not bool:
            raise ValueError(f'Visualization layer {key} must be boolean')
    for key, value in result['colors'].items():
        if (not isinstance(value, (list, tuple)) or len(value) != 4 or
                any(isinstance(x, bool) or not isinstance(x, Real) for x in value)):
            raise ValueError(f'Visualization color {key} must be RGBA')
        rgba = np.asarray(value, dtype=float)
        if not np.all(np.isfinite(rgba)) or np.any(rgba < 0) or np.any(rgba > 1):
            raise ValueError(f'Visualization color {key} channels must lie in [0,1]')
        result['colors'][key] = rgba.tolist()
    return result
