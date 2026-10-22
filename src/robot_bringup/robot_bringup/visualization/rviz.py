"""Bind an installed RViz template to the read-only display contract."""
from copy import deepcopy

from robot_bringup.visualization.configuration import validate_configuration


def configured_rviz(template, configuration):
    """Match TF prefixes/topics without altering the installed template."""
    config = validate_configuration(configuration)
    result = deepcopy(template)
    manager = result['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = config['frame_id']
    for display in manager['Displays']:
        kind = display['Class'].rsplit('/', 1)[-1]
        if kind == 'TF':
            display['Enabled'] = config['enabled'] and config['show_all_tf']
        elif kind == 'RobotModel':
            # RViz inserts its own '/' between prefix and URDF link names.
            display['TF Prefix'] = config['model_frame_prefix'].rstrip('/')
            display['Enabled'] = config['enabled']
        elif kind == 'MarkerArray':
            display['Marker Topic']['Value'] = config['topic']
            display['Enabled'] = config['enabled']
        elif kind == 'Path':
            layer = display['Name'].lower().replace(' ', '_')
            display['Topic']['Value'] = config['paths_topic'] + '/' + layer
            display['Enabled'] = config['enabled'] and config['layers'][layer]
            color_key = {'target_truth': 'truth', 'target_estimate': 'estimate',
                         'target_prediction': 'prediction'}.get(layer, layer)
            rgba = config['colors'][color_key]
            display['Color'] = '; '.join(str(round(value * 255)) for value in rgba[:3])
            display['Alpha'] = rgba[3]
            # RViz only applies metric Line Width to billboard paths.
            display['Line Style'] = 'Billboards'
            display['Line Width'] = config['path_widths'][layer]
    view = manager['Views']['Current']
    for key in ('distance', 'pitch', 'yaw'):
        view[key.title()] = config['view'][key]
    view['Focal Point'] = dict(zip(('X', 'Y', 'Z'), config['view']['focal_point']))
    return result
