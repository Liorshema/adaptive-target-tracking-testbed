"""Build a physical robot with configurable Gazebo actuation and feedback."""

import subprocess
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from copy import deepcopy
from pathlib import Path

import numpy as np

from robot_simulation.factory.config_loader import load_yaml


def _number(value, name, *, positive=False, nonnegative=False):
    """Validate a finite simulation parameter."""
    result = float(value)
    if (not np.isfinite(result) or (positive and result <= 0.0) or
            (nonnegative and result < 0.0)):
        raise ValueError(f'Invalid {name}: {value}')
    return result


def _plugin(model, system, parameters):
    """Add one built-in Gazebo system with explicit parameters."""
    filenames = {
        'JointPositionController': 'gz-sim-joint-position-controller-system',
        'JointStatePublisher': 'gz-sim-joint-state-publisher-system',
        'DiffDrive': 'gz-sim-diff-drive-system',
    }
    plugin = ET.SubElement(model, 'plugin', filename=filenames[system],
                           name=f'gz::sim::systems::{system}')
    for key, value in parameters:
        ET.SubElement(plugin, key).text = str(value)
    return plugin


def configure_robot_model(model, description, config, hold):
    """
    Attach configured effort holding and feedback to an authoritative model.

    Holding targets use robot-description joint keys. Gazebo joint names are
    resolved using the externally configured name template and checked against
    the rendered model. Limits and effort saturation come from robot.yaml.
    JointPositionController.initial_position sets the startup target; joints
    physically converge under PID effort, without modifying link geometry.
    """
    model = deepcopy(model)
    name = config['entity_name']
    if not isinstance(name, str) or not name.strip():
        raise ValueError('Robot entity_name must be nonempty')
    model.set('name', name)
    pose = np.asarray(config['pose'], dtype=float)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise ValueError('Robot pose must contain six finite xyz/rpy values')
    pose_element = model.find('pose')
    if pose_element is None:
        pose_element = ET.SubElement(model, 'pose')
    pose_element.text = ' '.join(str(value) for value in pose)

    robot = description['robot']
    joints = robot['manipulator']['joints']
    model_joints = {joint.get('name'): joint for joint in model.findall('joint')}
    names = {key: config['joint_name_template'].format(key=key) for key in joints}
    if (len(set(names.values())) != len(names) or
            any(name not in model_joints for name in names.values())):
        raise ValueError('Configured arm joint names must uniquely match rendered joints')

    if hold['enabled']:
        if set(hold['positions']) != set(joints):
            raise ValueError('Holding positions must cover exactly the description arm joints')
        # Reject competing effort/velocity/trajectory owners before adding PID.
        controllers = {
            'JointPositionController', 'JointController', 'JointTrajectoryController', 'DiffDrive',
        }
        for plugin in model.findall('plugin'):
            system = plugin.get('name', '').rsplit('::', 1)[-1]
            controlled = {element.text for element in plugin.findall('joint_name')}
            controlled.update(element.text for element in plugin.findall('left_joint'))
            controlled.update(element.text for element in plugin.findall('right_joint'))
            if system in controllers and controlled.intersection(names.values()):
                raise ValueError('Conflicting Gazebo joint controller on held arm')
        overrides = hold.get('joint_overrides', {})
        if not isinstance(overrides, Mapping) or set(overrides).difference(joints):
            raise ValueError('PID overrides must use description joint keys')
        if not isinstance(hold['pid'], Mapping):
            raise ValueError('Holding PID parameters must be a mapping')
        for key, joint in joints.items():
            pid = dict(hold['pid'])
            entry = overrides.get(key, {})
            if not isinstance(entry, Mapping) or set(entry).difference({'pid'}):
                raise ValueError(f'{key} joint override supports only a pid mapping')
            override = entry.get('pid', {})
            if not isinstance(override, Mapping):
                raise ValueError(f'{key} PID override must be a mapping')
            if set(override).difference(pid):
                raise ValueError(f'Unknown {key} PID override parameters')
            pid.update(override)
            gains = []
            for gain in ['p_gain', 'i_gain', 'd_gain']:
                value = _number(pid[gain], gain, positive=(gain == 'p_gain'),
                                nonnegative=(gain != 'p_gain'))
                gains.append((gain, value))
            integral_lower = _number(pid['i_min'], 'i_min')
            integral_upper = _number(pid['i_max'], 'i_max')
            if integral_lower > integral_upper:
                raise ValueError('PID i_min must not exceed i_max')
            offset = _number(pid['cmd_offset'], 'cmd_offset')
            position = _number(hold['positions'][key], f'{key} position')
            lower = _number(joint['lower'], f'{key} lower limit')
            upper = _number(joint['upper'], f'{key} upper limit')
            effort = _number(joint['effort'], f'{key} effort', positive=True)
            if not lower <= position <= upper:
                raise ValueError(f'{key} holding position exceeds description joint limits')
            if abs(offset) > effort:
                raise ValueError(f'{key} PID offset exceeds description effort limit')
            _plugin(model, 'JointPositionController', [
                ('joint_name', names[key]), ('initial_position', position),
                *gains, ('i_min', integral_lower), ('i_max', integral_upper),
                ('cmd_offset', offset), ('cmd_min', -effort), ('cmd_max', effort),
                ('use_velocity_commands', 'false'),
            ])

    feedback = config['joint_states']
    if any(p.get('name') == 'gz::sim::systems::JointStatePublisher'
           for p in model.findall('plugin')):
        raise ValueError('JointStatePublisher already exists')
    topic = feedback['topic']
    if not isinstance(topic, str) or not topic.strip():
        raise ValueError('Joint state topic must be nonempty')
    _plugin(model, 'JointStatePublisher', [
        ('topic', topic),
    ])

    drive = config['differential_drive']
    if drive['enabled']:
        if any(p.get('name') == 'gz::sim::systems::DiffDrive' for p in model.findall('plugin')):
            raise ValueError('DiffDrive already exists')
        left, right = drive['left_joints'], drive['right_joints']
        if set(left + right).intersection(names.values()):
            raise ValueError('Drive wheel joints cannot be arm joints')
        if (not left or not right or len(set(left + right)) != len(left + right) or
                any(name not in model_joints for name in left + right)):
            raise ValueError('Drive wheel joints must be distinct rendered joints')
        wheel = robot['wheels']
        track_width = _number(2.0 * abs(wheel['y_offset']), 'track width', positive=True)
        parameters = [('left_joint', name) for name in left]
        parameters += [('right_joint', name) for name in right]
        parameters += [
            ('wheel_radius', _number(wheel['radius'], 'wheel radius', positive=True)),
            ('wheel_separation', track_width),
            ('odom_publish_frequency', _number(drive['rate_hz'], 'odometry rate', positive=True)),
        ]
        for key in ['topic', 'odom_topic', 'frame_id', 'child_frame_id']:
            if not isinstance(drive[key], str) or not drive[key].strip():
                raise ValueError(f'Drive {key} must be nonempty')
            parameters.append((key, drive[key]))
        _plugin(model, 'DiffDrive', parameters)
    return model


def build_robot_model(xacro_path, description_path, config, hold, output_directory):
    """Render the existing description and return a configured SDF model."""
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    urdf = subprocess.run(
        ['xacro', str(xacro_path), f'config_file:={description_path}'],
        check=True, capture_output=True, text=True,
    )
    urdf_path = output / 'robot.urdf'
    urdf_path.write_text(urdf.stdout, encoding='utf-8')
    converted = subprocess.run(['gz', 'sdf', '-p', str(urdf_path)],
                               check=True, capture_output=True, text=True)
    model = ET.fromstring(converted.stdout).find('model')
    if model is None:
        raise ValueError('Converted robot SDF requires one model')
    configured = configure_robot_model(model, load_yaml(description_path), config, hold)
    sdf = ET.Element('sdf', version=ET.fromstring(converted.stdout).get('version'))
    sdf.append(configured)
    ET.ElementTree(sdf).write(output / 'robot.sdf', encoding='unicode', xml_declaration=True)
    return configured
