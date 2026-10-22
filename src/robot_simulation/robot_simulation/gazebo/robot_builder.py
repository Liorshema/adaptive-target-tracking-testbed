"""Build a physical robot with configurable Gazebo actuation and feedback."""

import subprocess
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from copy import deepcopy
from importlib.util import find_spec
from pathlib import Path

import numpy as np

from robot_configuration.yaml_loader import load_yaml


def actuator_library():
    """Resolve the compiled Gazebo System without importing it as a Python module."""
    spec = find_spec('robot_simulation.gazebo._actuator')
    if spec is None or not spec.origin or not Path(spec.origin).is_file():
        raise ValueError('Native actuator is not built; rebuild robot_simulation and source '
                         'install/setup.bash')
    return str(Path(spec.origin).resolve())


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
        'OdometryPublisher': 'gz-sim-odometry-publisher-system',
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
    allowed = {'enabled', 'pid', 'positions', 'joint_overrides', 'backend',
               'trace_directory', 'servo'}
    if set(hold) - allowed:
        raise ValueError(f'Unknown holding settings: {sorted(set(hold) - allowed)}')
    backend = hold.get('backend', 'legacy_pid')
    if backend not in ('legacy_pid', 'instrumented_pid', 'velocity_servo'):
        raise ValueError('Invalid holding backend')
    trace = hold.get('trace_directory', '')
    if not isinstance(trace, str) or (trace and not Path(trace).is_absolute()):
        raise ValueError('trace_directory must be empty or an absolute path')
    if backend == 'legacy_pid' and trace:
        raise ValueError('Legacy PID has no physics trace; select instrumented_pid')
    servo = hold.get('servo', {})
    if (not isinstance(servo, Mapping) or
            any(isinstance(value, bool) for value in servo.values()) or
            set(servo) - {'position_time_constant', 'acceleration_limit'} or
            (backend == 'velocity_servo' and
             set(servo) != {'position_time_constant', 'acceleration_limit'})):
        raise ValueError('Velocity servo requires its configured demand governor')
    servo = {k: _number(v, f'servo {k}', positive=True) for k, v in servo.items()}
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
            'ArmActuator',
        }
        for plugin in model.findall('plugin'):
            system = plugin.get('name', '').rsplit('::', 1)[-1]
            controlled = {element.text for element in plugin.findall('joint_name')}
            controlled.update(element.text for element in plugin.findall('left_joint'))
            controlled.update(element.text for element in plugin.findall('right_joint'))
            controlled.update(element.text for element in plugin.findall('joint/name'))
            if system in controllers and controlled.intersection(names.values()):
                raise ValueError('Conflicting Gazebo joint controller on held arm')
        overrides = hold.get('joint_overrides', {})
        if not isinstance(overrides, Mapping) or set(overrides).difference(joints):
            raise ValueError('PID overrides must use description joint keys')
        if not isinstance(hold['pid'], Mapping):
            raise ValueError('Holding PID parameters must be a mapping')
        native = None
        if backend != 'legacy_pid':
            native = ET.SubElement(model, 'plugin', filename=actuator_library(),
                                   name='robot_simulation::ArmActuator')
            ET.SubElement(native, 'backend').text = backend
            if backend == 'velocity_servo':
                for key, value in servo.items():
                    ET.SubElement(native, key).text = str(value)
            if trace:
                ET.SubElement(native, 'trace_directory').text = trace
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
            parameters = [
                ('joint_name', names[key]), ('initial_position', position),
                *([('topic', config['joint_position_topic_template'].format(
                    model=name, joint=names[key]))]
                  if 'joint_position_topic_template' in config else []),
                *gains, ('i_min', integral_lower), ('i_max', integral_upper),
                ('cmd_offset', offset), ('cmd_min', -effort), ('cmd_max', effort),
                ('use_velocity_commands', 'false'),
            ]
            if native is None:
                _plugin(model, 'JointPositionController', parameters)
            else:
                entry = ET.SubElement(native, 'joint')
                for field, value in parameters:
                    if field not in ('cmd_min', 'cmd_max', 'use_velocity_commands'):
                        tag = 'name' if field == 'joint_name' else field
                        ET.SubElement(entry, tag).text = str(value)

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
    truth = config.get('ground_truth_odometry', {})
    if truth.get('enabled', False):
        dimensions = truth['dimensions']
        if dimensions not in [2, 3]:
            raise ValueError('Ground-truth dimensions must be 2 or 3')
        parameters = [('dimensions', dimensions),
                      ('odom_publish_frequency', _number(
                          truth['rate_hz'], 'ground-truth rate', positive=True))]
        for source, destination in [
                ('frame_id', 'odom_frame'), ('child_frame_id', 'robot_base_frame'),
                ('topic', 'odom_topic'), ('tf_topic', 'tf_topic')]:
            if not isinstance(truth[source], str) or not truth[source].strip():
                raise ValueError(f'Ground-truth {source} must be nonempty')
            parameters.append((destination, truth[source]))
        _plugin(model, 'OdometryPublisher', parameters)
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
