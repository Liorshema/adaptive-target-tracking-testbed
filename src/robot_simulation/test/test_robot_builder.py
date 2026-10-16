"""Verify configurable Gazebo holding against real description limits."""

import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import pytest

from robot_configuration.yaml_loader import load_yaml


def configure_model(*args):
    """Load the implementation at call time to show missing behavior as a failure."""
    from robot_simulation.gazebo.robot_builder import configure_robot_model
    return configure_robot_model(*args)


@pytest.fixture
def inputs():
    """Use the existing physical description and custom external actuation data."""
    root = Path(__file__).resolve().parents[2]
    description = load_yaml(root / 'robot_description/config/robot/robot.yaml')
    model = ET.fromstring('<model name="original"><link name="base"/></model>')
    for key in description['robot']['manipulator']['joints']:
        ET.SubElement(model, 'joint', name=f'custom_{key}', type='revolute')
    config = {
        'entity_name': 'other_robot', 'pose': [1., 2., 0.25, 0., 0., 0.3],
        'joint_name_template': 'custom_{key}',
        'joint_position_topic_template': '/custom/{model}/{joint}/position',
        'joint_states': {'topic': '/custom/joints'},
        'differential_drive': {'enabled': False},
    }
    hold = {'enabled': True, 'pid': {
        'p_gain': 80., 'i_gain': 4., 'd_gain': 3.,
        'i_min': -2., 'i_max': 2., 'cmd_offset': 0.,
    }, 'positions': {'base_yaw': 0.2, 'shoulder': 0.6, 'elbow': -0.3, 'wrist': 0.1}}
    return model, description, config, hold


def test_hold_uses_configured_names_targets_gains_and_authoritative_effort(inputs):
    """Generate effort PID systems without copying fixture constants into code."""
    model = configure_model(*inputs)
    assert model.get('name') == 'other_robot'
    assert model.findtext('pose') == '1.0 2.0 0.25 0.0 0.0 0.3'
    plugins = [p for p in model.findall('plugin')
               if p.get('name') == 'gz::sim::systems::JointPositionController']
    assert len(plugins) == 4
    for plugin, key in zip(plugins, ['base_yaw', 'shoulder', 'elbow', 'wrist']):
        assert plugin.findtext('joint_name') == 'custom_' + key
        assert plugin.findtext('topic') == f'/custom/other_robot/custom_{key}/position'
        assert float(plugin.findtext('initial_position')) == inputs[3]['positions'][key]
        assert float(plugin.findtext('p_gain')) == 80.0
        assert float(plugin.findtext('i_gain')) == 4.0
        assert float(plugin.findtext('d_gain')) == 3.0
        expected_effort = 10.0 if key == 'wrist' else 20.0
        assert float(plugin.findtext('cmd_max')) == expected_effort
        assert float(plugin.findtext('cmd_min')) == -expected_effort
        assert plugin.findtext('use_velocity_commands') == 'false'
    feedback = model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    assert feedback.findtext('topic') == '/custom/joints'


def test_optional_ground_truth_is_observational_and_configuration_driven(inputs):
    """The physical pose publisher must not replace DiffDrive or joint PID owners."""
    model, description, config, hold = deepcopy(inputs)
    config['ground_truth_odometry'] = {
        'enabled': True, 'rate_hz': 20., 'dimensions': 3,
        'frame_id': 'world', 'child_frame_id': 'base_link',
        'topic': '/test/physical_pose', 'tf_topic': '/test/physical_tf'}
    result = configure_model(model, description, config, hold)
    plugin = result.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert plugin is not None
    assert plugin.findtext('odom_topic') == '/test/physical_pose'
    assert plugin.findtext('dimensions') == '3'
    assert len(result.findall("plugin[@name='gz::sim::systems::JointPositionController']")) == 4


@pytest.mark.parametrize('bad', [
    'out_of_bounds', 'below_horizontal', 'nonfinite', 'missing', 'unknown', 'pid'])
def test_invalid_hold_configuration_rejected_before_world_launch(inputs, bad):
    """Invalid setpoints cannot silently launch an uncontrolled arm."""
    model, description, config, hold = deepcopy(inputs)
    if bad == 'out_of_bounds':
        hold['positions']['shoulder'] = 2.0
    elif bad == 'below_horizontal':
        hold['positions']['shoulder'] = -0.1
    elif bad == 'nonfinite':
        hold['positions']['shoulder'] = float('nan')
    elif bad == 'missing':
        del hold['positions']['wrist']
    elif bad == 'unknown':
        hold['positions']['unknown'] = 0.0
    else:
        hold['pid']['p_gain'] = -1.0
    with pytest.raises(ValueError):
        configure_model(model, description, config, hold)


def test_hold_can_be_disabled_without_adding_competing_controllers(inputs):
    """Actuator ownership can move to a future execution layer."""
    model, description, config, hold = inputs
    hold['enabled'] = False
    configured = configure_model(model, description, config, hold)
    assert not [p for p in configured.findall('plugin')
                if 'JointPositionController' in p.get('name', '')]


def test_reject_conflicting_joint_controller(inputs):
    """Do not attach a second effort controller to the same joint."""
    model, description, config, hold = inputs
    plugin = ET.SubElement(model, 'plugin', name='gz::sim::systems::JointController')
    ET.SubElement(plugin, 'joint_name').text = 'custom_shoulder'
    with pytest.raises(ValueError, match='controller'):
        configure_model(model, description, config, hold)


def test_reject_drive_using_arm_joints(inputs):
    """Wheel velocity actuation cannot compete with effort holding on the arm."""
    model, description, config, hold = inputs
    config['differential_drive'] = {
        'enabled': True, 'left_joints': ['custom_shoulder'],
        'right_joints': ['custom_elbow'], 'rate_hz': 50.,
        'topic': '/drive', 'odom_topic': '/odom', 'frame_id': 'odom',
        'child_frame_id': 'base',
    }
    with pytest.raises(ValueError, match='arm'):
        configure_model(model, description, config, hold)


def test_reject_existing_drive_actuating_held_arm(inputs):
    """URDF-supplied drive controllers must obey the same ownership rule."""
    model, description, config, hold = inputs
    plugin = ET.SubElement(model, 'plugin', name='gz::sim::systems::DiffDrive')
    ET.SubElement(plugin, 'left_joint').text = 'custom_shoulder'
    ET.SubElement(plugin, 'right_joint').text = 'custom_elbow'
    with pytest.raises(ValueError, match='controller'):
        configure_model(model, description, config, hold)


def test_joint_specific_pid_override_preserves_other_joint_defaults(inputs):
    """Different arm inertias can be tuned through YAML without code constants."""
    model, description, config, hold = inputs
    hold['joint_overrides'] = {'wrist': {'pid': {'d_gain': 0.25}}}
    configured = configure_model(model, description, config, hold)
    plugins = [p for p in configured.findall('plugin')
               if p.get('name') == 'gz::sim::systems::JointPositionController']
    gains = {p.findtext('joint_name'): float(p.findtext('d_gain')) for p in plugins}
    assert gains == {'custom_base_yaw': 3., 'custom_shoulder': 3.,
                     'custom_elbow': 3., 'custom_wrist': 0.25}


@pytest.mark.parametrize('override', [
    {'unknown': {'pid': {'d_gain': 0.25}}},
    {'wrist': {'pid': {'d_gain': float('nan')}}},
    {'wrist': {'d_gain': 0.5}},
    {'wrist': {'pidd': {'d_gain': 0.5}}},
    {'wrist': {'pid': [0.5]}},
])
def test_invalid_joint_override_rejected(inputs, override):
    """Overrides cannot silently target nonexistent joints or invalid gains."""
    model, description, config, hold = inputs
    hold['joint_overrides'] = override
    with pytest.raises(ValueError):
        configure_model(model, description, config, hold)


def test_robot_builder_generates_real_description_without_geometry_changes(tmp_path):
    """Render the real Xacro/SDF and derive wheel parameters from its YAML."""
    from robot_simulation.gazebo.robot_builder import build_robot_model
    root = Path(__file__).resolve().parents[2]
    robot = load_yaml(root / 'robot_simulation/config/gazebo/robot.yaml')['robot']
    hold = load_yaml(
        root / 'robot_simulation/config/gazebo/joint_position_hold.yaml'
    )['joint_position_hold']
    model = build_robot_model(
        root / 'robot_description/urdf/robot.urdf.xacro',
        root / 'robot_description/config/robot/robot.yaml', robot, hold, tmp_path,
    )
    assert model.find("joint[@name='shoulder_joint']/axis/xyz").text == '0 -1 0'
    assert float(model.findtext("joint[@name='shoulder_joint']/axis/limit/lower")) == 0.0
    assert model.find("link[@name='base_link']/inertial/mass").text == '10'
    drive = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(drive.findtext('wheel_radius')) == 0.15
    assert float(drive.findtext('wheel_separation')) == 0.68
