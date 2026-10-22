"""Compose the permanent simulation and existing tracking pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml


def configure(context):
    """Resolve deployment geometry, topics and explicit actuation ownership."""
    def share(package):
        return Path(get_package_share_directory(package))

    def load(path):
        return yaml.safe_load(path.read_text())

    def flag(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ['true', 'false']:
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    config = Path(LaunchConfiguration('config').perform(context)).expanduser()
    if not config.is_absolute():
        config = share('robot_bringup') / config
    cfg = load(config)
    simulation_path = share('robot_simulation') / cfg['simulation_config']
    simulation_cfg = load(simulation_path)
    if simulation_cfg['paths']['base'] == 'package':
        resource = share('robot_simulation')
    elif simulation_cfg['paths']['base'] == 'config':
        resource = simulation_path.parent
    else:
        raise ValueError('Unsupported simulation resource base')
    robot_path = resource / simulation_cfg['robot']['config']
    robot = load(robot_path)['robot']
    description_path = share(robot['description']['package']) / robot['description']['config']
    physical = load(description_path)['robot']['manipulator']['joints']
    names = [robot['joint_name_template'].format(key=key) for key in physical]
    drive = robot['differential_drive']
    actuation_path = share('robot_simulation') / cfg['actuation_config']
    actuator = load(actuation_path)['actuation']
    control_path = share('robot_control') / cfg['control_config']
    control = load(control_path)['control_node']['ros__parameters']
    enabled = flag('enable_actuation')
    publish_arg = LaunchConfiguration('publish_commands').perform(context).lower()
    publish = enabled if publish_arg == '' else flag('publish_commands')
    if publish and not enabled:
        raise ValueError('Command publishing requires explicit enable_actuation=true')
    pose = robot['pose']
    common = {'use_sim_time': True}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share('robot_simulation') /
                                              'launch/simulation.launch.py')),
            launch_arguments={'config': str(simulation_path),
                              'gui': LaunchConfiguration('gui').perform(context),
                              'enable_actuation': str(enabled).lower()}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', str(pose[0]), '--y', str(pose[1]), '--z', str(pose[2]),
                        '--roll', str(pose[3]), '--pitch', str(pose[4]), '--yaw', str(pose[5]),
                        '--frame-id', simulation_cfg['tracking']['reference_frame'],
                        '--child-frame-id', drive['frame_id']], parameters=[common]),
        Node(package='robot_estimation', executable='estimation_node', output='screen', prefix='python3 /home/user/ros2_ws/docs/validation/v2_performance_before_estimator_cadence/estimation_cadence.py',
             parameters=[str(share('robot_estimation') / cfg['robot_estimator_config']),
                         str(share('robot_estimation') / cfg['target_estimator_config']), common,
                         {'joint_count': len(names), 'joint_names': names,
                          'wheel_odometry_topic': drive['odom_topic'],
                          'joint_states_topic': robot['joint_states']['topic'],
                          'robot_state_frame_id': drive['frame_id']}]),
        Node(package='robot_prediction', executable='prediction_node', output='screen',
             parameters=[common]),
        Node(package='robot_control', executable='control_node', output='screen',
             parameters=[str(control_path), common,
                         {'publish_commands': publish, 'command_joint_names': names,
                          'command_frame_id': drive['child_frame_id'],
                          'command_topic': actuator['command_topic'],
                          'publish_telemetry': True}]),
        Node(package='robot_simulation', executable='velocity_actuation', output='screen', prefix='python3 /home/user/ros2_ws/docs/validation/v2_performance_before_estimator_cadence/actuation_counters.py',
             parameters=[common, {'enabled': enabled,
                                  'configuration_file': str(actuation_path),
                                  'description_file': str(description_path),
                                  'robot_configuration_file': str(robot_path),
                                  'base_velocity_lower': control['base_velocity_lower'],
                                  'base_velocity_upper': control['base_velocity_upper']}]),
    ]


def generate_launch_description():
    """Keep Compute Only and the separate GUI workflow as launch defaults."""
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='config/closed_loop.yaml'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('enable_actuation', default_value='false'),
        DeclareLaunchArgument('publish_commands', default_value='',
                              description='Optional disable override for watchdog experiments'),
        OpaqueFunction(function=configure),
    ])
