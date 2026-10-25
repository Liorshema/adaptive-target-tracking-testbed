"""Compose the permanent simulation and existing tracking pipeline."""

from pathlib import Path
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from robot_bringup.factory.sensor_inventory import format_sensor_inventory
from robot_bringup.factory.system_configuration import load_system_configuration

from robot_configuration.yaml_loader import load_yaml


def configure(context):
    """Resolve deployment geometry, topics and explicit actuation ownership."""
    def share(package):
        return Path(get_package_share_directory(package))

    def load(path):
        return load_yaml(path)

    raw = {name: LaunchConfiguration(name).perform(context)
           for name in ['config', 'gui', 'enable_actuation', 'visualize_tracking',
                        'publish_ground_truth', 'publish_commands', 'strategy_config',
                        'lookahead', 'feedforward']}
    manifest = LaunchConfiguration('system_config', default='').perform(context)
    plan = None
    if manifest:
        path = Path(manifest).expanduser()
        if not path.is_absolute():
            path = share('robot_bringup') / path
        plan = load_system_configuration(path, share, overrides=raw)

    def value(name):
        if raw[name] != '':
            return raw[name]
        if plan and name in plan['runtime']:
            return str(plan['runtime'][name]).lower()
        return 'false' if name in ['gui', 'enable_actuation', 'visualize_tracking'] else ''

    def flag(name):
        selected = value(name).lower()
        if selected not in ['true', 'false']:
            raise ValueError(f'{name} must be true or false')
        return selected == 'true'

    config = Path(raw['config'] or 'config/closed_loop.yaml').expanduser()
    if not config.is_absolute():
        config = share('robot_bringup') / config
    cfg = plan['profile'] if plan else load(config)
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
    estimation_overrides = (load(share('robot_bringup') / cfg['estimation_overrides'])
                            if 'estimation_overrides' in cfg else {})
    enabled = flag('enable_actuation')
    publish_arg = value('publish_commands').lower()
    publish = enabled if publish_arg == '' else flag('publish_commands')
    if publish and not enabled:
        raise ValueError('Command publishing requires explicit enable_actuation=true')
    visualize = flag('visualize_tracking')
    ground_truth = value('publish_ground_truth').lower()
    if ground_truth not in ['', 'true', 'false']:
        raise ValueError('publish_ground_truth must be true, false or empty')
    if visualize:
        if ground_truth == 'false':
            raise ValueError('Tracking visualization requires publish_ground_truth=true')
        ground_truth = 'true'
        from robot_bringup.visualization import validate_configuration
        visual_argument = LaunchConfiguration('visualization_config', default='').perform(context)
        visual_path = Path(visual_argument or 'config/tracking_visualization.yaml').expanduser()
        if not visual_path.is_absolute():
            visual_path = share('robot_bringup') / visual_path
        validate_configuration(load(visual_path))
        if plan:
            visual_estimation = plan['estimation_parameters']
        else:
            visual_estimation = {}
            for key in ['robot_estimator_config', 'target_estimator_config']:
                visual_estimation.update(load(share('robot_estimation') / cfg[key])['estimation_node']['ros__parameters'])
            visual_estimation.update(estimation_overrides)
    pose = robot['pose']
    common = {'use_sim_time': True}
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share('robot_simulation') /
                                              'launch/simulation.launch.py')),
            launch_arguments={'config': str(simulation_path),
                              'gui': value('gui'),
                              'publish_ground_truth': ground_truth,
                              'enable_actuation': str(enabled).lower()}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', str(pose[0]), '--y', str(pose[1]), '--z', str(pose[2]),
                        '--roll', str(pose[3]), '--pitch', str(pose[4]), '--yaw', str(pose[5]),
                        '--frame-id', simulation_cfg['tracking']['reference_frame'],
                        '--child-frame-id', drive['frame_id']], parameters=[common]),
        Node(package='robot_estimation', executable='estimation_node', output='screen',
             parameters=[plan['estimation_parameters'] if plan else str(share('robot_estimation') / cfg['robot_estimator_config']),
                         {} if plan else str(share('robot_estimation') / cfg['target_estimator_config']), common,
                         {} if plan else estimation_overrides,
                         {'joint_count': len(names), 'joint_names': names,
                          'robot_description_file': str(description_path),
                          'wheel_odometry_topic': drive['odom_topic'],
                          'wheel_odom_frame': drive['frame_id'],
                          'base_frame': drive['child_frame_id'],
                          'joint_states_topic': robot['joint_states']['topic'],
                          'robot_state_frame_id': drive['frame_id']},
                         ({key: plan['selection'][key] for key in
                           ['robot_estimator_type', 'target_estimator_type']} if plan else {})]),
        Node(package='robot_prediction', executable='prediction_node', output='screen',
             parameters=[common, ({'configuration_file': str(plan['prediction_path']),
                                   **plan['prediction_topics'],
                                   **{key: plan['selection'][key] for key in
                                      ['predictor_type', 'adaptation_type', 'fusion_type']}}
                                  if plan else {})]),
        Node(package='robot_control', executable='control_node', output='screen',
             parameters=[plan['control_parameters'] if plan else str(control_path), common,
                         {'publish_commands': publish, 'command_joint_names': names,
                          'command_frame_id': drive['child_frame_id'],
                          'command_topic': actuator['command_topic'],
                          'joint_count': len(names),
                          'robot_description_file': str(description_path),
                          'strategy_config_file': (str(plan['strategy_path']) if plan and plan['strategy_path']
                                                   else '' if plan or raw['strategy_config'] == 'none'
                                                   else raw['strategy_config']),
                          'lookahead_override': value('lookahead'),
                          'feedforward_override': value('feedforward'),
                          'publish_telemetry': True}]),
        Node(package='robot_simulation', executable='velocity_actuation', output='screen',
             parameters=[common, {'enabled': enabled,
                                  'configuration_file': str(actuation_path),
                                  'description_file': str(description_path),
                                  'robot_configuration_file': str(robot_path),
                                  'holding_configuration_file': str(
                                      resource / simulation_cfg['robot']['joint_position_hold']),
                                  'base_velocity_lower': control['base_velocity_lower'],
                                  'base_velocity_upper': control['base_velocity_upper']}]),
    ]
    if visualize:
        nodes.append(Node(package='robot_bringup', executable='tracking_visualization',
                          output='screen', parameters=[
                              common,
                              {'configuration_file': str(visual_path),
                               'description_file': str(description_path), 'joint_names': names,
                               'joint_topic': robot['joint_states']['topic'],
                               'odom_topic': drive['odom_topic'],
                               'physical_pose_topic': robot['ground_truth_odometry']['topic'],
                               'reference_frame': simulation_cfg['tracking']['reference_frame'],
                               'forward_axis': control['tracking_forward_axis'],
                               'tracking_radius': float(control['tracking_radius']),
                               'target_estimate_topic': visual_estimation['target_state_full_topic'],
                               'robot_estimate_topic': visual_estimation['robot_state_full_topic'],
                               'prediction_topic': control['prediction_topic'],
                               'base_frame': drive['child_frame_id'],
                               'odometry_frame': drive['frame_id']}]))
    if simulation_cfg['tracking']['source'] == 'tf':
        physical_tf = simulation_cfg['tracking'].get('pose_source') == 'physical'
        pose_feedback = robot['ground_truth_odometry'] if physical_tf else drive
        nodes.append(Node(package='robot_bringup', executable='tracking_feedback_tf',
                          output='screen', parameters=[
                              common, cfg['tracking_tf'],
                              {'joint_names': names,
                               'description_file': str(description_path),
                               'joint_topic': robot['joint_states']['topic'],
                               'odom_topic': pose_feedback['topic'] if physical_tf else drive['odom_topic'],
                               'parent_frame': pose_feedback['frame_id'],
                               'child_frame': simulation_cfg['tracking']['tracking_frame']}]))
    if plan:
        nodes.insert(0, LogInfo(msg=format_sensor_inventory(plan['sensor_inventory'])))
        nodes.insert(0, LogInfo(msg='System assembly: ' + json.dumps({
            'identities': plan['identities'], 'manifest': str(plan['source']),
            'profile': str(plan['profile_path']), 'description': str(plan['description_path']),
            'prediction': str(plan['prediction_path']), 'runtime': plan['runtime']}, sort_keys=True)))
    # All pipeline nodes are required for a meaningful experiment. A failed
    # child must terminate the composition rather than leave launch alive.
    essential = [node for node in nodes if isinstance(node, Node)
                 and node.node_executable != 'tracking_visualization']
    guards = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='Essential tracking process exited'))]))
        for node in essential]
    return [*guards, *nodes]


def generate_launch_description():
    """Keep Compute Only and the separate GUI workflow as launch defaults."""
    return LaunchDescription([
        DeclareLaunchArgument('system_config', default_value='',
                              description='Unified composition manifest; empty keeps legacy workflow'),
        DeclareLaunchArgument('config', default_value='',
                              description='Profile override; legacy default config/closed_loop.yaml'),
        DeclareLaunchArgument('gui', default_value='', description='Default false'),
        DeclareLaunchArgument('enable_actuation', default_value='', description='Default false'),
        DeclareLaunchArgument('visualize_tracking', default_value='',
                              description='Publish measured frame axes and optical/target arrows'),
        DeclareLaunchArgument('visualization_config', default_value='',
                              description='Optional read-only visualization YAML; empty uses installed default'),
        DeclareLaunchArgument('publish_ground_truth', default_value='',
                              description=(
                                  'Optional physical pose output independent of RViz markers')),
        DeclareLaunchArgument('publish_commands', default_value='',
                              description='Optional disable override for watchdog experiments'),
        DeclareLaunchArgument('strategy_config', default_value='',
                              description='Control strategy YAML; empty retains v1 DLS'),
        DeclareLaunchArgument('lookahead', default_value='',
                              description='Override strategy lookahead in simulation seconds'),
        DeclareLaunchArgument('feedforward', default_value='',
                              description='Override strategy target feedforward: true or false'),
        OpaqueFunction(function=configure),
    ])
