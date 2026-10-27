
"""Launch the simulation from an external YAML configuration."""

from pathlib import Path
import math
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import Node

from robot_configuration.yaml_loader import load_yaml
from robot_simulation.gazebo.world_builder import build_world
from robot_simulation.gazebo.robot_builder import build_robot_model


def configure(context):
    """Configure and launch the simulation."""

    share = Path(
        get_package_share_directory("robot_simulation")
    )

    config_arg = Path(
        LaunchConfiguration("config").perform(context)
    ).expanduser()

    # Resolve the launch configuration itself.
    config_path = (
        config_arg
        if config_arg.is_absolute()
        else share / config_arg
    ).resolve()

    if not config_path.is_file():
        raise FileNotFoundError(
            f"Simulation launch configuration not found: {config_path}"
        )

    cfg = load_yaml(str(config_path))

    # Explicit resource base, independent of symlink-install.
    base_mode = cfg["paths"]["base"]

    if base_mode == "package":
        resource_base = share
    elif base_mode == "config":
        resource_base = config_path.parent
    else:
        raise ValueError(
            f"Unsupported paths.base: {base_mode!r}. "
            "Expected 'package' or 'config'."
        )

    def resolve_path(value):
        """Resolve a resource path from the configured base."""
        candidate = Path(value).expanduser()

        if candidate.is_absolute():
            return candidate.resolve()

        return (resource_base / candidate).resolve()

    world_cfg = cfg["world"]
    target_cfg = cfg["target"]
    tracking_cfg = cfg["tracking"]
    simulation_cfg = cfg["simulation"]
    gazebo_cfg = cfg["gazebo"]

    # Validate owner field/context contracts before any process starts.
    from robot_simulation.factory.deployment_configuration import validate_deployment_configuration
    from robot_simulation.factory.simulation_factory import SimulationFactory
    target_data = load_yaml(resolve_path(cfg['target']['config']))
    environment_data = load_yaml(resolve_path(cfg['simulation']['environment_config']))
    integration_data = load_yaml(resolve_path(cfg['simulation']['config']))
    validate_deployment_configuration(cfg, target_data, environment_data, integration_data,
                                     load_yaml(resolve_path(cfg['simulation']['measurement_config'])))
    _, integration_dt = SimulationFactory.with_defaults().create(target_data, environment_data, integration_data)

    sensor_path = None
    sensor = None
    if cfg.get('localization_sensor_config'):
        from robot_simulation.measurements.planar_localization import validate_configuration
        sensor_path = resolve_path(cfg['localization_sensor_config'])
        sensor = load_yaml(sensor_path)['localization_sensor']
        validate_configuration(sensor)

    # Build the configured Gazebo world.
    output_dir = Path(
        tempfile.mkdtemp(prefix="robot_simulation_")
    )

    robot_launch = cfg.get('robot', {'enabled': False})
    robot_model = None
    robot_cfg = None
    if robot_launch['enabled']:
        robot_cfg = load_yaml(resolve_path(robot_launch['config']))['robot']
        truth_override = LaunchConfiguration('publish_ground_truth').perform(context).lower()
        if truth_override not in ['', 'true', 'false']:
            raise ValueError('publish_ground_truth must be true or false')
        if truth_override:
            robot_cfg['ground_truth_odometry']['enabled'] = truth_override == 'true'
        if sensor and sensor['enabled']:
            truth = robot_cfg['ground_truth_odometry']
            if (not truth['enabled'] or truth['topic'] != sensor['source_topic'] or truth['frame_id'] != sensor['source_frame'] or truth['child_frame_id'] != sensor['child_frame']):
                raise ValueError('Localization sensor requires matching enabled physical source')
        hold_cfg = load_yaml(resolve_path(robot_launch['joint_position_hold']))['joint_position_hold']
        description = robot_cfg['description']
        description_share = Path(get_package_share_directory(description['package']))
        robot_model = build_robot_model(
            description_share / description['xacro'],
            description_share / description['config'],
            robot_cfg, hold_cfg, output_dir,
        )

    physical_context = cfg.get('physical_context')
    if physical_context:
        truth = robot_cfg['ground_truth_odometry'] if robot_cfg else {}
        if (not truth.get('enabled') or truth['topic'] != physical_context['base_topic']
                or truth['frame_id'] != physical_context['reference_frame']):
            raise ValueError('contextual guidance requires matching enabled physical ground truth')

    world_path, world_name = build_world(
        resolve_path(world_cfg["source"]),
        resolve_path(world_cfg["terrain_config"]),
        resolve_path(target_cfg["config"]),
        output_dir / world_cfg["output_name"],
        entity_name=target_cfg["entity_name"],
        system_plugins=world_cfg["system_plugins"],
        models=[] if robot_model is None else [robot_model],
        physics=world_cfg.get('physics'),
    )
    # Read the effective world step, including the existing source-world value
    # when the optional deployment physics override is absent.
    physical_step = float(ET.parse(world_path).getroot().find(
        'world/physics/max_step_size').text)

    gui_override = LaunchConfiguration('gui').perform(context).lower()
    if gui_override not in ['', 'true', 'false']:
        raise ValueError('gui must be true or false')
    gui_enabled = gazebo_cfg['gui'] if not gui_override else gui_override == 'true'
    # Supervise the actual headless server, without ros_gz's shell wrapper.
    # Separate GUI mode avoids the vendor CLI's forked server/GUI pair too.
    gazebo = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-s',
             *(['-r'] if gazebo_cfg['start_running'] else []),
             str(world_path)], name='gazebo_server', output='screen')
    gui = (ExecuteProcess(cmd=[FindExecutable(name='gz'), 'sim', '-g'],
                          name='gazebo_gui', output='screen') if gui_enabled else None)

    # Gazebo service and clock bridge.
    pose_service = gazebo_cfg[
        "pose_service_template"
    ].format(world=world_name)

    bridge_arguments = [
        (
            f'{gazebo_cfg["clock_topic"]}'
            "@rosgraph_msgs/msg/Clock"
            "[gz.msgs.Clock"
        ),
        (
            f'{pose_service}'
            f'@{gazebo_cfg["pose_service_type"]}'
        ),
    ]
    if robot_cfg is not None:
        truth = robot_cfg.get('ground_truth_odometry', {})
        if truth.get('enabled', False):
            bridge_arguments.append(
                f'{truth["topic"]}@nav_msgs/msg/Odometry[gz.msgs.Odometry')
        bridge_arguments.append(
            f'{robot_cfg["joint_states"]["topic"]}'
            '@sensor_msgs/msg/JointState[gz.msgs.Model'
        )
        if robot_cfg['differential_drive']['enabled']:
            bridge_arguments.append(
                f'{robot_cfg["differential_drive"]["odom_topic"]}'
                '@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            )
        enabled = LaunchConfiguration('enable_actuation').perform(context).lower()
        if enabled not in ['true', 'false']:
            raise ValueError('enable_actuation must be true or false')
        if enabled == 'true':
            if not hold_cfg['enabled'] or not robot_cfg['differential_drive']['enabled']:
                raise ValueError('Actuation requires position PIDs and DiffDrive')
            bridge_arguments.append(
                f'{robot_cfg["differential_drive"]["topic"]}'
                '@geometry_msgs/msg/Twist]gz.msgs.Twist')
            physical = load_yaml(description_share / description['config'])
            for key in physical['robot']['manipulator']['joints']:
                joint = robot_cfg['joint_name_template'].format(key=key)
                topic = robot_cfg['joint_position_topic_template'].format(
                    model=robot_cfg['entity_name'], joint=joint)
                bridge_arguments.append(f'{topic}@std_msgs/msg/Float64]gz.msgs.Double')
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_arguments,
        output="screen",
    )

    # Simulation ROS node.
    simulation = Node(
        package="robot_simulation",
        executable="simulation_node",
        output="screen",
        parameters=[
            {
                'use_sim_time': True,
                "target_config": str(
                    resolve_path(target_cfg["config"])
                ),
                "environment_config": str(
                    resolve_path(
                        simulation_cfg["environment_config"]
                    )
                ),
                "simulation_config": str(
                    resolve_path(simulation_cfg["config"])
                ),
                "measurement_config": str(
                    resolve_path(simulation_cfg["measurement_config"])
                ),
                "pose_service": pose_service,
                "entity_name": target_cfg["entity_name"],
                "tracking_source": tracking_cfg["source"],
                "tracking_position": [
                    float(value)
                    for value in tracking_cfg["position"]
                ],
                "initial_position": [
                    float(value)
                    for value in target_cfg["initial_position"]
                ],
                "initial_velocity": [
                    float(value)
                    for value in target_cfg["initial_velocity"]
                ],
                "reference_frame": tracking_cfg[
                    "reference_frame"
                ],
                "tracking_frame": tracking_cfg[
                    "tracking_frame"
                ],
                'physical_base_topic': '' if not physical_context else physical_context['base_topic'],
                'physical_base_max_age': .2 if not physical_context else physical_context['max_age'],
                'tracking_max_age': tracking_cfg.get('max_age', 0.2),
                'tracking_future_tolerance': tracking_cfg.get('future_tolerance', 0.01),
                # Sources cannot acquire more than once per physical step.
                # Include the maximum integration lookback plus sample freshness.
                'context_history_capacity': math.ceil((
                    integration_dt * integration_data['simulation']['integration']['max_steps']
                    + 2 * max(tracking_cfg['max_age'], cfg.get('physical_context', {}).get('max_age', 0.)))
                    / physical_step) + 2,
            }
        ],
    )

    nodes = [gazebo, bridge, simulation]
    if sensor and sensor['enabled']:
        nodes.append(Node(package='robot_simulation', executable='localization_sensor',
            parameters=[{'use_sim_time': True, 'configuration_file': str(sensor_path)}], output='screen'))
    essential = [node for node in nodes if isinstance(node, (Node, ExecuteProcess))]
    guards = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='Essential simulation process exited'))]))
        for node in essential]
    return [*guards, *nodes, *([gui] if gui is not None else [])]


def generate_launch_description():
    """Create the ROS 2 launch description."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                description=(
                    "Absolute or package-relative path "
                    "to the simulation launch YAML"
                ),
            ),
            DeclareLaunchArgument(
                'gui', default_value='',
                description='Override configured Gazebo GUI: true or false',
            ),
            DeclareLaunchArgument('enable_actuation', default_value='false',
                                  description='Explicitly enable ROS-to-Gazebo commands'),
            DeclareLaunchArgument('publish_ground_truth', default_value='',
                                  description='Optional diagnostic physical-base odometry'),
            OpaqueFunction(function=configure),
        ]
    )
