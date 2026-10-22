
"""Launch the simulation from an external YAML configuration."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_simulation.factory.config_loader import load_yaml
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

    # Build the configured Gazebo world.
    output_dir = Path(
        tempfile.mkdtemp(prefix="robot_simulation_")
    )

    robot_launch = cfg.get('robot', {'enabled': False})
    robot_model = None
    robot_cfg = None
    if robot_launch['enabled']:
        robot_cfg = load_yaml(resolve_path(robot_launch['config']))['robot']
        hold_cfg = load_yaml(resolve_path(robot_launch['joint_position_hold']))['joint_position_hold']
        description = robot_cfg['description']
        description_share = Path(get_package_share_directory(description['package']))
        robot_model = build_robot_model(
            description_share / description['xacro'],
            description_share / description['config'],
            robot_cfg, hold_cfg, output_dir,
        )

    world_path, world_name = build_world(
        resolve_path(world_cfg["source"]),
        resolve_path(world_cfg["terrain_config"]),
        resolve_path(target_cfg["config"]),
        output_dir / world_cfg["output_name"],
        entity_name=target_cfg["entity_name"],
        system_plugins=world_cfg["system_plugins"],
        models=[] if robot_model is None else [robot_model],
    )

    # Start Gazebo.
    gz_share = Path(
        get_package_share_directory("ros_gz_sim")
    )

    gz_args = str(world_path)

    if gazebo_cfg["start_running"]:
        gz_args = "-r " + gz_args

    gui_override = LaunchConfiguration('gui').perform(context).lower()
    if gui_override not in ['', 'true', 'false']:
        raise ValueError('gui must be true or false')
    gui_enabled = gazebo_cfg['gui'] if not gui_override else gui_override == 'true'
    if not gui_enabled:
        gz_args = "-s " + gz_args

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args,
        }.items(),
    )

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
        executable="parameter_bridge", remappings=[('/joint_states', '/v2/raw_joints')],
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
            }
        ],
    )

    return [
        gazebo,
        bridge,
        simulation,
    ]


def generate_launch_description():
    """Create the ROS 2 launch description."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="config/launch.yaml",
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
            OpaqueFunction(function=configure),
        ]
    )
