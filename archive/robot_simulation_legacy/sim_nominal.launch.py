from pathlib import Path

import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def launch_setup(context):

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------

    simulation_share = Path(
        get_package_share_directory('robot_simulation')
    )

    description_share = Path(
        get_package_share_directory('robot_description')
    )

    ros_gz_sim_share = Path(
        get_package_share_directory('ros_gz_sim')
    )

    # --------------------------------------------------
    # Robot files
    # --------------------------------------------------

    robot_xacro_file = (
        description_share
        / 'urdf'
        / 'simple_robot.urdf.xacro'
    )

    robot_config_file = (
        description_share
        / 'config'
        / 'robot'
        / 'robot.yaml'
    )

    # --------------------------------------------------
    # Terrain / world files
    # --------------------------------------------------

    world_xacro_file = (
        simulation_share
        / 'worlds'
        / 'tracking_testbed.sdf.xacro'
    )

    terrain_config_file = Path(
        LaunchConfiguration(
            'terrain_config'
        ).perform(context)
    )

    generated_world_file = Path(
        '/tmp/tracking_testbed_generated.sdf'
    )

    # --------------------------------------------------
    # Generate world SDF from terrain configuration
    # --------------------------------------------------

    world_document = xacro.process_file(
        str(world_xacro_file),
        mappings={
            'terrain_config': str(
                terrain_config_file
            ),
        },
    )

    generated_world_file.write_text(
        world_document.toxml()
    )

    # --------------------------------------------------
    # Xacro -> robot_description
    # --------------------------------------------------

    robot_description = ParameterValue(
        Command([
            'xacro ',
            str(robot_xacro_file),
            ' config_file:=',
            str(robot_config_file),
        ]),
        value_type=str,
    )

    # --------------------------------------------------
    # Gazebo
    # --------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                ros_gz_sim_share
                / 'launch'
                / 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args':
                f'-r {generated_world_file}',
        }.items(),
    )

    # --------------------------------------------------
    # Robot state publisher
    # --------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description':
                    robot_description,
            }
        ],
        output='screen',
    )

    # --------------------------------------------------
    # Spawn robot
    # --------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world',
            'tracking_testbed',

            '-topic',
            'robot_description',

            '-name',
            'simple_robot',

            '-x',
            '0.0',

            '-y',
            '0.0',

            '-z',
            '0.5',
        ],
        output='screen',
    )

    # --------------------------------------------------
    # ROS <-> Gazebo bridge
    # --------------------------------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel'
            '@geometry_msgs/msg/Twist]'
            'gz.msgs.Twist',

            '/model/simple_robot/odometry'
            '@nav_msgs/msg/Odometry['
            'gz.msgs.Odometry',

            '/clock'
            '@rosgraph_msgs/msg/Clock['
            'gz.msgs.Clock',
        ],
        output='screen',
    )

    # --------------------------------------------------
    # Nominal kinematic model
    # --------------------------------------------------

    nominal_node = Node(
        package='robot_simulation',
        executable='nominal_kinematics_node',
        parameters=[
            {
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # --------------------------------------------------
    # State comparator
    # --------------------------------------------------

    state_comparator = Node(
        package='robot_simulation',
        executable='state_comparator',
        parameters=[
            {
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    return [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        nominal_node,
        state_comparator,
    ]


def generate_launch_description():

    simulation_share = Path(
        get_package_share_directory(
            'robot_simulation'
        )
    )

    default_terrain_config = (
        simulation_share
        / 'config'
        / 'terrain'
        / 'flat.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'terrain_config',
            default_value=str(
                default_terrain_config
            ),
            description=(
                'Terrain configuration YAML file'
            ),
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])