"""Launch the robot description in RViz."""

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the robot-description visualization launch."""
    package_share = Path(
        get_package_share_directory(
            'robot_description'
        )
    )

    xacro_path = (
        package_share
        / 'urdf'
        / 'robot.urdf.xacro'
    )

    config_path = (
        package_share
        / 'config'
        / 'robot'
        / 'robot.yaml'
    )

    rviz_config_path = (
        package_share
        / 'config'
        / 'robot.rviz'
    )

    robot_description = {
        'robot_description': Command([
            'xacro ',
            str(xacro_path),
            ' config_file:=',
            str(config_path),
        ])
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            str(rviz_config_path),
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
