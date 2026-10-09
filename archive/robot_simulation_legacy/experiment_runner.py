from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from ament_index_python.packages import get_package_share_directory

import yaml


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__('experiment_runner')

        # --------------------------------------------------
        # Configuration
        # --------------------------------------------------

        default_config = (
            Path(
                get_package_share_directory(
                    'robot_simulation'
                )
            )
            / 'config'
            / 'experiments'
            / 'flat_baseline.yaml'
        )

        self.declare_parameter(
            'config_file',
            str(default_config),
        )

        config_path = Path(
            self.get_parameter(
                'config_file'
            ).value
        )

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        experiment = config['experiment']

        self.experiment_name = experiment['name']
        self.terrain = experiment['terrain']
        self.segments = experiment['segments']

        # --------------------------------------------------
        # ROS publishers
        # --------------------------------------------------

        self.command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.active_publisher = self.create_publisher(
            Bool,
            '/experiment/active',
            10,
        )

        # --------------------------------------------------
        # Experiment state
        # --------------------------------------------------

        self.started = False
        self.finished = False

        self.segment_index = 0
        self.segment_start_time = None

        self.timer = self.create_timer(
            0.02,
            self.update,
        )

        self.get_logger().info(
            f'Loaded experiment: {self.experiment_name}'
        )

        self.get_logger().info(
            f'Terrain: {self.terrain}'
        )

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    def publish_active(self, active):

        msg = Bool()
        msg.data = active

        self.active_publisher.publish(msg)

    def publish_command(
        self,
        linear_velocity,
        angular_velocity,
    ):

        msg = Twist()

        msg.linear.x = float(linear_velocity)
        msg.angular.z = float(angular_velocity)

        self.command_publisher.publish(msg)

    # --------------------------------------------------
    # Main experiment loop
    # --------------------------------------------------

    def update(self):

        if self.finished:
            return

        # Wait until both consumers of cmd_vel exist:
        # nominal model + Gazebo bridge
        if self.command_publisher.get_subscription_count() < 2:
            return

        # Wait until comparator is listening
        if self.active_publisher.get_subscription_count() < 1:
            return

        now = self.get_clock().now()

        # --------------------------------------------------
        # Start experiment
        # --------------------------------------------------

        if not self.started:

            self.started = True
            self.segment_start_time = now

            self.publish_active(True)

            self.get_logger().info(
                f'Experiment started: '
                f'{self.experiment_name}'
            )

        # --------------------------------------------------
        # Current segment
        # --------------------------------------------------

        segment = self.segments[
            self.segment_index
        ]

        elapsed = (
            now - self.segment_start_time
        ).nanoseconds / 1e9

        duration = float(
            segment['duration']
        )

        # --------------------------------------------------
        # Execute segment
        # --------------------------------------------------

        if elapsed < duration:

            self.publish_command(
                segment['linear_velocity'],
                segment['angular_velocity'],
            )

            return

        # --------------------------------------------------
        # Move to next segment
        # --------------------------------------------------

        self.segment_index += 1

        if self.segment_index < len(
            self.segments
        ):

            self.segment_start_time = now

            self.get_logger().info(
                f'Starting segment '
                f'{self.segment_index + 1}'
            )

            return

        # --------------------------------------------------
        # Finish experiment
        # --------------------------------------------------

        self.publish_command(
            0.0,
            0.0,
        )

        self.publish_active(False)

        self.finished = True

        self.get_logger().info(
            f'Experiment finished: '
            f'{self.experiment_name}'
        )
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = ExperimentRunner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()