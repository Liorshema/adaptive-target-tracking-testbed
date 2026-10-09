from pathlib import Path
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory

from robot_dynamics.kinematic_model import RobotKinematicModel
from robot_dynamics.robot_config import load_robot_parameters


class NominalKinematicsNode(Node):

    def __init__(self):
        super().__init__('nominal_kinematics_node')

        # --------------------------------------------------
        # Load robot configuration
        # --------------------------------------------------

        config_path = (
            Path(get_package_share_directory('robot_description'))
            / 'config'
            / 'robot'
            / 'robot.yaml'
        )

        params = load_robot_parameters(config_path)

        # --------------------------------------------------
        # Stage 2 kinematic model
        # --------------------------------------------------

        self.model = RobotKinematicModel(
            wheel_radius=params.wheels.radius,
            track_width=params.wheels.track_width,
        )

        # State = [x, y, theta]
        self.state = [0.0, 0.0, 0.0]

        # Latest commanded body velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # --------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/nominal/odom',
            10,
        )

        # Desired callback period: 100 Hz
        self.timer_period = 0.01

        # Store actual time of previous integration step
        self.last_update_time = self.get_clock().now()

        self.create_timer(
            self.timer_period,
            self.update,
        )

    def cmd_vel_callback(self, msg):
        """
        Store the latest commanded body velocity.
        """

        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update(self):
        """
        Propagate the nominal kinematic model using the
        actual elapsed time between callbacks.
        """

        # --------------------------------------------------
        # Actual elapsed time
        # --------------------------------------------------

        now = self.get_clock().now()

        dt = (
            now - self.last_update_time
        ).nanoseconds / 1e9

        self.last_update_time = now

        # Safety check
        if dt <= 0.0:
            return

        # --------------------------------------------------
        # Body velocity -> wheel angular velocity
        # --------------------------------------------------

        omega_left, omega_right = (
            self.model.body_to_wheel_velocity(
                self.linear_velocity,
                self.angular_velocity,
            )
        )

        # --------------------------------------------------
        # Integrate nominal robot state
        # --------------------------------------------------

        self.state = self.model.step(
            self.state,
            omega_left,
            omega_right,
            dt,
        )

        x, y, theta = self.state

        # --------------------------------------------------
        # Publish nominal odometry
        # --------------------------------------------------

        msg = Odometry()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        msg.twist.twist.linear.x = self.linear_velocity
        msg.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = NominalKinematicsNode()

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