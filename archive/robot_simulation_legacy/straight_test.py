import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StraightTest(Node):

    def __init__(self):
        super().__init__('straight_test')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.duration = 5.0
        self.speed = 0.5

        self.start_time = None

        self.timer = self.create_timer(
            0.1,
            self.update
        )

    def update(self):

        # Wait until both subscribers are connected:
        # 1. nominal_kinematics_node
        # 2. ros_gz_bridge
        if self.publisher.get_subscription_count() < 2:
            return

        # Start experiment clock only after communication is ready
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info('Straight test started')

        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        msg = Twist()

        if elapsed < self.duration:
            msg.linear.x = self.speed
            self.publisher.publish(msg)

        else:
            msg.linear.x = 0.0
            self.publisher.publish(msg)

            self.get_logger().info('Straight test finished')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = StraightTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()