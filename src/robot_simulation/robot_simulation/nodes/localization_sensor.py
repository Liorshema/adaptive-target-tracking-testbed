"""Simulate an explicitly imperfect external pose source; truth stays in simulation."""
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from robot_configuration.yaml_loader import load_yaml
from robot_simulation.measurements.planar_localization import PlanarLocalizationSensor


class LocalizationSensorNode(Node):
    """Publish source-stamped noisy poses after configurable latency/dropout."""

    def __init__(self):
        super().__init__('localization_sensor')
        self.declare_parameter('configuration_file', '')
        self.config = load_yaml(Path(self.get_parameter('configuration_file').value))['localization_sensor']
        self.sensor = PlanarLocalizationSensor(self.config)
        self.publisher = self.create_publisher(Odometry, self.config['topic'], 10)
        self.diagnostics = self.create_publisher(String, self.config['diagnostic_topic'], 10)
        self.create_subscription(Odometry, self.config['source_topic'], self.observe, qos_profile_sensor_data)
        self.create_timer(self.config['timer_period'], self.release)
        self.create_timer(self.config['diagnostic_period'], self.report)
        self._jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=lambda jump: self.sensor.reset())

    def observe(self, message):
        """Accept only the declared physical source/frame/epoch inside simulation."""
        t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if (message.header.frame_id != self.config['source_frame']
                or message.child_frame_id != self.config['child_frame']
                or t > now + self.config['future_tolerance']
                or now - t > self.config['max_delivery_age']):
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(quaternion).all() or not np.isclose(quaternion @ quaternion, 1.):
            return
        yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        try:
            self.sensor.observe([p.x, p.y, yaw], t)
        except ValueError:
            return

    def release(self):
        """Publish acquisition stamps, never receipt or delayed-release timestamps."""
        for stamp, value, covariance in self.sensor.release(self.get_clock().now().nanoseconds * 1e-9):
            message = Odometry()
            message.header.stamp = rclpy.time.Time(seconds=stamp).to_msg()
            message.header.frame_id = self.config['source_frame']
            message.child_frame_id = self.config['child_frame']
            message.pose.pose.position.x, message.pose.pose.position.y = value[:2].tolist()
            message.pose.pose.orientation.z = float(np.sin(value[2]/2))
            message.pose.pose.orientation.w = float(np.cos(value[2]/2))
            full = np.eye(6) * self.config['unobserved_variance']
            full[np.ix_([0, 1, 5], [0, 1, 5])] = covariance
            message.pose.covariance = full.ravel().tolist()
            message.twist.covariance = (np.eye(6)*self.config['unobserved_variance']).ravel().tolist()
            self.publisher.publish(message)

    def report(self):
        """Expose source/measurement ownership and stochastic timing counters."""
        self.diagnostics.publish(String(data=json.dumps({
            'time': self.get_clock().now().nanoseconds*1e-9,
            'model': 'imperfect_external_planar_pose_abstraction',
            'source': self.config['source_topic'], 'measurement': self.config['topic'],
            'counters': self.sensor.counters, 'pending': len(self.sensor.pending)})))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
