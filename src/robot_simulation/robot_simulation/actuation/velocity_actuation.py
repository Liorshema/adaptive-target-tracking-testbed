"""Fail-safe ROS adapter for body Twist and existing joint position PIDs."""

from pathlib import Path
import signal
from time import monotonic

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from robot_interfaces.msg import SafeVelocityCommand

from robot_simulation.actuation.position_reference import PositionReference

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64

import yaml


class VelocityActuationNode(Node):
    """Own actuator transport with independent command and feedback watchdogs."""

    def __init__(self):
        """Load physical limits and deployment topics without robot constants."""
        super().__init__('velocity_actuation')
        self.declare_parameter('enabled', False)
        self.declare_parameter('configuration_file', '')
        self.declare_parameter('description_file', '')
        self.declare_parameter('robot_configuration_file', '')
        self.declare_parameter('base_velocity_lower', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('base_velocity_upper', Parameter.Type.DOUBLE_ARRAY)
        self._enabled = self.get_parameter('enabled').value

        def load(parameter):
            return yaml.safe_load(Path(self.get_parameter(parameter).value).read_text())

        self._config = load('configuration_file')['actuation']
        robot = load('robot_configuration_file')['robot']
        physical = load('description_file')['robot']['manipulator']['joints']
        self._names = [robot['joint_name_template'].format(key=key) for key in physical]
        if not self._names or len(set(self._names)) != len(self._names):
            raise ValueError('Actuation joint names must be unique')
        self._reference = PositionReference(
            [j['lower'] for j in physical.values()],
            [j['upper'] for j in physical.values()],
            [j['velocity'] for j in physical.values()],
            self._config['max_position_lead'], self._config['max_integration_dt'])
        base_lower = self.get_parameter('base_velocity_lower').value
        base_upper = self.get_parameter('base_velocity_upper').value
        if len(base_lower) != 2 or len(base_upper) != 2:
            raise ValueError('Base limits must be explicit [v, omega] arrays')
        self._lower = np.r_[base_lower, -self._reference.velocity_limit]
        self._upper = np.r_[base_upper, self._reference.velocity_limit]
        if (not np.all(np.isfinite(np.r_[self._lower, self._upper])) or
                np.any(self._lower >= self._upper) or
                np.any(self._lower > 0) or np.any(self._upper < 0)):
            raise ValueError('Actuation limits must be finite and contain a safe zero')
        for name in ['rate_hz', 'command_max_age', 'feedback_max_age',
                     'wall_timeout', 'future_tolerance']:
            if not np.isfinite(self._config[name]) or self._config[name] <= 0:
                raise ValueError(f'Invalid actuation {name}')
        drive = robot['differential_drive']
        self._frame = drive['child_frame_id']
        self._odom_frame = drive['frame_id']
        self._command_topic = self._config['command_topic']
        self._session = None
        self._retired = set()
        self._sequence = -1
        self._command_stamp = None
        self._command = None
        self._joint = None
        self._odom = None
        self._last_joint_stamp = None
        self._last_odom_stamp = None
        self._last_tick = None
        self._was_active = False
        self._reason = None
        self._base_publisher = None
        self._joint_publishers = []
        self._output_topics = []
        if self._enabled:
            if not drive['enabled']:
                raise ValueError('Actuation requires the configured base drive')
            self._base_publisher = self.create_publisher(Twist, drive['topic'], 10)
            self._output_topics.append(drive['topic'])
            for name in self._names:
                topic = robot['joint_position_topic_template'].format(
                    model=robot['entity_name'], joint=name)
                self._joint_publishers.append(self.create_publisher(Float64, topic, 10))
                self._output_topics.append(topic)
        self.create_subscription(SafeVelocityCommand, self._command_topic,
                                 self._command_callback, 10)
        self.create_subscription(JointState, robot['joint_states']['topic'],
                                 self._joint_callback, 10)
        self.create_subscription(Odometry, drive['odom_topic'], self._odom_callback, 10)
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True),
            post_callback=self._on_time_jump)
        self._timer = self.create_timer(1 / self._config['rate_hz'], self._tick)
        # An independent wall clock stops the base even when simulation time pauses.
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._watchdog = self.create_timer(
            1 / self._config['rate_hz'], self._watchdog_tick, clock=self._watchdog_clock)
        self.get_logger().info(f'Velocity actuation enabled={self._enabled}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _wall():
        return monotonic()

    def _stamp(self, header):
        if header.stamp.sec < 0 or not 0 <= header.stamp.nanosec < 1_000_000_000:
            raise ValueError('Malformed timestamp')
        timestamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        if timestamp > self._now() + self._config['future_tolerance']:
            raise ValueError('Future timestamp')
        return timestamp

    def _fresh(self, sample, max_age):
        return (sample is not None and
                -self._config['future_tolerance'] <= self._now() - sample[0] <= max_age and
                self._wall() - sample[1] <= self._config['wall_timeout'])

    def _feedback_ready(self):
        return (self._fresh(self._joint, self._config['feedback_max_age']) and
                self._fresh(self._odom, self._config['feedback_max_age']))

    def _joint_callback(self, message):
        try:
            stamp = self._stamp(message.header)
            if len(set(message.name)) != len(message.name):
                raise ValueError('Duplicate measured joints')
            indices = [message.name.index(name) for name in self._names]
            measured = np.asarray([message.position[i] for i in indices])
            velocity = np.asarray([message.velocity[i] for i in indices])
            if not np.all(np.isfinite(np.r_[measured, velocity])):
                raise ValueError('Nonfinite joint feedback')
            if self._last_joint_stamp is not None and stamp < self._last_joint_stamp:
                raise ValueError('Out-of-order joint feedback')
            self._last_joint_stamp = stamp
            self._joint = (stamp, self._wall(), measured)
        except (ValueError, IndexError, TypeError) as error:
            self._joint = None
            self._halt(str(error))

    def _odom_callback(self, message):
        try:
            stamp = self._stamp(message.header)
            if (message.header.frame_id != self._odom_frame or
                    message.child_frame_id != self._frame):
                raise ValueError('Incompatible odometry frames')
            pose = message.pose.pose
            quaternion = [pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w]
            values = [pose.position.x, pose.position.y, pose.position.z,
                      *quaternion, message.twist.twist.linear.x,
                      message.twist.twist.angular.z]
            if not np.all(np.isfinite(values)):
                raise ValueError('Nonfinite base feedback')
            if np.linalg.norm(quaternion) == 0:
                raise ValueError('Invalid base orientation')
            if self._last_odom_stamp is not None and stamp < self._last_odom_stamp:
                raise ValueError('Out-of-order base feedback')
            self._last_odom_stamp = stamp
            self._odom = (stamp, self._wall())
        except (ValueError, TypeError) as error:
            self._odom = None
            self._halt(str(error))

    def _command_callback(self, message):
        if not self._enabled:
            return
        try:
            stamp = self._stamp(message.header)
            if (message.header.frame_id != self._frame or
                    list(message.joint_names) != self._names or not message.session_id):
                raise ValueError('Incompatible command identity or body frame')
            velocity = np.asarray(message.velocity, dtype=float)
            if (velocity.shape != self._lower.shape or not np.all(np.isfinite(velocity)) or
                    np.any(velocity < self._lower - 1e-9) or
                    np.any(velocity > self._upper + 1e-9)):
                raise ValueError('Unsafe generalized velocity')
            if message.session_id in self._retired:
                raise ValueError('Retired command session')
            if self._now() - stamp > self._config['command_max_age']:
                raise ValueError('Stale command')
            if len(self.get_publishers_info_by_topic(self._command_topic)) != 1:
                raise ValueError('Competing command publishers')
            if not self._owns_outputs():
                raise ValueError('Competing actuator publishers')
            if message.session_id != self._session:
                if self._session is not None:
                    self._retired.add(self._session)
                self._halt('Command source changed')
                self._session = message.session_id
                self._sequence = -1
                self._command_stamp = None
            if message.sequence <= self._sequence:
                raise ValueError('Duplicate or out-of-order command')
            if self._command_stamp is not None and stamp < self._command_stamp:
                raise ValueError('Out-of-order command timestamp')
            self._sequence = message.sequence
            self._command_stamp = stamp
            if not message.active:
                self._halt('Control invalid or disabled')
                return
            if (not self._feedback_ready() or
                    self._now() - stamp > self._config['command_max_age']):
                raise ValueError('Missing feedback, stale command or competing publishers')
            if not self._was_active:
                self._reference.initialize(self._joint[2])
                self._last_tick = self._now()
            self._command = (stamp, self._wall(), velocity)
            self._was_active = True
            self._reason = None
        except (ValueError, TypeError) as error:
            self._halt(str(error))

    def _halt(self, reason):
        self._command = None
        self._was_active = False
        self._last_tick = None
        if self._enabled:
            self._send()
            if reason != self._reason:
                self.get_logger().warning(f'Actuation holding: {reason}')
                self._reason = reason

    def _on_time_jump(self, _jump):
        if self._session is not None:
            self._retired.add(self._session)
        self._session = None
        self._sequence = -1
        self._command_stamp = None
        self._joint = None
        self._odom = None
        self._last_joint_stamp = None
        self._last_odom_stamp = None
        # Never send a pre-reset arm reference into the new physical epoch.
        self._reference.position = None
        self._halt('Clock reset: waiting for fresh feedback and command session')

    def _watchdog_tick(self):
        if self._enabled and self._command is not None:
            if (not self._fresh(self._command, self._config['command_max_age']) or
                    not self._feedback_ready() or not self._owns_outputs()):
                self._halt('Command or feedback timeout')

    def _owns_outputs(self):
        return all(len(self.get_publishers_info_by_topic(topic)) == 1
                   for topic in self._output_topics)

    def _tick(self):
        if not self._enabled:
            return
        self._watchdog_tick()
        if self._command is None:
            self._send()
            return
        now = self._now()
        dt = now - self._last_tick
        if dt <= 0:
            return
        try:
            self._reference.step(self._command[2][2:], self._joint[2], dt)
        except ValueError as error:
            self._halt(str(error))
            return
        self._last_tick = now
        self._send(self._command[2])

    def _send(self, velocity=None):
        if self._base_publisher is None:
            return
        message = Twist()
        if velocity is not None:
            message.linear.x = float(velocity[0])
            message.angular.z = float(velocity[1])
        self._base_publisher.publish(message)
        if self._reference.position is not None:
            for publisher, position in zip(self._joint_publishers, self._reference.position):
                publisher.publish(Float64(data=float(position)))


def main(args=None):
    """Run the sole downstream actuator owner and stop on graceful shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = VelocityActuationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node._halt('Shutdown')
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
