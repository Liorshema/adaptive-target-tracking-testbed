"""Fail-safe ROS adapter for body Twist and existing joint position PIDs."""

import signal
from pathlib import Path
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

from robot_interfaces.msg import ActuationTelemetry, SafeVelocityCommand

from robot_simulation.actuation.position_guard import PositionGuard
from robot_simulation.actuation.position_reference import PositionReference

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64

from robot_configuration.yaml_loader import load_yaml
from robot_simulation.factory.actuation_factory import create_actuation_components


class VelocityActuationNode(Node):
    """Own actuator transport with independent command and feedback watchdogs."""

    def __init__(self):
        """Load physical limits and deployment topics without robot constants."""
        super().__init__('velocity_actuation')
        self.declare_parameter('enabled', False)
        self.declare_parameter('configuration_file', '')
        self.declare_parameter('description_file', '')
        self.declare_parameter('robot_configuration_file', '')
        self.declare_parameter('holding_configuration_file', '')
        self.declare_parameter('base_velocity_lower', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('base_velocity_upper', Parameter.Type.DOUBLE_ARRAY)
        self._enabled = self.get_parameter('enabled').value

        def load(parameter):
            return load_yaml(Path(self.get_parameter(parameter).value))

        self._config = load('configuration_file')['actuation']
        robot = load('robot_configuration_file')['robot']
        physical = load('description_file')['robot']['manipulator']['joints']
        holding_path = self.get_parameter('holding_configuration_file').value
        holding = load_yaml(holding_path)['joint_position_hold'] if holding_path else None
        components = create_actuation_components(
            self._config, robot, physical, holding,
            self.get_parameter('base_velocity_lower').value,
            self.get_parameter('base_velocity_upper').value, enabled=self._enabled)
        self._names = components.names
        self._reference = components.reference
        self._position_guard = components.guard
        self._lower, self._upper = components.lower, components.upper
        self._recovery_heartbeat = None
        self._last_recovery_tick = None
        drive = robot['differential_drive']
        self._frame = drive['child_frame_id']
        self._odom_frame = drive['frame_id']
        self._command_topic = self._config['command_topic']
        self._session = None
        self._retired = set()
        self._sequence = -1
        self._command_stamp = None
        self._command = None
        self._accepted_identity = None
        self._joint = None
        self._joint_velocity = None
        self._joint_source_stamp = None
        self._odom = None
        self._last_joint_stamp = None
        self._last_odom_stamp = None
        self._last_tick = None
        self._was_active = False
        self._reason = None
        self._accepted_commands = 0
        self._rejected_commands = 0
        self._inactive_commands = 0
        self._feedback_rejections = 0
        self._timeout_events = 0
        self._reference_limit_events = 0
        self._diagnostic_publisher = None
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
        if self._config.get('publish_diagnostics', False):
            rate = float(self._config['diagnostic_rate_hz'])
            if not np.isfinite(rate) or rate <= 0:
                raise ValueError('Diagnostic rate must be finite and positive')
            self._diagnostic_publisher = self.create_publisher(
                ActuationTelemetry, self._config['diagnostic_topic'], 10)
            self._diagnostic_timer = self.create_timer(1 / rate, self._publish_diagnostics)
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
            self._joint_velocity = velocity
            self._joint_source_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if (self._position_guard.observe(measured) == 'fault' and
                    self._reason != self._position_guard.fault):
                self._halt(self._position_guard.fault)
        except (ValueError, IndexError, TypeError) as error:
            self._feedback_rejections += 1
            self._joint = None
            self._joint_velocity = None
            self._joint_source_stamp = None
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
            self._feedback_rejections += 1
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
                self._inactive_commands += 1
                mode = self._position_guard.mode
                reason = (self._position_guard.fault if mode == 'fault' else
                          'Position ' + mode if mode != 'normal' else
                          'Control invalid or disabled')
                self._halt(reason)
                # Only a fully validated source heartbeat can authorize recovery.
                # It still commands zero base motion and no ordinary arm motion.
                self._recovery_heartbeat = (stamp, self._wall())
                return
            if (not self._feedback_ready() or
                    self._now() - stamp > self._config['command_max_age']):
                raise ValueError('Missing feedback, stale command or competing publishers')
            mode = self._position_guard.observe(self._joint[2])
            if mode == 'fault':
                raise ValueError(self._position_guard.fault)
            if mode != 'normal':
                self._halt('Position ' + mode)
                self._recovery_heartbeat = (stamp, self._wall())
                return
            if not self._was_active:
                if self._reference.position is None:
                    self._reference.initialize(self._joint[2])
                self._last_tick = self._now()
            self._command = (stamp, self._wall(), velocity)
            self._accepted_identity = (message.session_id, message.sequence,
                                       message.header.stamp.sec, message.header.stamp.nanosec)
            self._accepted_commands += 1
            self._was_active = True
            self._reason = None
        except (ValueError, TypeError) as error:
            self._rejected_commands += 1
            self._halt(str(error))

    def _halt(self, reason):
        self._command = None
        self._recovery_heartbeat = None
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
        self._accepted_identity = None
        self._joint = None
        self._joint_velocity = None
        self._joint_source_stamp = None
        self._odom = None
        self._last_joint_stamp = None
        self._last_odom_stamp = None
        # Never send a pre-reset arm reference into the new physical epoch.
        self._reference.position = None
        self._last_recovery_tick = None
        # A clock reset retires transport state, never a latched physical fault.
        self._halt('Clock reset: waiting for fresh feedback and command session')

    def _watchdog_tick(self):
        authority = self._command if self._command is not None else self._recovery_heartbeat
        if self._enabled and authority is not None:
            if (not self._fresh(authority, self._config['command_max_age']) or
                    not self._feedback_ready() or not self._owns_outputs()):
                self._halt('Command or feedback timeout')
                self._timeout_events += 1

    def _owns_outputs(self):
        return all(len(self.get_publishers_info_by_topic(topic)) == 1
                   for topic in self._output_topics)

    def _tick(self):
        if not self._enabled:
            return
        self._watchdog_tick()
        now = self._now()
        recovery_dt = (None if self._last_recovery_tick is None
                       else now - self._last_recovery_tick)
        self._last_recovery_tick = now
        if self._feedback_ready():
            release_boundary = (self._command is None and
                                self._fresh(self._recovery_heartbeat,
                                            self._config['command_max_age']) and
                                self._owns_outputs())
            mode = self._position_guard.observe(self._joint[2],
                                                release_boundary=release_boundary)
            if mode == 'fault':
                self._halt(self._position_guard.fault)
                return
            if mode != 'normal':
                heartbeat = (self._command if self._command is not None
                             else self._recovery_heartbeat)
                self._command = None
                self._was_active = False
                self._last_tick = None
                self._recovery_heartbeat = heartbeat
                if (self._fresh(heartbeat, self._config['command_max_age']) and
                        self._owns_outputs() and recovery_dt is not None and
                        0 < recovery_dt <= self._reference.max_dt):
                    if self._reference.position is None:
                        self._reference.initialize(self._joint[2])
                    self._reference.position = self._position_guard.step(
                        self._reference.position, self._joint[2], recovery_dt)
                    self._reason = 'Position ' + mode
                self._send()
                return
        if self._command is None:
            self._send()
            return
        dt = now - self._last_tick
        if dt <= 0:
            return
        try:
            expected = self._reference.position + dt * self._command[2][2:]
            self._reference.step(self._command[2][2:], self._joint[2], dt)
            if not np.allclose(expected, self._reference.position, atol=1e-10):
                self._reference_limit_events += 1
        except ValueError as error:
            self._halt(str(error))
            return
        self._last_tick = now
        self._send(self._command[2])

    def _publish_diagnostics(self):
        """Snapshot adapter state; stamps identify feedback and accepted command sources."""
        message = ActuationTelemetry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame
        message.enabled = self._enabled
        message.active = self._command is not None
        message.reason = self._reason or ''
        message.joint_names = self._names
        message.accepted_command_valid = self._accepted_identity is not None
        if self._accepted_identity is not None:
            session, sequence, seconds, nanoseconds = self._accepted_identity
            message.accepted_session_id = session
            message.accepted_sequence = sequence
            message.accepted_stamp.sec = seconds
            message.accepted_stamp.nanosec = nanoseconds
        if self._joint is not None and self._position_guard.mode != 'normal':
            message.reason = (self._position_guard.fault or
                              'Position ' + self._position_guard.mode)
            message.reason += f' events={self._position_guard.events}: ' + (
                self._position_guard.diagnostic(self._names, self._joint[2]))
        if self._command is not None:
            message.commanded_velocity = self._command[2].tolist()
        if self._reference.position is not None:
            message.position_reference = self._reference.position.tolist()
        if self._joint is not None:
            message.measured_positions = self._joint[2].tolist()
            message.measured_velocities = self._joint_velocity.tolist()
            message.feedback_stamp.sec, message.feedback_stamp.nanosec = self._joint_source_stamp
            message.feedback_valid = self._fresh(self._joint, self._config['feedback_max_age'])
            if self._reference.position is not None:
                message.position_errors = (self._reference.position - self._joint[2]).tolist()
        for name in ['accepted_commands', 'rejected_commands', 'inactive_commands',
                     'feedback_rejections', 'timeout_events', 'reference_limit_events']:
            setattr(message, name, getattr(self, '_' + name))
        self._diagnostic_publisher.publish(message)

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
