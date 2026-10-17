"""ROS 2 orchestration for robot and target state estimation."""

import json
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PointStamped

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_estimation.parameter_schema import PARAMETERS

from robot_estimation.measurement import Measurement
from robot_estimation.timestamp_buffer import TimestampBuffer
from robot_estimation.factory.estimator_factory import (
    create_robot_estimator, create_target_estimator, validate_runtime_parameters)
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.target.target_state_estimator import TargetStateEstimator

from robot_interfaces.msg import StateEstimate as StateEstimateMsg

from sensor_msgs.msg import Imu, JointState

from std_msgs.msg import Float64MultiArray, String

import yaml


class EstimationNode(Node):
    """
    ROS 2 orchestration node for robot and target state estimation.
    The node converts ROS messages into generic Measurement objects
    and delegates all estimation mathematics to the estimation layer.
    Robot state:
        x_R = [
            x_B,
            y_B,
            theta_B,
            v_B,
            omega_B,
            q,
            q_dot
        ]^T
    Target state:
        x_T = [
            p,
            v
        ]^T
    """

    def __init__(self) -> None:
        super().__init__('estimation_node')
        self._declare_parameters()
        self.declare_parameter('robot_estimator_type', 'robot_ekf')
        self.declare_parameter('target_estimator_type', 'kalman')
        self.declare_parameter('full_state_publish_rate', 0.)
        self.declare_parameter('diagnostic_log_period', 5.)
        validate_runtime_parameters({name: self.get_parameter(name).value for name in
                                     [*self._PARAMETERS, 'full_state_publish_rate', 'diagnostic_log_period']})
        rate = float(self.get_parameter('full_state_publish_rate').value)
        period = float(self.get_parameter('diagnostic_log_period').value)
        if not np.isfinite(rate) or rate < 0 or not np.isfinite(period) or period <= 0:
            raise ValueError('Invalid publication rate or diagnostic period')
        self._full_state_period = 0. if rate == 0 else 1 / rate
        self._last_full_publish_time = None
        self._diagnostic_log_period = period
        self._last_time_warning = None
        self._time_rejections = 0
        self._robot_estimator = self._create_robot_estimator()
        self._target_estimator = self._create_target_estimator()
        self._last_robot_time: Optional[float] = None
        self._last_target_time: Optional[float] = None
        self._last_wheel_time: Optional[float] = None
        self._last_joint_time: Optional[float] = None
        self._target_frame_id = ""
        self._clock_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._on_time_jump,
        )
        self._configure_robot_timing()
        self._robot_buffer_timer = self.create_timer(
            1 / self.get_parameter('reorder_rate_hz').value, self._drain_robot)
        self._create_publishers()
        self._diagnostic_publisher = self.create_publisher(
            String, self.get_parameter('diagnostic_topic').value, 10)
        self._diagnostic_timer = self.create_timer(
            1 / self.get_parameter('diagnostic_rate_hz').value, self._publish_diagnostics)
        self._create_subscriptions()
        self.get_logger().info(
            f'Estimation node initialized: robot={type(self._robot_estimator).__name__}, '
            f'target={type(self._target_estimator).__name__}.')

    # Parameter schemas define types, never experiment-specific values.
    _PARAMETERS = {name: getattr(Parameter.Type, kind) for name, kind in PARAMETERS.items()}

    def _declare_parameters(self) -> None:
        """Declare strictly typed parameters supplied by ROS YAML overrides."""
        for name, parameter_type in self._PARAMETERS.items():
            self.declare_parameter(name, parameter_type)
        missing = [
            name for name in self._PARAMETERS
            if self.get_parameter(name).value is None
        ]
        if missing:
            raise ValueError(
                "Missing required estimator parameters: "
                + ", ".join(missing)
                + ". Load both robot_estimator.yaml and target_estimator.yaml."
            )
        joint_names = list(self.get_parameter('joint_names').value)
        if (
            len(joint_names) != int(self.get_parameter('joint_count').value)
            or len(set(joint_names)) != len(joint_names)
            or any(not name for name in joint_names)
        ):
            raise ValueError('joint_names must contain one unique name per arm joint.')
        tolerance = float(self.get_parameter('future_tolerance').value)
        if not np.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError('future_tolerance must be finite and nonnegative.')
        for name in ['max_wheel_odometry_age', 'max_joint_state_age']:
            age = float(self.get_parameter(name).value)
            if not np.isfinite(age) or age <= 0.0:
                raise ValueError(f'{name} must be finite and positive.')

    def _create_robot_estimator(self) -> RobotStateEstimator:
        """Resolve the physical description and delegate pure filter assembly."""
        parameters = {name: self.get_parameter(name).value for name in self._PARAMETERS}
        physical = None
        if parameters['joint_bounds_enabled']:
            path = parameters['robot_description_file']
            if not path:
                path = Path(get_package_share_directory(
                    parameters['robot_description_package'])) / parameters['robot_description_config']
            physical = yaml.safe_load(Path(path).read_text())['robot']['manipulator']['joints']
        estimator, self._joint_constraint = create_robot_estimator(
            parameters, physical, self.get_parameter('robot_estimator_type').value)
        return estimator

    def _create_target_estimator(self) -> TargetStateEstimator:
        """Delegate target-filter assembly without moving ROS resource policy."""
        return create_target_estimator({
            name: self.get_parameter(name).value for name in self._PARAMETERS},
            self.get_parameter('target_estimator_type').value)

    def _create_publishers(self) -> None:
        """
        Create robot and target state publishers.
        """
        robot_state_topic = str(self.get_parameter('robot_state_topic').value)
        target_state_topic = str(self.get_parameter('target_state_topic').value)
        self._robot_state_publisher = self.create_publisher(Float64MultiArray, robot_state_topic, 10)
        self._robot_state_full_publisher = self.create_publisher(
            StateEstimateMsg,
            str(self.get_parameter('robot_state_full_topic').value),
            10,
        )
        self._target_state_publisher = self.create_publisher(Float64MultiArray, target_state_topic, 10)
        self._target_state_full_publisher = self.create_publisher(
            StateEstimateMsg,
            str(self.get_parameter('target_state_full_topic').value),
            10,
        )

    def _create_subscriptions(self) -> None:
        """
        Create robot-sensor and target-measurement subscriptions.
        """
        self.create_subscription(Imu, str(self.get_parameter('imu_topic').value), self._imu_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter('wheel_odometry_topic').value), self._wheel_odometry_callback, 10)
        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value), self._joint_state_callback, 10)
        self.create_subscription(PointStamped, str(self.get_parameter('target_measurement_topic').value), self._target_measurement_callback, 10)

    def _imu_callback(self, msg: Imu) -> None:
        """
        Fuse planar IMU yaw-rate measurement.
        """
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        if msg.header.frame_id != self.get_parameter('imu_frame').value:
            self._reject_robot_input('imu')
            return
        variance = float(self.get_parameter('imu_yaw_rate_variance').value)
        if variance < 0.0:
            self.get_logger().error('imu_yaw_rate_variance must be non-negative.')
            return
        measurement = Measurement(value=np.array([msg.angular_velocity.z], dtype=float), covariance=np.array([[variance]], dtype=float), timestamp=timestamp)
        if self._measurement_valid(measurement):
            self._queue_robot('imu', measurement)
        else:
            self._reject_robot_input('imu')

    def _wheel_odometry_callback(self, msg: Odometry) -> None:
        """
        Fuse wheel-derived linear and angular velocity.
        """
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        if (msg.header.frame_id != self.get_parameter('wheel_odom_frame').value or
                msg.child_frame_id != self.get_parameter('base_frame').value):
            self._reject_robot_input('wheel')
            return
        linear_variance = float(self.get_parameter('wheel_linear_velocity_variance').value)
        angular_variance = float(self.get_parameter('wheel_angular_velocity_variance').value)
        if linear_variance < 0.0 or angular_variance < 0.0:
            self.get_logger().error('Wheel odometry variances must be non-negative.')
            return
        measurement = Measurement(value=np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z], dtype=float), covariance=np.diag([linear_variance, angular_variance]), timestamp=timestamp)
        if self._measurement_valid(measurement):
            self._queue_robot('wheel', measurement)
        else:
            self._reject_robot_input('wheel')

    def _joint_state_callback(self, msg: JointState) -> None:
        """
        Fuse joint positions and velocities.
        """
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        joint_names = list(self.get_parameter('joint_names').value)
        joint_count = len(joint_names)
        try:
            names = list(msg.name)
            if len(set(names)) != len(names):
                raise ValueError('JointState contains duplicate joint names.')
            if len(msg.position) != len(names) or len(msg.velocity) != len(names):
                raise ValueError('JointState requires position and velocity for each named joint.')
            indices = [names.index(name) for name in joint_names]
            positions = np.asarray(msg.position, dtype=float)[indices]
            velocities = np.asarray(msg.velocity, dtype=float)[indices]
            if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)):
                raise ValueError('JointState arm measurements must be finite.')
        except ValueError as error:
            self._reject_robot_input('joints')
            self.get_logger().warning(
                f'Ignoring invalid JointState: {error}', throttle_duration_sec=5.0,
            )
            return
        position_variance = float(self.get_parameter('joint_position_variance').value)
        velocity_variance = float(self.get_parameter('joint_velocity_variance').value)
        if position_variance < 0.0 or velocity_variance < 0.0:
            self.get_logger().error('Joint encoder variances must be non-negative.')
            return
        covariance_diagonal = np.concatenate([np.full(joint_count, position_variance), np.full(joint_count, velocity_variance)])
        measurement = Measurement(value=np.concatenate([positions, velocities]), covariance=np.diag(covariance_diagonal), timestamp=timestamp)
        if self._measurement_valid(measurement):
            self._queue_robot('joints', measurement)
        else:
            self._reject_robot_input('joints')

    def _target_measurement_callback(self, msg: PointStamped) -> None:
        """
        Fuse a target position measurement.
        """
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        spatial_dimension = int(self.get_parameter('target_spatial_dimension').value)
        if spatial_dimension != 3:
            self.get_logger().error('PointStamped currently supports target_spatial_dimension=3.')
            return
        position = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        variances = np.asarray(self.get_parameter('target_position_variance').value, dtype=float)
        if variances.shape != (spatial_dimension,):
            self.get_logger().error(f'target_position_variance has invalid dimension. Expected {spatial_dimension}, got {variances.shape}.')
            return
        if np.any(variances < 0.0):
            self.get_logger().error('target_position_variance must contain non-negative values.')
            return
        measurement = Measurement(value=position, covariance=np.diag(variances), timestamp=timestamp)
        if not msg.header.frame_id or (
            self._target_frame_id and msg.header.frame_id != self._target_frame_id
        ):
            self.get_logger().error('Target measurement frame is empty or changed without a transform.')
            return
        if not self._measurement_valid(measurement) or not self._predict_target_if_needed(timestamp):
            return
        self._target_estimator.update(measurement)
        self._target_frame_id = msg.header.frame_id
        self._publish_target_state()

    def _measurement_valid(self, measurement: Measurement) -> bool:
        """Reject nonfinite sensor data before prediction or correction."""
        if (
            not np.all(np.isfinite(measurement.value))
            or not np.all(np.isfinite(measurement.covariance))
        ):
            self.get_logger().warning(
                'Ignoring nonfinite sensor measurement.', throttle_duration_sec=5.0,
            )
            return False
        return True

    def _on_time_jump(self, _jump) -> None:
        """Reinitialize both filters when ROS time changes epoch."""
        self._robot_estimator = self._create_robot_estimator()
        self._target_estimator = self._create_target_estimator()
        self._last_robot_time = None
        self._last_target_time = None
        self._last_wheel_time = None
        self._last_joint_time = None
        self._target_frame_id = ''
        self._last_full_publish_time = None
        self._last_time_warning = None
        self._robot_buffer.reset()
        self._raw_joint_time = None
        self._raw_joint_valid = None
        self.get_logger().warning('ROS clock changed; reinitialized estimation filters.')

    def _measurement_time_valid(
        self, timestamp: float, previous: Optional[float], name: str,
    ) -> bool:
        """Reject delayed and former-epoch measurements before filter mutation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        tolerance = float(self.get_parameter('future_tolerance').value)
        if (
            not np.isfinite(timestamp)
            or timestamp > now + tolerance
            or (previous is not None and timestamp < previous)
        ):
            self._time_rejections = getattr(self, '_time_rejections', 0) + 1
            last = getattr(self, '_last_time_warning', None)
            interval = getattr(self, '_diagnostic_log_period', 5.)
            if last is None or now < last or now - last >= interval:
                self._last_time_warning = now
                self.get_logger().warning(
                    f'Ignoring {name} measurement time: '
                    f'stamp={timestamp:.6f}, last={previous}, now={now:.6f}; '
                    f'total timestamp rejections={self._time_rejections}',
                )
            return False
        return True

    def _configure_robot_timing(self):
        """Create one generic queue for the shared-time robot filter."""
        self._robot_buffer = TimestampBuffer(
            self.get_parameter('reorder_holdback').value,
            self.get_parameter('reorder_max_age').value,
            self.get_parameter('future_tolerance').value,
            self.get_parameter('reorder_capacity').value)
        for name in ['reorder_rate_hz', 'diagnostic_rate_hz']:
            value = self.get_parameter(name).value
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self._robot_buffer.holdback + 1 / self.get_parameter('reorder_rate_hz').value >= min(
                self.get_parameter('max_joint_state_age').value,
                self.get_parameter('max_wheel_odometry_age').value):
            raise ValueError('Reorder latency must fit within component freshness budgets')
        self._raw_joint_time = None
        self._raw_joint_valid = None
        self._fusion_errors = 0

    def _reject_robot_input(self, source):
        """Count malformed/frame-invalid observations that cannot enter the queue."""
        self._robot_buffer.counters[source]['received'] += 1
        self._robot_buffer.counters[source]['invalid'] += 1

    def _queue_robot(self, source, measurement):
        """Buffer valid transport while preserving its original measurement time."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._robot_buffer.push(source, measurement.timestamp, measurement, now):
            self._time_rejections = getattr(self, '_time_rejections', 0) + 1
            return
        if source == 'joints' and self._joint_constraint is not None:
            if self._raw_joint_time is None or measurement.timestamp >= self._raw_joint_time:
                q = measurement.value[:len(self._joint_constraint.indices)]
                excess = np.maximum(np.maximum(self._joint_constraint.lower - q,
                                               q - self._joint_constraint.upper), 0.)
                self._raw_joint_valid = bool(np.all(excess <= self._joint_constraint.tolerance))
                self._raw_joint_time = measurement.timestamp
        if self._robot_buffer.holdback == 0:
            self._drain_robot()

    def _drain_robot(self):
        """Fuse ordered observations at their stamps, then publish the committed state."""
        now = self.get_clock().now().nanoseconds * 1e-9
        changed, failed = False, False
        for source, timestamp, measurement in self._robot_buffer.pop_ready(now):
            try:
                if not self._predict_robot_if_needed(timestamp):
                    continue
                if source == 'wheel':
                    self._robot_estimator.update_wheel_odometry(measurement)
                    self._last_wheel_time = timestamp
                elif source == 'joints':
                    self._robot_estimator.update_joint_encoders(measurement)
                    self._last_joint_time = timestamp
                elif source == 'imu':
                    self._robot_estimator.update_imu(measurement)
                else:
                    raise ValueError('Unsupported robot observation model')
                changed = True
            except (ValueError, np.linalg.LinAlgError) as error:
                failed = True
                self._fusion_errors += 1
                self.get_logger().error(f'Robot fusion failed: {error}', throttle_duration_sec=5.)
        if changed and not failed:
            self._publish_robot_state()

    def _publish_diagnostics(self):
        """Publish timing/constraint evidence without making ground truth a sensor."""
        state = self._robot_estimator.state
        constraint = self._joint_constraint
        data = {
            'time': self.get_clock().now().nanoseconds * 1e-9,
            'filter_time': state.timestamp, 'timing': self._robot_buffer.diagnostics(),
            'fusion_errors': self._fusion_errors, 'raw_joint_valid': self._raw_joint_valid,
            'raw_joint_time': self._raw_joint_time,
        }
        if constraint is not None:
            data['joint_bounds'] = {
                'physical_valid': constraint.physical_valid,
                'positions': (None if constraint.positions is None
                              else constraint.positions.tolist()),
                'excess_rad': constraint.excess.tolist(),
                'conditionings': constraint.conditionings,
                'physical_violation_samples': constraint.physical_violation_samples,
            }
        self._diagnostic_publisher.publish(String(data=json.dumps(data)))

    def _predict_robot_if_needed(self, timestamp: float) -> bool:
        """
        Predict robot state to the incoming measurement time.
        """
        if not self._measurement_time_valid(timestamp, self._last_robot_time, 'robot'):
            return False
        if self._last_robot_time is None:
            self._last_robot_time = timestamp
            return True
        dt = timestamp - self._last_robot_time
        if dt <= 0.0:
            return True
        self._robot_estimator.predict(dt=dt)
        self._last_robot_time = timestamp
        return True

    def _predict_target_if_needed(self, timestamp: float) -> bool:
        """
        Predict target state to the incoming measurement time.
        """
        if not self._measurement_time_valid(timestamp, self._last_target_time, 'target'):
            return False
        if self._last_target_time is None:
            self._last_target_time = timestamp
            return True
        dt = timestamp - self._last_target_time
        if dt <= 0.0:
            return True
        self._target_estimator.predict(dt=dt)
        self._last_target_time = timestamp
        return True

    def _publish_robot_state(self) -> None:
        """Publish the legacy state vector and the full stamped estimate."""
        state = self._robot_estimator.state
        legacy_msg = Float64MultiArray()
        legacy_msg.data = state.mean.tolist()
        self._robot_state_publisher.publish(legacy_msg)
        if self._joint_constraint is not None and (
                not self._joint_constraint.physical_valid or self._raw_joint_valid is False):
            self.get_logger().warning(
                'Full robot estimate unavailable: raw joint position violates physical bounds.',
                throttle_duration_sec=5.)
            return

        # A fresh aggregate filter timestamp alone does not prove all
        # configuration components have been observed recently.
        now = self.get_clock().now().nanoseconds * 1e-9
        for name, timestamp, max_age in [
            ('wheel odometry', self._last_wheel_time,
             float(self.get_parameter('max_wheel_odometry_age').value)),
            ('joint states', self._last_joint_time,
             float(self.get_parameter('max_joint_state_age').value)),
        ]:
            if timestamp is None or now - timestamp > max_age:
                self.get_logger().warning(
                    f'Full robot estimate unavailable: waiting for fresh {name}.',
                    throttle_duration_sec=5.0,
                )
                return

        if state.timestamp is None:
            self.get_logger().warning(
                'Robot estimate has no timestamp; skipping full publication.'
            )
            return

        timestamp = float(state.timestamp)
        previous = getattr(self, '_last_full_publish_time', None)
        period = getattr(self, '_full_state_period', 0.)
        if previous is not None and period > 0 and 0 <= timestamp - previous < period - 1e-9:
            return
        mean = np.asarray(state.mean, dtype=float)
        covariance = np.asarray(state.covariance, dtype=float)

        if (
            not np.isfinite(timestamp)
            or mean.ndim != 1
            or not np.all(np.isfinite(mean))
            or covariance.shape != (mean.size, mean.size)
            or not np.all(np.isfinite(covariance))
        ):
            self.get_logger().error(
                'Invalid robot estimate; skipping full publication.'
            )
            return

        frame_id = str(self.get_parameter('robot_state_frame_id').value)
        if not frame_id:
            self.get_logger().error(
                'Robot state frame_id is empty; skipping full publication.'
            )
            return

        full_msg = StateEstimateMsg()
        full_msg.header.stamp = rclpy.time.Time(
            seconds=timestamp
        ).to_msg()
        full_msg.header.frame_id = frame_id
        full_msg.mean = mean.tolist()
        full_msg.covariance = covariance.ravel(order='C').tolist()
        self._robot_state_full_publisher.publish(full_msg)
        self._last_full_publish_time = timestamp

    def _publish_target_state(self) -> None:
        """
        Publish current target-state mean.
        """
        state = self._target_estimator.state
        legacy_msg = Float64MultiArray()
        legacy_msg.data = state.mean.tolist()
        self._target_state_publisher.publish(legacy_msg)
        if state.timestamp is None:
            self.get_logger().warning('Full target estimate has no timestamp; skipping.')
            return
        full_msg = StateEstimateMsg()
        full_msg.header.stamp = rclpy.time.Time(seconds=float(state.timestamp)).to_msg()
        full_msg.header.frame_id = self._target_frame_id
        full_msg.mean = state.mean.tolist()
        full_msg.covariance = state.covariance.ravel(order='C').tolist()
        self._target_state_full_publisher.publish(full_msg)

    @staticmethod

    def _stamp_to_seconds(stamp) -> float:
        """
        Convert a ROS time stamp to floating-point seconds.
        """
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            return float('nan')
        return float(stamp.sec) + float(stamp.nanosec) * 1e-09


def main(args=None) -> None:
    """
    ROS 2 node entry point.
    """
    rclpy.init(args=args)
    node = EstimationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
