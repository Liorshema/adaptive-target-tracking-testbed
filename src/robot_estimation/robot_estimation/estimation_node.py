from typing import Optional
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from robot_interfaces.msg import StateEstimate as StateEstimateMsg
from robot_estimation.common.measurement import Measurement
from robot_estimation.common.state import StateEstimate
from robot_estimation.filters.kalman_filter import KalmanFilter
from robot_estimation.robot.models.measurement_model import ImuYawRateMeasurementModel, JointEncoderMeasurementModel, WheelOdometryMeasurementModel
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.target.models.measurement_model import PositionMeasurementModel
from robot_estimation.target.models.process_model import ConstantVelocityProcessModel
from robot_estimation.target.target_state_estimator import TargetStateEstimator

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
        self._robot_estimator = self._create_robot_estimator()
        self._target_estimator = self._create_target_estimator()
        self._last_robot_time: Optional[float] = None
        self._last_target_time: Optional[float] = None
        self._target_frame_id = ""
        self._create_publishers()
        self._create_subscriptions()
        self.get_logger().info('Estimation node initialized.')

    # Parameter schemas define types, never experiment-specific values.
    _PARAMETERS = {
        "joint_count": Parameter.Type.INTEGER,
        "robot_initial_state": Parameter.Type.DOUBLE_ARRAY,
        "robot_initial_covariance_diagonal": Parameter.Type.DOUBLE_ARRAY,
        "linear_acceleration_noise_std": Parameter.Type.DOUBLE,
        "angular_acceleration_noise_std": Parameter.Type.DOUBLE,
        "joint_acceleration_noise_std": Parameter.Type.DOUBLE,
        "wheel_linear_velocity_variance": Parameter.Type.DOUBLE,
        "wheel_angular_velocity_variance": Parameter.Type.DOUBLE,
        "imu_yaw_rate_variance": Parameter.Type.DOUBLE,
        "joint_position_variance": Parameter.Type.DOUBLE,
        "joint_velocity_variance": Parameter.Type.DOUBLE,
        "target_spatial_dimension": Parameter.Type.INTEGER,
        "target_initial_state": Parameter.Type.DOUBLE_ARRAY,
        "target_initial_covariance_diagonal": Parameter.Type.DOUBLE_ARRAY,
        "target_acceleration_noise_std": Parameter.Type.DOUBLE,
        "target_position_variance": Parameter.Type.DOUBLE_ARRAY,
        "imu_topic": Parameter.Type.STRING,
        "wheel_odometry_topic": Parameter.Type.STRING,
        "joint_states_topic": Parameter.Type.STRING,
        "target_measurement_topic": Parameter.Type.STRING,
        "robot_state_topic": Parameter.Type.STRING,
        "target_state_topic": Parameter.Type.STRING,
        "target_state_full_topic": Parameter.Type.STRING,
    }

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

    def _create_robot_estimator(self) -> RobotStateEstimator:
        """
        Construct the configured robot EKF.
        """
        joint_count = int(self.get_parameter('joint_count').value)
        process_model = RobotProcessModel(joint_count=joint_count, linear_acceleration_noise_std=float(self.get_parameter('linear_acceleration_noise_std').value), angular_acceleration_noise_std=float(self.get_parameter('angular_acceleration_noise_std').value), joint_acceleration_noise_std=float(self.get_parameter('joint_acceleration_noise_std').value))
        state_dimension = process_model.state_dimension
        initial_mean = np.asarray(self.get_parameter('robot_initial_state').value, dtype=float)
        covariance_diagonal = np.asarray(self.get_parameter('robot_initial_covariance_diagonal').value, dtype=float)
        if initial_mean.shape != (state_dimension,):
            raise ValueError(f'robot_initial_state has invalid dimension. Expected {state_dimension}, got {initial_mean.shape}.')
        if covariance_diagonal.shape != (state_dimension,):
            raise ValueError(f'robot_initial_covariance_diagonal has invalid dimension. Expected {state_dimension}, got {covariance_diagonal.shape}.')
        if np.any(covariance_diagonal < 0.0):
            raise ValueError('robot_initial_covariance_diagonal must contain non-negative values.')
        initial_state = StateEstimate(mean=initial_mean, covariance=np.diag(covariance_diagonal), timestamp=None)
        wheel_measurement_model = WheelOdometryMeasurementModel(joint_count=joint_count)
        imu_measurement_model = ImuYawRateMeasurementModel(joint_count=joint_count)
        joint_measurement_model = JointEncoderMeasurementModel(joint_count=joint_count)
        return RobotStateEstimator(initial_state=initial_state, process_model=process_model, wheel_measurement_model=wheel_measurement_model, imu_measurement_model=imu_measurement_model, joint_measurement_model=joint_measurement_model)

    def _create_target_estimator(self) -> TargetStateEstimator:
        """
        Construct the configured target Kalman filter.
        """
        spatial_dimension = int(self.get_parameter('target_spatial_dimension').value)
        process_model = ConstantVelocityProcessModel(spatial_dimension=spatial_dimension, acceleration_noise_std=float(self.get_parameter('target_acceleration_noise_std').value))
        measurement_model = PositionMeasurementModel(spatial_dimension=spatial_dimension)
        initial_mean = np.asarray(self.get_parameter('target_initial_state').value, dtype=float)
        covariance_diagonal = np.asarray(self.get_parameter('target_initial_covariance_diagonal').value, dtype=float)
        expected_state_dimension = process_model.state_dimension
        if initial_mean.shape != (expected_state_dimension,):
            raise ValueError(f'target_initial_state has invalid dimension. Expected {expected_state_dimension}, got {initial_mean.shape}.')
        if covariance_diagonal.shape != (expected_state_dimension,):
            raise ValueError(f'target_initial_covariance_diagonal has invalid dimension. Expected {expected_state_dimension}, got {covariance_diagonal.shape}.')
        if np.any(covariance_diagonal < 0.0):
            raise ValueError('target_initial_covariance_diagonal must contain non-negative values.')
        initial_state = StateEstimate(mean=initial_mean, covariance=np.diag(covariance_diagonal), timestamp=None)
        filter_ = KalmanFilter(initial_state=initial_state, state_transition=process_model.state_transition_matrix, process_covariance=process_model.process_covariance, measurement_matrix=measurement_model.measurement_matrix())
        return TargetStateEstimator(filter_=filter_)

    def _create_publishers(self) -> None:
        """
        Create robot and target state publishers.
        """
        robot_state_topic = str(self.get_parameter('robot_state_topic').value)
        target_state_topic = str(self.get_parameter('target_state_topic').value)
        self._robot_state_publisher = self.create_publisher(Float64MultiArray, robot_state_topic, 10)
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
        self._predict_robot_if_needed(timestamp)
        variance = float(self.get_parameter('imu_yaw_rate_variance').value)
        if variance < 0.0:
            self.get_logger().error('imu_yaw_rate_variance must be non-negative.')
            return
        measurement = Measurement(value=np.array([msg.angular_velocity.z], dtype=float), covariance=np.array([[variance]], dtype=float), timestamp=timestamp)
        self._robot_estimator.update_imu(measurement)
        self._publish_robot_state()

    def _wheel_odometry_callback(self, msg: Odometry) -> None:
        """
        Fuse wheel-derived linear and angular velocity.
        """
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        self._predict_robot_if_needed(timestamp)
        linear_variance = float(self.get_parameter('wheel_linear_velocity_variance').value)
        angular_variance = float(self.get_parameter('wheel_angular_velocity_variance').value)
        if linear_variance < 0.0 or angular_variance < 0.0:
            self.get_logger().error('Wheel odometry variances must be non-negative.')
            return
        measurement = Measurement(value=np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z], dtype=float), covariance=np.diag([linear_variance, angular_variance]), timestamp=timestamp)
        self._robot_estimator.update_wheel_odometry(measurement)
        self._publish_robot_state()

    def _joint_state_callback(self, msg: JointState) -> None:
        """
        Fuse joint positions and velocities.
        """
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        self._predict_robot_if_needed(timestamp)
        joint_count = int(self.get_parameter('joint_count').value)
        positions = np.asarray(msg.position, dtype=float)
        velocities = np.asarray(msg.velocity, dtype=float)
        if positions.shape != (joint_count,):
            self.get_logger().warning(f'Ignoring JointState message with {positions.shape[0]} positions. Expected {joint_count}.')
            return
        if velocities.shape != (joint_count,):
            self.get_logger().warning(f'Ignoring JointState message with {velocities.shape[0]} velocities. Expected {joint_count}.')
            return
        position_variance = float(self.get_parameter('joint_position_variance').value)
        velocity_variance = float(self.get_parameter('joint_velocity_variance').value)
        if position_variance < 0.0 or velocity_variance < 0.0:
            self.get_logger().error('Joint encoder variances must be non-negative.')
            return
        covariance_diagonal = np.concatenate([np.full(joint_count, position_variance), np.full(joint_count, velocity_variance)])
        measurement = Measurement(value=np.concatenate([positions, velocities]), covariance=np.diag(covariance_diagonal), timestamp=timestamp)
        self._robot_estimator.update_joint_encoders(measurement)
        self._publish_robot_state()

    def _target_measurement_callback(self, msg: PointStamped) -> None:
        """
        Fuse a target position measurement.
        """
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        self._predict_target_if_needed(timestamp)
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
        self._target_estimator.update(measurement)
        self._target_frame_id = msg.header.frame_id
        self._publish_target_state()

    def _predict_robot_if_needed(self, timestamp: float) -> None:
        """
        Predict robot state to the incoming measurement time.
        """
        if self._last_robot_time is None:
            self._last_robot_time = timestamp
            return
        dt = timestamp - self._last_robot_time
        if dt <= 0.0:
            return
        self._robot_estimator.predict(dt=dt)
        self._last_robot_time = timestamp

    def _predict_target_if_needed(self, timestamp: float) -> None:
        """
        Predict target state to the incoming measurement time.
        """
        if self._last_target_time is None:
            self._last_target_time = timestamp
            return
        dt = timestamp - self._last_target_time
        if dt <= 0.0:
            return
        self._target_estimator.predict(dt=dt)
        self._last_target_time = timestamp

    def _publish_robot_state(self) -> None:
        """
        Publish current robot-state mean.
        """
        state = self._robot_estimator.state
        msg = Float64MultiArray()
        msg.data = state.mean.tolist()
        self._robot_state_publisher.publish(msg)

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
