"""Pure construction of the currently supported robot and target estimators."""

import numpy as np
from robot_estimation.robot.planar_pose import validate_localization_parameters
from robot_estimation.state import StateEstimate
from robot_estimation.filters.kalman_filter import KalmanFilter
from robot_estimation.robot.joint_bounds import JointBoundsConditioner
from robot_estimation.robot.models.measurement_model import (
    ImuYawRateMeasurementModel, JointEncoderMeasurementModel, WheelOdometryMeasurementModel)
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.target.models.measurement_model import PositionMeasurementModel
from robot_estimation.target.models.process_model import ConstantVelocityProcessModel
from robot_estimation.target.target_state_estimator import TargetStateEstimator
from robot_estimation.timestamp_buffer import TimestampBuffer


def validate_runtime_parameters(parameters):
    """Check the existing node's publication and causal-reordering budgets."""
    validate_localization_parameters(parameters)
    rate = parameters.get('full_state_publish_rate', 0.)
    period = parameters.get('diagnostic_log_period', 5.)
    if not np.isfinite(rate) or rate < 0:
        raise ValueError('full_state_publish_rate must be finite and nonnegative')
    if not np.isfinite(period) or period <= 0:
        raise ValueError('diagnostic_log_period must be finite and positive')
    buffer = TimestampBuffer(parameters['reorder_holdback'], parameters['reorder_max_age'],
                             parameters['future_tolerance'], parameters['reorder_capacity'])
    for name in ['reorder_rate_hz', 'diagnostic_rate_hz', 'max_joint_state_age', 'max_wheel_odometry_age']:
        if not np.isfinite(parameters[name]) or parameters[name] <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if buffer.holdback + 1 / parameters['reorder_rate_hz'] >= min(
            parameters['max_joint_state_age'], parameters['max_wheel_odometry_age']):
        raise ValueError('Reorder latency must fit within component freshness budgets')

def create_robot_estimator(parameters, physical=None, estimator_type='robot_ekf'):
    """
    Construct the configured robot EKF.
    """
    if estimator_type != 'robot_ekf':
        raise ValueError(f'estimation.robot.type={estimator_type!r}: supported: robot_ekf')
    joint_count = int(parameters['joint_count'])
    process_model = RobotProcessModel(joint_count=joint_count, linear_acceleration_noise_std=float(parameters['linear_acceleration_noise_std']), angular_acceleration_noise_std=float(parameters['angular_acceleration_noise_std']), joint_acceleration_noise_std=float(parameters['joint_acceleration_noise_std']),
        planar_slip_noise_std=parameters.get('planar_slip_noise_std', 0.),
        yaw_slip_noise_std=parameters.get('yaw_slip_noise_std', 0.))
    state_dimension = process_model.state_dimension
    initial_mean = np.asarray(parameters['robot_initial_state'], dtype=float)
    covariance_diagonal = np.asarray(parameters['robot_initial_covariance_diagonal'], dtype=float)
    if initial_mean.shape != (state_dimension,):
        raise ValueError(f'robot_initial_state has invalid dimension. Expected {state_dimension}, got {initial_mean.shape}.')
    if covariance_diagonal.shape != (state_dimension,):
        raise ValueError(f'robot_initial_covariance_diagonal has invalid dimension. Expected {state_dimension}, got {covariance_diagonal.shape}.')
    if not np.all(np.isfinite(initial_mean)) or not np.all(np.isfinite(covariance_diagonal)):
        raise ValueError('Robot initial distribution must be finite')
    if np.any(covariance_diagonal < 0.0):
        raise ValueError('robot_initial_covariance_diagonal must contain non-negative values.')
    initial_state = StateEstimate(mean=initial_mean, covariance=np.diag(covariance_diagonal), timestamp=None)
    wheel_measurement_model = WheelOdometryMeasurementModel(joint_count=joint_count)
    imu_measurement_model = ImuYawRateMeasurementModel(joint_count=joint_count)
    joint_measurement_model = JointEncoderMeasurementModel(joint_count=joint_count)
    constraint = None
    if parameters['joint_bounds_enabled']:
        if len(physical) != joint_count:
            raise ValueError('Description joint order/count must match estimator layout')
        constraint = JointBoundsConditioner(
            np.arange(5, 5 + joint_count),
            [joint['lower'] for joint in physical.values()],
            [joint['upper'] for joint in physical.values()],
            parameters['joint_bounds_numerical_tolerance'],
            parameters['max_joint_state_age'])
    estimator = RobotStateEstimator(
        initial_state=initial_state, process_model=process_model,
        wheel_measurement_model=wheel_measurement_model,
        imu_measurement_model=imu_measurement_model,
        joint_measurement_model=joint_measurement_model,
        state_constraint=constraint)
    return estimator, constraint

def create_target_estimator(parameters, estimator_type='kalman'):
    """
    Construct the configured target Kalman filter.
    """
    if estimator_type != 'kalman':
        raise ValueError(f'estimation.target.type={estimator_type!r}: supported: kalman')
    spatial_dimension = int(parameters['target_spatial_dimension'])
    process_model = ConstantVelocityProcessModel(spatial_dimension=spatial_dimension, acceleration_noise_std=float(parameters['target_acceleration_noise_std']))
    measurement_model = PositionMeasurementModel(spatial_dimension=spatial_dimension)
    initial_mean = np.asarray(parameters['target_initial_state'], dtype=float)
    covariance_diagonal = np.asarray(parameters['target_initial_covariance_diagonal'], dtype=float)
    expected_state_dimension = process_model.state_dimension
    if initial_mean.shape != (expected_state_dimension,):
        raise ValueError(f'target_initial_state has invalid dimension. Expected {expected_state_dimension}, got {initial_mean.shape}.')
    if covariance_diagonal.shape != (expected_state_dimension,):
        raise ValueError(f'target_initial_covariance_diagonal has invalid dimension. Expected {expected_state_dimension}, got {covariance_diagonal.shape}.')
    if not np.all(np.isfinite(initial_mean)) or not np.all(np.isfinite(covariance_diagonal)):
        raise ValueError('Target initial distribution must be finite')
    if np.any(covariance_diagonal < 0.0):
        raise ValueError('target_initial_covariance_diagonal must contain non-negative values.')
    initial_state = StateEstimate(mean=initial_mean, covariance=np.diag(covariance_diagonal), timestamp=None)
    filter_ = KalmanFilter(initial_state=initial_state, state_transition=process_model.state_transition_matrix, process_covariance=process_model.process_covariance, measurement_matrix=measurement_model.measurement_matrix())
    return TargetStateEstimator(filter_=filter_)
