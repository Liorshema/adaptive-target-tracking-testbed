"""Pure construction of the currently supported robot and target estimators."""

import numpy as np
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

def create_robot_estimator(parameters, physical=None):
    """
    Construct the configured robot EKF.
    """
    joint_count = int(parameters['joint_count'])
    process_model = RobotProcessModel(joint_count=joint_count, linear_acceleration_noise_std=float(parameters['linear_acceleration_noise_std']), angular_acceleration_noise_std=float(parameters['angular_acceleration_noise_std']), joint_acceleration_noise_std=float(parameters['joint_acceleration_noise_std']))
    state_dimension = process_model.state_dimension
    initial_mean = np.asarray(parameters['robot_initial_state'], dtype=float)
    covariance_diagonal = np.asarray(parameters['robot_initial_covariance_diagonal'], dtype=float)
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

def create_target_estimator(parameters):
    """
    Construct the configured target Kalman filter.
    """
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
    if np.any(covariance_diagonal < 0.0):
        raise ValueError('target_initial_covariance_diagonal must contain non-negative values.')
    initial_state = StateEstimate(mean=initial_mean, covariance=np.diag(covariance_diagonal), timestamp=None)
    filter_ = KalmanFilter(initial_state=initial_state, state_transition=process_model.state_transition_matrix, process_covariance=process_model.process_covariance, measurement_matrix=measurement_model.measurement_matrix())
    return TargetStateEstimator(filter_=filter_)
