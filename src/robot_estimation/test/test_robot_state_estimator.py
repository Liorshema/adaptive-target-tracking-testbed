import numpy as np

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate
from robot_estimation.robot.models.measurement_model import (
    ImuYawRateMeasurementModel,
    JointEncoderMeasurementModel,
    WheelOdometryMeasurementModel,
)
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator


def create_estimator() -> RobotStateEstimator:
    joint_count = 4

    process_model = RobotProcessModel(
        joint_count=joint_count,
        linear_acceleration_noise_std=0.1,
        angular_acceleration_noise_std=0.05,
        joint_acceleration_noise_std=0.1,
    )

    wheel_model = WheelOdometryMeasurementModel(
        joint_count=joint_count,
    )

    imu_model = ImuYawRateMeasurementModel(
        joint_count=joint_count,
    )

    joint_model = JointEncoderMeasurementModel(
        joint_count=joint_count,
    )

    initial_state = StateEstimate(
        mean=np.array([
            0.0,
            0.0,
            0.0,
            1.0,
            0.2,

            0.1,
            0.2,
            0.3,
            0.4,

            0.01,
            0.02,
            0.03,
            0.04,
        ]),
        covariance=np.eye(13),
        timestamp=0.0,
    )

    return RobotStateEstimator(
        initial_state=initial_state,
        process_model=process_model,
        wheel_measurement_model=wheel_model,
        imu_measurement_model=imu_model,
        joint_measurement_model=joint_model,
    )


def test_initial_state_is_available() -> None:
    estimator = create_estimator()

    assert estimator.state.dimension == 13
    assert estimator.state.timestamp == 0.0


def test_prediction_propagates_robot_state() -> None:
    estimator = create_estimator()

    predicted = estimator.predict(
        dt=1.0,
    )

    expected = np.array([
        np.sin(.2) / .2,
        (1 - np.cos(.2)) / .2,
        0.2,
        1.0,
        0.2,

        0.11,
        0.22,
        0.33,
        0.44,

        0.01,
        0.02,
        0.03,
        0.04,
    ])

    assert np.allclose(
        predicted.mean,
        expected,
    )


def test_wheel_odometry_update_corrects_base_velocity() -> None:
    estimator = create_estimator()

    estimator.predict(
        dt=1.0,
    )

    measurement = Measurement(
        value=np.array([
            1.1,
            0.15,
        ]),
        covariance=np.diag([
            0.05,
            0.02,
        ]),
        timestamp=1.0,
    )

    updated = estimator.update_wheel_odometry(
        measurement,
    )

    assert updated.mean[3] > 1.0
    assert updated.mean[3] < 1.1

    assert updated.mean[4] < 0.2
    assert updated.mean[4] > 0.15


def test_imu_update_corrects_yaw_rate() -> None:
    estimator = create_estimator()

    estimator.predict(
        dt=1.0,
    )

    predicted_yaw_rate = estimator.state.mean[4]

    measurement = Measurement(
        value=np.array([
            0.15,
        ]),
        covariance=np.array([
            [0.01],
        ]),
        timestamp=1.0,
    )

    updated = estimator.update_imu(
        measurement,
    )

    assert updated.mean[4] < predicted_yaw_rate
    assert updated.mean[4] > 0.15


def test_joint_encoder_update_corrects_joint_state() -> None:
    estimator = create_estimator()

    estimator.predict(
        dt=1.0,
    )

    measurement = Measurement(
        value=np.array([
            0.12,
            0.21,
            0.34,
            0.43,

            0.015,
            0.018,
            0.028,
            0.045,
        ]),
        covariance=0.02 * np.eye(8),
        timestamp=1.0,
    )

    predicted_joint_state = (
        estimator.state.mean[5:].copy()
    )

    updated = estimator.update_joint_encoders(
        measurement,
    )

    predicted_error = np.linalg.norm(
        measurement.value
        - predicted_joint_state
    )

    updated_error = np.linalg.norm(
        measurement.value
        - updated.mean[5:]
    )

    assert updated_error < predicted_error


def test_sequential_sensor_fusion() -> None:
    estimator = create_estimator()

    estimator.predict(
        dt=1.0,
    )

    wheel_measurement = Measurement(
        value=np.array([
            1.1,
            0.16,
        ]),
        covariance=np.diag([
            0.05,
            0.02,
        ]),
        timestamp=1.0,
    )

    estimator.update_wheel_odometry(
        wheel_measurement,
    )

    imu_measurement = Measurement(
        value=np.array([
            0.14,
        ]),
        covariance=np.array([
            [0.01],
        ]),
        timestamp=1.0,
    )

    estimator.update_imu(
        imu_measurement,
    )

    joint_measurement = Measurement(
        value=np.array([
            0.12,
            0.21,
            0.34,
            0.43,

            0.015,
            0.018,
            0.028,
            0.045,
        ]),
        covariance=0.02 * np.eye(8),
        timestamp=1.0,
    )

    updated = estimator.update_joint_encoders(
        joint_measurement,
    )

    assert updated.mean.shape == (13,)
    assert updated.covariance.shape == (13, 13)
    assert updated.timestamp == 1.0


def test_covariance_remains_symmetric() -> None:
    estimator = create_estimator()

    estimator.predict(
        dt=1.0,
    )

    measurement = Measurement(
        value=np.array([
            0.15,
        ]),
        covariance=np.array([
            [0.01],
        ]),
        timestamp=1.0,
    )

    updated = estimator.update_imu(
        measurement,
    )

    assert np.allclose(
        updated.covariance,
        updated.covariance.T,
    )


def test_invalid_dt_raises_error() -> None:
    estimator = create_estimator()

    try:
        estimator.predict(
            dt=0.0,
        )
    except ValueError:
        return

    raise AssertionError(
        "Expected ValueError for non-positive dt."
    )