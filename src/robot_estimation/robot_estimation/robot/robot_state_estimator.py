from typing import Optional

import numpy as np
from numpy.typing import NDArray

from robot_estimation.interfaces.estimator_interface import EstimatorInterface
from robot_estimation.measurement import Measurement
from robot_estimation.robot.planar_pose import PlanarPoseModel
from robot_estimation.state import StateEstimate
from robot_estimation.filters.extended_kalman_filter import (
    ExtendedKalmanFilter,
)
from robot_estimation.robot.models.measurement_model import (
    ImuYawRateMeasurementModel,
    JointEncoderMeasurementModel,
    WheelOdometryMeasurementModel,
)
from robot_estimation.robot.models.process_model import RobotProcessModel


Vector = NDArray[np.float64]


class RobotStateEstimator(EstimatorInterface):
    """
    State estimator for a differential-drive mobile manipulator.

    Robot state
    -----------
    x_R = [
        x_B,
        y_B,
        theta_B,
        v_B,
        omega_B,
        q,
        q_dot
    ]^T

    The estimator performs nonlinear state propagation and sequential
    measurement updates from wheel odometry, IMU yaw-rate measurements,
    and joint encoders.
    """

    def __init__(
        self,
        initial_state: StateEstimate,
        process_model: RobotProcessModel,
        wheel_measurement_model: WheelOdometryMeasurementModel,
        imu_measurement_model: ImuYawRateMeasurementModel,
        joint_measurement_model: JointEncoderMeasurementModel,
        state_constraint=None,
    ) -> None:
        if (
            initial_state.dimension
            != process_model.state_dimension
        ):
            raise ValueError(
                "Initial state dimension does not match "
                "the robot process model."
            )

        joint_count = process_model.joint_count

        if (
            wheel_measurement_model.joint_count
            != joint_count
        ):
            raise ValueError(
                "Process model and wheel measurement model "
                "must use the same joint count."
            )

        if (
            imu_measurement_model.joint_count
            != joint_count
        ):
            raise ValueError(
                "Process model and IMU measurement model "
                "must use the same joint count."
            )

        if (
            joint_measurement_model.joint_count
            != joint_count
        ):
            raise ValueError(
                "Process model and joint measurement model "
                "must use the same joint count."
            )

        self._process_model = process_model

        self._wheel_measurement_model = (
            wheel_measurement_model
        )

        self._imu_measurement_model = (
            imu_measurement_model
        )

        self._joint_measurement_model = (
            joint_measurement_model
        )

        self._filter = ExtendedKalmanFilter(
            initial_state=initial_state,
            process_function=self._process_function,
            process_jacobian=self._process_jacobian,
            process_covariance=(
                self._process_covariance
            ),
            state_constraint=state_constraint,
        )
        self._state_constraint = state_constraint
        self._pose_model = PlanarPoseModel(joint_count)
        self.pose_diagnostics = {"accepted": False, "updates": 0, "rejected": 0}

    @property
    def state(self) -> StateEstimate:
        """
        Return the current robot-state estimate.
        """
        return self._filter.state

    def predict(
        self,
        dt: float,
        control: Optional[Vector] = None,
    ) -> StateEstimate:
        """
        Predict the robot state forward in time.
        """
        return self._filter.predict(
            dt=dt,
            control=control,
        )

    def update(
        self,
        measurement: Measurement,
    ) -> StateEstimate:
        """
        Generic robot measurement update.

        Robot measurements use different measurement models depending
        on the sensor, so sensor-specific update methods must be used.
        """
        raise NotImplementedError(
            "RobotStateEstimator requires a sensor-specific update. "
            "Use update_wheel_odometry(), update_imu(), "
            "or update_joint_encoders()."
        )

    def update_wheel_odometry(
        self,
        measurement: Measurement,
    ) -> StateEstimate:
        """
        Update the robot state using wheel odometry.

        Measurement
        -----------
        z_odom = [v_B, omega_B]^T
        """
        self._validate_measurement_dimension(
            measurement=measurement,
            expected_dimension=(
                self._wheel_measurement_model.measurement_dimension
            ),
            sensor_name="Wheel odometry",
        )

        return self._filter.update(
            measurement=measurement,
            measurement_function=(
                self._wheel_measurement_model.predict_measurement
            ),
            measurement_jacobian=(
                self._wheel_measurement_model.jacobian
            ),
        )

    def update_imu(
        self,
        measurement: Measurement,
    ) -> StateEstimate:
        """
        Update the robot state using IMU yaw-rate measurement.

        Measurement
        -----------
        z_imu = [omega_B]
        """
        self._validate_measurement_dimension(
            measurement=measurement,
            expected_dimension=(
                self._imu_measurement_model.measurement_dimension
            ),
            sensor_name="IMU",
        )

        return self._filter.update(
            measurement=measurement,
            measurement_function=(
                self._imu_measurement_model.predict_measurement
            ),
            measurement_jacobian=(
                self._imu_measurement_model.jacobian
            ),
        )

    def update_joint_encoders(
        self,
        measurement: Measurement,
    ) -> StateEstimate:
        """
        Update the robot state using joint encoders.

        Measurement
        -----------
        z_joint = [q, q_dot]^T
        """
        self._validate_measurement_dimension(
            measurement=measurement,
            expected_dimension=(
                self._joint_measurement_model.measurement_dimension
            ),
            sensor_name="Joint encoder",
        )
        if self._state_constraint is not None:
            self._state_constraint.observe(
                measurement.value[:self._process_model.joint_count], measurement.timestamp)

        return self._filter.update(
            measurement=measurement,
            measurement_function=(
                self._joint_measurement_model.predict_measurement
            ),
            measurement_jacobian=(
                self._joint_measurement_model.jacobian
            ),
        )

    def update_planar_pose(self, measurement, nis_limit):
        """Gate and fuse an independent absolute pose without changing state layout.

        Represent observed heading in the chart around predicted heading, so the
        existing Joseph-form EKF update receives the correct wrapped innovation.
        """
        self._validate_measurement_dimension(measurement, 3, 'Planar pose')
        R = measurement.covariance
        if (not np.isfinite(nis_limit) or nis_limit <= 0
                or not np.isfinite(measurement.value).all() or not np.isfinite(R).all()
                or not np.allclose(R, R.T) or np.linalg.eigvalsh(R).min() <= 0):
            raise ValueError('Invalid pose covariance, value or innovation gate')
        state = self.state
        H = self._pose_model.jacobian(state.mean)
        innovation = self._pose_model.innovation(measurement.value, state.mean)
        S = H @ state.covariance @ H.T + R
        nis = float(innovation @ np.linalg.solve(S, innovation))
        accepted = nis <= nis_limit
        self.pose_diagnostics.update(accepted=accepted, nis=nis,
            updates=self.pose_diagnostics['updates'] + int(accepted),
            rejected=self.pose_diagnostics['rejected'] + int(not accepted))
        if not accepted:
            return state
        equivalent = Measurement(self._pose_model.predict_measurement(state.mean) + innovation,
                                 R.copy(), measurement.timestamp)
        return self._filter.update(equivalent, self._pose_model.predict_measurement,
                                   self._pose_model.jacobian)

    def _process_function(
        self,
        state: Vector,
        control: Optional[Vector],
        dt: float,
    ) -> Vector:
        """
        Adapt RobotProcessModel to the EKF process-function API.
        """
        return self._process_model.propagate(
            state=state,
            dt=dt,
        )

    def _process_jacobian(
        self,
        state: Vector,
        control: Optional[Vector],
        dt: float,
    ) -> np.ndarray:
        """
        Adapt RobotProcessModel Jacobian to the EKF API.
        """
        return self._process_model.jacobian(
            state=state,
            dt=dt,
        )

    def _process_covariance(self, dt):
        """Use the same state linearization as the nonlinear prediction step."""
        return self._process_model.process_covariance(dt, self._filter.state.mean)

    @staticmethod
    def _validate_measurement_dimension(
        measurement: Measurement,
        expected_dimension: int,
        sensor_name: str,
    ) -> None:
        """
        Validate a sensor measurement dimension.
        """
        if measurement.dimension != expected_dimension:
            raise ValueError(
                f"{sensor_name} measurement must have dimension "
                f"{expected_dimension}, "
                f"got {measurement.dimension}."
            )
