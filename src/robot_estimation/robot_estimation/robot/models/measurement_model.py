from robot_estimation.interfaces.robot_measurement_model import RobotMeasurementModel

import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class WheelOdometryMeasurementModel(RobotMeasurementModel):
    """
    Wheel-odometry measurement model.

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

    Measurement
    -----------
    z_odom = [v_B, omega_B]^T
    """

    BASE_STATE_DIMENSION = 5

    def __init__(
        self,
        joint_count: int,
    ) -> None:
        if joint_count <= 0:
            raise ValueError(
                "joint_count must be positive."
            )

        self._joint_count = joint_count

    @property
    def joint_count(self) -> int:
        """Return the expected number of robot joints."""
        return self._joint_count

    @property
    def state_dimension(self) -> int:
        """Return the expected robot-state dimension."""
        return (
            self.BASE_STATE_DIMENSION
            + 2 * self._joint_count
        )

    @property
    def measurement_dimension(self) -> int:
        """Return the wheel-odometry measurement dimension."""
        return 2

    def predict_measurement(
        self,
        state: Vector,
    ) -> Vector:
        """
        Predict wheel-odometry measurement.
        """
        state = self._validate_state(state)

        return state[[3, 4]].copy()

    def jacobian(
        self,
        state: Vector,
    ) -> Matrix:
        """
        Return the wheel-odometry measurement Jacobian.
        """
        self._validate_state(state)

        H = np.zeros(
            (
                self.measurement_dimension,
                self.state_dimension,
            )
        )

        H[0, 3] = 1.0
        H[1, 4] = 1.0

        return H

    def _validate_state(
        self,
        state: Vector,
    ) -> Vector:
        """
        Validate and return the robot state vector.
        """
        state = np.asarray(
            state,
            dtype=float,
        )

        if state.shape != (
            self.state_dimension,
        ):
            raise ValueError(
                "Robot state must have shape "
                f"({self.state_dimension},), "
                f"got {state.shape}."
            )

        return state


class JointEncoderMeasurementModel(RobotMeasurementModel):
    """
    Joint-encoder measurement model.

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

    Measurement
    -----------
    z_joint = [q, q_dot]^T
    """

    BASE_STATE_DIMENSION = 5

    def __init__(
        self,
        joint_count: int,
    ) -> None:
        if joint_count <= 0:
            raise ValueError(
                "joint_count must be positive."
            )

        self._joint_count = joint_count

    @property
    def joint_count(self) -> int:
        """Return the expected number of robot joints."""
        return self._joint_count

    @property
    def state_dimension(self) -> int:
        """Return the expected robot-state dimension."""
        return (
            self.BASE_STATE_DIMENSION
            + 2 * self._joint_count
        )

    @property
    def measurement_dimension(self) -> int:
        """Return the joint-encoder measurement dimension."""
        return 2 * self._joint_count

    def predict_measurement(
        self,
        state: Vector,
    ) -> Vector:
        """
        Predict the joint-encoder measurement.
        """
        state = self._validate_state(state)

        return state[
            self.BASE_STATE_DIMENSION:
        ].copy()

    def jacobian(
        self,
        state: Vector,
    ) -> Matrix:
        """
        Return the joint-encoder measurement Jacobian.
        """
        self._validate_state(state)

        H = np.zeros(
            (
                self.measurement_dimension,
                self.state_dimension,
            )
        )

        H[
            :,
            self.BASE_STATE_DIMENSION:
        ] = np.eye(
            self.measurement_dimension
        )

        return H

    def _validate_state(
        self,
        state: Vector,
    ) -> Vector:
        """
        Validate and return the robot state vector.
        """
        state = np.asarray(
            state,
            dtype=float,
        )

        if state.shape != (
            self.state_dimension,
        ):
            raise ValueError(
                "Robot state must have shape "
                f"({self.state_dimension},), "
                f"got {state.shape}."
            )

        return state


class ImuYawRateMeasurementModel(RobotMeasurementModel):
    """
    IMU yaw-rate measurement model.

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

    Measurement
    -----------
    z_imu = [omega_B]

    Only the planar yaw-rate component is used.

    Linear acceleration is not directly represented in the current
    robot state and is therefore not part of this measurement model.
    """

    BASE_STATE_DIMENSION = 5

    def __init__(
        self,
        joint_count: int,
    ) -> None:
        if joint_count <= 0:
            raise ValueError(
                "joint_count must be positive."
            )

        self._joint_count = joint_count

    @property
    def joint_count(self) -> int:
        """Return the expected number of robot joints."""
        return self._joint_count

    @property
    def state_dimension(self) -> int:
        """Return the expected robot-state dimension."""
        return (
            self.BASE_STATE_DIMENSION
            + 2 * self._joint_count
        )

    @property
    def measurement_dimension(self) -> int:
        """Return the IMU yaw-rate measurement dimension."""
        return 1

    def predict_measurement(
        self,
        state: Vector,
    ) -> Vector:
        """
        Predict the IMU yaw-rate measurement.
        """
        state = self._validate_state(state)

        return np.array([
            state[4],
        ])

    def jacobian(
        self,
        state: Vector,
    ) -> Matrix:
        """
        Return the IMU yaw-rate measurement Jacobian.
        """
        self._validate_state(state)

        H = np.zeros(
            (
                self.measurement_dimension,
                self.state_dimension,
            )
        )

        H[0, 4] = 1.0

        return H

    def _validate_state(
        self,
        state: Vector,
    ) -> Vector:
        """
        Validate and return the robot state vector.
        """
        state = np.asarray(
            state,
            dtype=float,
        )

        if state.shape != (
            self.state_dimension,
        ):
            raise ValueError(
                "Robot state must have shape "
                f"({self.state_dimension},), "
                f"got {state.shape}."
            )

        return state