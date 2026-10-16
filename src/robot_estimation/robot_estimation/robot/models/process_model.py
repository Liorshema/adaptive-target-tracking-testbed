import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class RobotProcessModel:
    """
    Process model for a differential-drive mobile manipulator.

    State
    -----
    x_R = [
        x_B,
        y_B,
        theta_B,
        v_B,
        omega_B,
        q,
        q_dot
    ]^T

    where

    q, q_dot in R^n

    and the complete state dimension is

        5 + 2n.
    """

    BASE_STATE_DIMENSION = 5

    def __init__(
        self,
        joint_count: int,
        linear_acceleration_noise_std: float = 1.0,
        angular_acceleration_noise_std: float = 1.0,
        joint_acceleration_noise_std: float = 1.0,
    ) -> None:
        if joint_count <= 0:
            raise ValueError(
                "joint_count must be positive."
            )

        if linear_acceleration_noise_std < 0.0:
            raise ValueError(
                "linear_acceleration_noise_std must be non-negative."
            )

        if angular_acceleration_noise_std < 0.0:
            raise ValueError(
                "angular_acceleration_noise_std must be non-negative."
            )

        if joint_acceleration_noise_std < 0.0:
            raise ValueError(
                "joint_acceleration_noise_std must be non-negative."
            )

        self._joint_count = joint_count

        self._linear_acceleration_noise_std = (
            linear_acceleration_noise_std
        )

        self._angular_acceleration_noise_std = (
            angular_acceleration_noise_std
        )

        self._joint_acceleration_noise_std = (
            joint_acceleration_noise_std
        )

    @property
    def joint_count(self) -> int:
        """Return the number of manipulator joints."""
        return self._joint_count

    @property
    def state_dimension(self) -> int:
        """Return the complete robot-state dimension."""
        return (
            self.BASE_STATE_DIMENSION
            + 2 * self._joint_count
        )

    def propagate(
        self,
        state: Vector,
        dt: float,
    ) -> Vector:
        """
        Propagate the robot state forward by dt.

        Base model
        ----------
        x_dot     = v cos(theta)
        y_dot     = v sin(theta)
        theta_dot = omega

        Joint model
        -----------
        q_dot is assumed constant over the prediction interval.
        """
        state = self._validate_state(state)
        self._validate_dt(dt)

        next_state = state.copy()

        theta = state[2]
        linear_velocity = state[3]
        angular_velocity = state[4]

        joint_position_start = self.BASE_STATE_DIMENSION
        joint_velocity_start = (
            joint_position_start + self._joint_count
        )

        joint_positions = state[
            joint_position_start:joint_velocity_start
        ]

        joint_velocities = state[
            joint_velocity_start:
        ]

        travel, _, _ = self._base_increment(theta, linear_velocity, angular_velocity, dt)
        next_state[:2] += travel

        next_state[2] += (
            dt
            * angular_velocity
        )

        next_state[
            joint_position_start:joint_velocity_start
        ] = (
            joint_positions
            + dt * joint_velocities
        )

        return next_state

    def jacobian(
        self,
        state: Vector,
        dt: float,
    ) -> Matrix:
        """
        Return the Jacobian of the process model with respect to state.
        """
        state = self._validate_state(state)
        self._validate_dt(dt)

        F = np.eye(
            self.state_dimension
        )

        theta = state[2]
        linear_velocity = state[3]

        travel, velocity_column, angular_column = self._base_increment(
            theta, linear_velocity, state[4], dt)
        F[:2, 2] = [-travel[1], travel[0]]
        F[:2, 3] = velocity_column
        F[:2, 4] = angular_column

        F[2, 4] = dt

        joint_position_start = self.BASE_STATE_DIMENSION
        joint_velocity_start = (
            joint_position_start + self._joint_count
        )

        F[
            joint_position_start:joint_velocity_start,
            joint_velocity_start:
        ] = (
            dt * np.eye(self._joint_count)
        )

        return F

    @staticmethod
    def _base_increment(theta, velocity, yaw_rate, dt):
        """Exact constant-twist SE(2) travel and velocity/yaw-rate derivatives."""
        a = yaw_rate * dt / 2
        if abs(a) < 1e-4:
            sinc = 1 - a * a / 6 + a**4 / 120
            derivative = -a / 3 + a**3 / 30
        else:
            sinc = np.sin(a) / a
            derivative = (a * np.cos(a) - np.sin(a)) / (a * a)
        direction = np.array([np.cos(theta + a), np.sin(theta + a)])
        tangent = np.array([-direction[1], direction[0]])
        velocity_column = dt * sinc * direction
        angular_column = velocity * dt**2 / 2 * (derivative * direction + sinc * tangent)
        return velocity * velocity_column, velocity_column, angular_column

    def process_covariance(self, dt: float, state=None) -> Matrix:
        """Discretize continuous white acceleration noise about the current state.

        Noise amplitudes are in m/s**(3/2) and rad/s**(3/2). The covariance is
        the integral of the frozen continuous Jacobian, including base position
        and yaw cross terms. At a stationary linearization it composes exactly
        over arbitrary prediction-time partitions.
        """
        self._validate_dt(dt)
        theta, velocity = (0., 0.) if state is None else self._validate_state(state)[[2, 3]]
        q = np.zeros((self.state_dimension, self.state_dimension))
        direction = np.array([np.cos(theta), np.sin(theta)])
        tangent = velocity * np.array([-direction[1], direction[0]])
        linear_density = self._linear_acceleration_noise_std**2
        angular_density = self._angular_acceleration_noise_std**2
        q[:2, :2] = (
            linear_density * dt**3 / 3 * np.outer(direction, direction) +
            angular_density * dt**5 / 20 * np.outer(tangent, tangent))
        q[:2, 3] = q[3, :2] = linear_density * dt**2 / 2 * direction
        q[3, 3] = linear_density * dt
        q[4, 4] = angular_density * dt
        q[2, 2] = angular_density * dt**3 / 3
        q[2, 4] = q[4, 2] = angular_density * dt**2 / 2
        q[:2, 4] = q[4, :2] = angular_density * dt**3 / 6 * tangent
        q[:2, 2] = q[2, :2] = angular_density * dt**4 / 8 * tangent
        positions = self.BASE_STATE_DIMENSION + np.arange(self._joint_count)
        speeds = positions + self._joint_count
        density = self._joint_acceleration_noise_std**2
        q[positions, positions] = density * dt**3 / 3
        q[positions, speeds] = q[speeds, positions] = density * dt**2 / 2
        q[speeds, speeds] = density * dt
        return q

    def _validate_state(
        self,
        state: Vector,
    ) -> Vector:
        """Validate and return the robot state vector."""
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

    @staticmethod
    def _validate_dt(
        dt: float,
    ) -> None:
        """Validate the prediction time step."""
        if dt <= 0.0:
            raise ValueError(
                "dt must be positive."
            )
        """
        Validate the prediction time step.
        """
        if dt <= 0.0:
            raise ValueError(
                "dt must be positive."
            )
