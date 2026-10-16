from typing import Callable, Optional

import numpy as np
from numpy.typing import NDArray

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate
from robot_estimation.interfaces.filter_interface import FilterInterface


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]

StateTransitionProvider = Callable[[float], Matrix]
ProcessCovarianceProvider = Callable[[float], Matrix]
ControlMatrixProvider = Callable[[float], Matrix]


class KalmanFilter(FilterInterface):
    """
    Generic discrete-time linear Kalman filter.

    State model
    -----------
    x_k = A(dt) x_{k-1} + B(dt) u_k + w_k

    Measurement model
    -----------------
    z_k = H x_k + v_k

    where

    w_k ~ N(0, Q(dt))
    v_k ~ N(0, R)
    """

    def __init__(
        self,
        initial_state: StateEstimate,
        state_transition: StateTransitionProvider,
        process_covariance: ProcessCovarianceProvider,
        measurement_matrix: Matrix,
        control_matrix: Optional[ControlMatrixProvider] = None,
    ) -> None:
        self._state = initial_state.copy()

        self._state_transition = state_transition
        self._process_covariance = process_covariance
        self._control_matrix = control_matrix

        self._H = np.asarray(measurement_matrix, dtype=float)

        self._validate_measurement_matrix()

    @property
    def state(self) -> StateEstimate:
        """Return a copy of the current filter state."""
        return self._state.copy()

    def predict(
        self,
        dt: float,
        control: Optional[Vector] = None,
    ) -> StateEstimate:
        """
        Perform the linear Kalman prediction step.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        x = self._state.mean
        P = self._state.covariance

        A = np.asarray(
            self._state_transition(dt),
            dtype=float,
        )

        Q = np.asarray(
            self._process_covariance(dt),
            dtype=float,
        )

        self._validate_prediction_matrices(A, Q)

        predicted_mean = A @ x

        if control is not None:
            if self._control_matrix is None:
                raise ValueError(
                    "Control input was provided, but no control matrix "
                    "provider is configured."
                )

            control = np.asarray(control, dtype=float)

            if control.ndim != 1:
                raise ValueError(
                    f"Control input must be a 1D vector, "
                    f"got shape {control.shape}."
                )

            B = np.asarray(
                self._control_matrix(dt),
                dtype=float,
            )

            if B.ndim != 2:
                raise ValueError(
                    f"B must be a 2D matrix, got shape {B.shape}."
                )

            if B.shape[0] != self._state.dimension:
                raise ValueError(
                    f"B must have {self._state.dimension} rows, "
                    f"got {B.shape[0]}."
                )

            if B.shape[1] != control.shape[0]:
                raise ValueError(
                    "Control input dimension does not match "
                    "control matrix B."
                )

            predicted_mean = predicted_mean + B @ control

        predicted_covariance = (
            A @ P @ A.T + Q
        )

        timestamp = self._state.timestamp

        if timestamp is not None:
            timestamp += dt

        self._state = StateEstimate(
            mean=predicted_mean,
            covariance=predicted_covariance,
            timestamp=timestamp,
        )

        return self.state

    def update(
        self,
        measurement: Measurement,
    ) -> StateEstimate:
        """
        Perform the linear Kalman measurement-update step.
        """
        x = self._state.mean
        P = self._state.covariance

        z = measurement.value
        R = measurement.covariance

        if self._H.shape[0] != z.shape[0]:
            raise ValueError(
                "Measurement dimension does not match "
                "measurement matrix H."
            )

        innovation = z - self._H @ x

        innovation_covariance = (
            self._H @ P @ self._H.T + R
        )

        kalman_gain = np.linalg.solve(
            innovation_covariance.T,
            (P @ self._H.T).T,
        ).T

        updated_mean = x + kalman_gain @ innovation

        identity = np.eye(P.shape[0])
        correction = identity - kalman_gain @ self._H

        updated_covariance = (
            correction
            @ P
            @ correction.T
            + kalman_gain
            @ R
            @ kalman_gain.T
        )

        self._state = StateEstimate(
            mean=updated_mean,
            covariance=updated_covariance,
            timestamp=measurement.timestamp,
        )

        return self.state

    def _validate_measurement_matrix(self) -> None:
        """
        Validate the measurement matrix.
        """
        n = self._state.dimension

        if self._H.ndim != 2:
            raise ValueError(
                f"H must be a 2D matrix, got shape {self._H.shape}."
            )

        if self._H.shape[1] != n:
            raise ValueError(
                f"H must have {n} columns, "
                f"got {self._H.shape[1]}."
            )

    def _validate_prediction_matrices(
        self,
        A: Matrix,
        Q: Matrix,
    ) -> None:
        """
        Validate matrices generated for the current prediction step.
        """
        n = self._state.dimension

        if A.shape != (n, n):
            raise ValueError(
                f"A must have shape ({n}, {n}), "
                f"got {A.shape}."
            )

        if Q.shape != (n, n):
            raise ValueError(
                f"Q must have shape ({n}, {n}), "
                f"got {Q.shape}."
            )