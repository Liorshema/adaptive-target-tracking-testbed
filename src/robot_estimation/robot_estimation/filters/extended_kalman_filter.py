from typing import Callable, Optional

import numpy as np
from numpy.typing import NDArray

from robot_estimation.common.measurement import Measurement
from robot_estimation.common.state import StateEstimate
from robot_estimation.filters.filter_interface import FilterInterface


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]

ProcessFunction = Callable[
    [Vector, Optional[Vector], float],
    Vector,
]

ProcessJacobian = Callable[
    [Vector, Optional[Vector], float],
    Matrix,
]

ProcessCovarianceProvider = Callable[
    [float],
    Matrix,
]

MeasurementFunction = Callable[
    [Vector],
    Vector,
]

MeasurementJacobian = Callable[
    [Vector],
    Matrix,
]


class ExtendedKalmanFilter(FilterInterface):
    """
    Generic discrete-time Extended Kalman Filter.

    Nonlinear process model
    -----------------------
    x_k = f(x_{k-1}, u_k, dt) + w_k

    with

    w_k ~ N(0, Q(dt))

    Nonlinear measurement model
    ---------------------------
    z_k = h(x_k) + v_k

    with

    v_k ~ N(0, R)

    Measurement models may either be configured at construction time
    or supplied dynamically for each update step.
    """

    def __init__(
        self,
        initial_state: StateEstimate,
        process_function: ProcessFunction,
        process_jacobian: ProcessJacobian,
        process_covariance: ProcessCovarianceProvider,
        measurement_function: Optional[
            MeasurementFunction
        ] = None,
        measurement_jacobian: Optional[
            MeasurementJacobian
        ] = None,
        state_constraint: Optional[Callable[[StateEstimate], StateEstimate]] = None,
    ) -> None:
        self._state = initial_state.copy()

        self._process_function = process_function
        self._process_jacobian = process_jacobian
        self._process_covariance = process_covariance

        self._measurement_function = measurement_function
        self._measurement_jacobian = measurement_jacobian
        self._state_constraint = state_constraint

    @property
    def state(self) -> StateEstimate:
        """
        Return a copy of the current filter state.
        """
        return self._state.copy()

    def predict(
        self,
        dt: float,
        control: Optional[Vector] = None,
    ) -> StateEstimate:
        """
        Perform the nonlinear EKF prediction step.

        Parameters
        ----------
        dt:
            Prediction time step in seconds.

        control:
            Optional control-input vector.

        Returns
        -------
        StateEstimate
            Predicted state estimate.
        """
        if dt <= 0.0:
            raise ValueError(
                "dt must be positive."
            )

        x = self._state.mean
        P = self._state.covariance

        if control is not None:
            control = np.asarray(
                control,
                dtype=float,
            )

            if control.ndim != 1:
                raise ValueError(
                    "Control input must be a 1D vector, "
                    f"got shape {control.shape}."
                )

        predicted_mean = np.asarray(
            self._process_function(
                x,
                control,
                dt,
            ),
            dtype=float,
        )

        F = np.asarray(
            self._process_jacobian(
                x,
                control,
                dt,
            ),
            dtype=float,
        )

        Q = np.asarray(
            self._process_covariance(dt),
            dtype=float,
        )

        n = self._state.dimension

        if predicted_mean.shape != (n,):
            raise ValueError(
                "Process function must return shape "
                f"({n},), "
                f"got {predicted_mean.shape}."
            )

        if F.shape != (n, n):
            raise ValueError(
                "Process Jacobian must have shape "
                f"({n}, {n}), "
                f"got {F.shape}."
            )

        if Q.shape != (n, n):
            raise ValueError(
                "Process covariance must have shape "
                f"({n}, {n}), "
                f"got {Q.shape}."
            )

        predicted_covariance = (
            F
            @ P
            @ F.T
            + Q
        )

        timestamp = self._state.timestamp

        if timestamp is not None:
            timestamp += dt

        candidate = StateEstimate(
            mean=predicted_mean,
            covariance=predicted_covariance,
            timestamp=timestamp,
        )
        self._state = (candidate if self._state_constraint is None
                       else self._state_constraint(candidate))

        return self.state

    def update(
        self,
        measurement: Measurement,
        measurement_function: Optional[
            MeasurementFunction
        ] = None,
        measurement_jacobian: Optional[
            MeasurementJacobian
        ] = None,
    ) -> StateEstimate:
        """
        Perform the nonlinear EKF measurement-update step.

        A measurement model may be supplied dynamically for this
        update. If omitted, the model configured at construction time
        is used.

        Parameters
        ----------
        measurement:
            Measurement used to correct the predicted state.

        measurement_function:
            Optional measurement function h(x) for this update.

        measurement_jacobian:
            Optional measurement Jacobian H(x) for this update.

        Returns
        -------
        StateEstimate
            Updated state estimate.
        """
        h = (
            measurement_function
            if measurement_function is not None
            else self._measurement_function
        )

        H_function = (
            measurement_jacobian
            if measurement_jacobian is not None
            else self._measurement_jacobian
        )

        if h is None or H_function is None:
            raise ValueError(
                "A measurement function and measurement Jacobian "
                "must be provided for the update step."
            )

        x = self._state.mean
        P = self._state.covariance

        z = measurement.value
        R = measurement.covariance

        predicted_measurement = np.asarray(
            h(x),
            dtype=float,
        )

        H = np.asarray(
            H_function(x),
            dtype=float,
        )

        measurement_dimension = z.shape[0]
        state_dimension = self._state.dimension

        if predicted_measurement.shape != z.shape:
            raise ValueError(
                "Measurement function output shape does not match "
                "measurement shape. "
                f"Got {predicted_measurement.shape} "
                f"and {z.shape}."
            )

        if H.shape != (
            measurement_dimension,
            state_dimension,
        ):
            raise ValueError(
                "Measurement Jacobian has invalid shape. "
                f"Expected "
                f"({measurement_dimension}, {state_dimension}), "
                f"got {H.shape}."
            )

        if R.shape != (
            measurement_dimension,
            measurement_dimension,
        ):
            raise ValueError(
                "Measurement covariance has invalid shape. "
                f"Expected "
                f"({measurement_dimension}, "
                f"{measurement_dimension}), "
                f"got {R.shape}."
            )

        innovation = (
            z - predicted_measurement
        )

        innovation_covariance = (
            H
            @ P
            @ H.T
            + R
        )

        kalman_gain = np.linalg.solve(
            innovation_covariance.T,
            (P @ H.T).T,
        ).T

        updated_mean = (
            x
            + kalman_gain
            @ innovation
        )

        identity = np.eye(
            state_dimension
        )

        correction = (
            identity
            - kalman_gain
            @ H
        )

        updated_covariance = (
            correction
            @ P
            @ correction.T
            + kalman_gain
            @ R
            @ kalman_gain.T
        )

        candidate = StateEstimate(
            mean=updated_mean,
            covariance=updated_covariance,
            timestamp=measurement.timestamp,
        )
        self._state = (candidate if self._state_constraint is None
                       else self._state_constraint(candidate))

        return self.state
