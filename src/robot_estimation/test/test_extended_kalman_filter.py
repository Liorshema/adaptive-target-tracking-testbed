import numpy as np

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate
from robot_estimation.filters.extended_kalman_filter import (
    ExtendedKalmanFilter,
)


def process_function(
    state: np.ndarray,
    control: np.ndarray | None,
    dt: float,
) -> np.ndarray:
    A = np.array([
        [1.0, dt],
        [0.0, 1.0],
    ])

    return A @ state


def process_jacobian(
    state: np.ndarray,
    control: np.ndarray | None,
    dt: float,
) -> np.ndarray:
    return np.array([
        [1.0, dt],
        [0.0, 1.0],
    ])


def process_covariance(
    dt: float,
) -> np.ndarray:
    """
    Return process-noise covariance for the current prediction step.
    """
    return 0.01 * np.eye(2)


def measurement_function(
    state: np.ndarray,
) -> np.ndarray:
    return np.array([
        state[0],
    ])


def measurement_jacobian(
    state: np.ndarray,
) -> np.ndarray:
    return np.array([
        [1.0, 0.0],
    ])


def create_filter() -> ExtendedKalmanFilter:
    initial_state = StateEstimate(
        mean=np.array([
            0.0,
            1.0,
        ]),
        covariance=np.eye(2),
        timestamp=0.0,
    )

    return ExtendedKalmanFilter(
        initial_state=initial_state,
        process_function=process_function,
        process_jacobian=process_jacobian,
        process_covariance=process_covariance,
        measurement_function=measurement_function,
        measurement_jacobian=measurement_jacobian,
    )


def test_prediction_updates_state() -> None:
    ekf = create_filter()

    predicted = ekf.predict(
        dt=1.0,
    )

    expected = np.array([
        1.0,
        1.0,
    ])

    assert np.allclose(
        predicted.mean,
        expected,
    )


def test_prediction_updates_timestamp() -> None:
    ekf = create_filter()

    predicted = ekf.predict(
        dt=0.5,
    )

    assert predicted.timestamp == 0.5


def test_update_moves_state_toward_measurement() -> None:
    ekf = create_filter()

    ekf.predict(
        dt=1.0,
    )

    measurement = Measurement(
        value=np.array([
            1.2,
        ]),
        covariance=np.array([
            [0.1],
        ]),
        timestamp=1.0,
    )

    updated = ekf.update(
        measurement,
    )

    assert updated.mean[0] > 1.0
    assert updated.mean[0] < 1.2


def test_update_reduces_position_uncertainty() -> None:
    ekf = create_filter()

    predicted = ekf.predict(
        dt=1.0,
    )

    predicted_variance = (
        predicted.covariance[0, 0]
    )

    measurement = Measurement(
        value=np.array([
            1.2,
        ]),
        covariance=np.array([
            [0.1],
        ]),
        timestamp=1.0,
    )

    updated = ekf.update(
        measurement,
    )

    updated_variance = (
        updated.covariance[0, 0]
    )

    assert (
        updated_variance
        < predicted_variance
    )


def test_covariance_remains_symmetric() -> None:
    ekf = create_filter()

    ekf.predict(
        dt=1.0,
    )

    measurement = Measurement(
        value=np.array([
            1.2,
        ]),
        covariance=np.array([
            [0.1],
        ]),
        timestamp=1.0,
    )

    updated = ekf.update(
        measurement,
    )

    assert np.allclose(
        updated.covariance,
        updated.covariance.T,
    )


def test_invalid_dt_raises_error() -> None:
    ekf = create_filter()

    try:
        ekf.predict(
            dt=0.0,
        )
    except ValueError:
        return

    raise AssertionError(
        "Expected ValueError for non-positive dt."
    )
    ekf = create_filter()

    try:
        ekf.predict(dt=0.0)
    except ValueError:
        return

    raise AssertionError(
        "Expected ValueError for non-positive dt."
    )