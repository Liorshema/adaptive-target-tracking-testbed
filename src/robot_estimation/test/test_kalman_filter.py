import numpy as np

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate
from robot_estimation.filters.kalman_filter import KalmanFilter


def state_transition(dt: float) -> np.ndarray:
    return np.array([
        [1.0, dt],
        [0.0, 1.0],
    ])


def process_covariance(dt: float) -> np.ndarray:
    return 0.01 * np.eye(2)


def create_filter() -> KalmanFilter:
    initial_state = StateEstimate(
        mean=np.array([0.0, 1.0]),
        covariance=np.eye(2),
        timestamp=0.0,
    )

    measurement_matrix = np.array([
        [1.0, 0.0],
    ])

    return KalmanFilter(
        initial_state=initial_state,
        state_transition=state_transition,
        process_covariance=process_covariance,
        measurement_matrix=measurement_matrix,
    )


def test_prediction_updates_state() -> None:
    kf = create_filter()

    predicted = kf.predict(dt=1.0)

    expected = np.array([
        1.0,
        1.0,
    ])

    assert np.allclose(
        predicted.mean,
        expected,
    )


def test_prediction_updates_timestamp() -> None:
    kf = create_filter()

    predicted = kf.predict(dt=0.5)

    assert predicted.timestamp == 0.5


def test_update_moves_state_toward_measurement() -> None:
    kf = create_filter()

    kf.predict(dt=1.0)

    measurement = Measurement(
        value=np.array([1.2]),
        covariance=np.array([[0.1]]),
        timestamp=1.0,
    )

    updated = kf.update(measurement)

    assert updated.mean[0] > 1.0
    assert updated.mean[0] < 1.2


def test_update_reduces_position_uncertainty() -> None:
    kf = create_filter()

    predicted = kf.predict(dt=1.0)

    predicted_variance = predicted.covariance[0, 0]

    measurement = Measurement(
        value=np.array([1.2]),
        covariance=np.array([[0.1]]),
        timestamp=1.0,
    )

    updated = kf.update(measurement)

    updated_variance = updated.covariance[0, 0]

    assert updated_variance < predicted_variance


def test_covariance_remains_symmetric() -> None:
    kf = create_filter()

    kf.predict(dt=1.0)

    measurement = Measurement(
        value=np.array([1.2]),
        covariance=np.array([[0.1]]),
        timestamp=1.0,
    )

    updated = kf.update(measurement)

    assert np.allclose(
        updated.covariance,
        updated.covariance.T,
    )


def test_invalid_dt_raises_error() -> None:
    kf = create_filter()

    try:
        kf.predict(dt=0.0)
    except ValueError:
        return

    raise AssertionError(
        "Expected ValueError for non-positive dt."
    )