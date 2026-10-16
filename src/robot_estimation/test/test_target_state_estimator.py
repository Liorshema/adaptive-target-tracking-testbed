import numpy as np

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate
from robot_estimation.filters.kalman_filter import KalmanFilter
from robot_estimation.target.models.measurement_model import (
    PositionMeasurementModel,
)
from robot_estimation.target.models.process_model import (
    ConstantVelocityProcessModel,
)
from robot_estimation.target.target_state_estimator import (
    TargetStateEstimator,
)


def create_estimator() -> TargetStateEstimator:
    process_model = ConstantVelocityProcessModel(
        spatial_dimension=3,
        acceleration_noise_std=0.1,
    )

    measurement_model = PositionMeasurementModel(
        spatial_dimension=3,
    )

    initial_state = StateEstimate(
        mean=np.array([
            0.0, 0.0, 0.0,
            1.0, 0.5, -0.2,
        ]),
        covariance=np.eye(6),
        timestamp=0.0,
    )

    filter_ = KalmanFilter(
        initial_state=initial_state,
        state_transition=process_model.state_transition_matrix,
        process_covariance=process_model.process_covariance,
        measurement_matrix=measurement_model.measurement_matrix(),
    )

    return TargetStateEstimator(
        filter_=filter_,
    )


def test_initial_state_is_available() -> None:
    estimator = create_estimator()

    expected = np.array([
        0.0, 0.0, 0.0,
        1.0, 0.5, -0.2,
    ])

    assert np.allclose(
        estimator.state.mean,
        expected,
    )


def test_prediction_propagates_target_state() -> None:
    estimator = create_estimator()

    predicted = estimator.predict(
        dt=2.0,
    )

    expected = np.array([
        2.0, 1.0, -0.4,
        1.0, 0.5, -0.2,
    ])

    assert np.allclose(
        predicted.mean,
        expected,
    )


def test_prediction_updates_timestamp() -> None:
    estimator = create_estimator()

    predicted = estimator.predict(
        dt=0.5,
    )

    assert predicted.timestamp == 0.5


def test_measurement_update_corrects_position() -> None:
    estimator = create_estimator()

    predicted = estimator.predict(
        dt=1.0,
    )

    measurement = Measurement(
        value=np.array([
            1.2,
            0.4,
            -0.1,
        ]),
        covariance=0.1 * np.eye(3),
        timestamp=1.0,
    )

    updated = estimator.update(
        measurement,
    )

    predicted_position = predicted.mean[:3]
    measured_position = measurement.value
    updated_position = updated.mean[:3]

    predicted_error = np.linalg.norm(
        measured_position - predicted_position
    )

    updated_error = np.linalg.norm(
        measured_position - updated_position
    )

    assert updated_error < predicted_error


def test_measurement_update_changes_velocity() -> None:
    estimator = create_estimator()

    predicted = estimator.predict(
        dt=1.0,
    )

    predicted_velocity = predicted.mean[3:].copy()

    measurement = Measurement(
        value=np.array([
            1.2,
            0.4,
            -0.1,
        ]),
        covariance=0.1 * np.eye(3),
        timestamp=1.0,
    )

    updated = estimator.update(
        measurement,
    )

    updated_velocity = updated.mean[3:]

    assert not np.allclose(
        updated_velocity,
        predicted_velocity,
    )


def test_measurement_update_reduces_position_uncertainty() -> None:
    estimator = create_estimator()

    predicted = estimator.predict(
        dt=1.0,
    )

    predicted_position_covariance = (
        predicted.covariance[:3, :3]
    )

    measurement = Measurement(
        value=np.array([
            1.2,
            0.4,
            -0.1,
        ]),
        covariance=0.1 * np.eye(3),
        timestamp=1.0,
    )

    updated = estimator.update(
        measurement,
    )

    updated_position_covariance = (
        updated.covariance[:3, :3]
    )

    assert np.trace(
        updated_position_covariance
    ) < np.trace(
        predicted_position_covariance
    )


def test_update_sets_measurement_timestamp() -> None:
    estimator = create_estimator()

    estimator.predict(
        dt=1.0,
    )

    measurement = Measurement(
        value=np.array([
            1.2,
            0.4,
            -0.1,
        ]),
        covariance=0.1 * np.eye(3),
        timestamp=1.0,
    )

    updated = estimator.update(
        measurement,
    )

    assert updated.timestamp == 1.0


def test_invalid_prediction_dt_raises_error() -> None:
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