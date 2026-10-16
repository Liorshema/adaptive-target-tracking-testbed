"""Behavioral coverage for synthetic observations; no ROS or Gazebo required."""

import numpy as np
import pytest
from robot_simulation.measurements.synthetic_measurement import SyntheticMeasurementGenerator


def test_noise_free_measurement_uses_observation_matrix_timestamp_and_copies():
    matrix = np.array([[1., 0., 0.], [0., 0., 1.]])
    covariance = np.zeros((2, 2))
    generator = SyntheticMeasurementGenerator(matrix, covariance, np.random.default_rng(42))
    matrix[:] = 0
    measurement = generator.measure(np.array([2., 99., 3.]), 1.25)
    np.testing.assert_array_equal(measurement.value, [2., 3.])
    assert measurement.timestamp == 1.25
    measurement.covariance[:] = 99
    np.testing.assert_array_equal(generator.measure(np.zeros(3), 2.).covariance, covariance)


def test_seeded_noise_is_reproducible_and_has_configured_covariance():
    covariance = np.array([[.04, .01], [.01, .09]])
    first = SyntheticMeasurementGenerator(np.eye(2), covariance, np.random.default_rng(17))
    second = SyntheticMeasurementGenerator(np.eye(2), covariance, np.random.default_rng(17))
    np.testing.assert_array_equal(first.measure(np.zeros(2), 0.).value, second.measure(np.zeros(2), 0.).value)
    samples = np.array([first.measure(np.zeros(2), float(i)).value for i in range(4000)])
    np.testing.assert_allclose(samples.mean(axis=0), np.zeros(2), atol=.015)
    np.testing.assert_allclose(np.cov(samples.T), covariance, atol=.005)


def test_measurement_rejects_nonfinite_state_and_timestamp():
    generator = SyntheticMeasurementGenerator(np.eye(2), np.eye(2), np.random.default_rng(42))
    with pytest.raises(ValueError, match='finite'):
        generator.measure(np.array([np.nan, 0.]), 1.)
    with pytest.raises(ValueError, match='finite'):
        generator.measure(np.zeros(2), np.nan)
