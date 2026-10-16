"""Bounded Gaussian moments must transform joint correlations and remain observable."""

import numpy as np

import pytest

from robot_estimation.common.state import StateEstimate
from robot_estimation.robot.joint_bounds import JointBoundsConditioner


def conditioner():
    """Use an arbitrary state index, not a particular robot joint."""
    c = JointBoundsConditioner([0], [0.], [1.], 1e-9, .2)
    c.observe([0.], 1.)
    return c


def test_truncated_moments_match_conditioned_correlated_samples():
    """Cross-covariance and velocity must change consistently with the mean."""
    c = conditioner()
    original = StateEstimate(np.array([-.1, -.2]),
                             np.array([[.04, .01], [.01, .09]]), 1.)
    result = c(original)
    rng = np.random.default_rng(42)
    samples = rng.multivariate_normal(original.mean, original.covariance, 200000)
    selected = samples[(samples[:, 0] >= 0.) & (samples[:, 0] <= 1.)]
    np.testing.assert_allclose(result.mean, selected.mean(axis=0), atol=.003)
    np.testing.assert_allclose(result.covariance, np.cov(selected.T), atol=.002)
    assert result.timestamp == original.timestamp
    assert np.linalg.eigvalsh(result.covariance).min() >= 0
    assert 0 < result.mean[0] < 1


@pytest.mark.parametrize('mean', [-.01, 1.01])
def test_both_boundaries_leave_positive_covariance_and_release(mean):
    """Both boundaries leave positive covariance and release."""
    c = conditioner()
    result = c(StateEstimate(np.array([mean, 0.]), np.eye(2) * .001, 1.))
    assert 0 <= result.mean[0] <= 1
    assert result.covariance[0, 0] > 0
    legal = StateEstimate(np.array([.2, .1]), np.eye(2) * .002, 1.01)
    np.testing.assert_array_equal(c(legal).mean, legal.mean)


def test_actual_physical_violation_is_exposed_and_never_conditioned_away():
    """Actual physical violation is exposed and never conditioned away."""
    c = conditioner()
    c.observe([-.0001], 1.)
    assert not c.physical_valid
    assert c.excess[0] == .0001
    original = StateEstimate(np.array([-.001, 0.]), np.eye(2), 1.)
    np.testing.assert_array_equal(c(original).mean, original.mean)


def test_stale_or_future_encoder_information_cannot_authorize_conditioning():
    """Stale or future encoder information cannot authorize conditioning."""
    c = conditioner()
    for t in [.9, 1.3]:
        state = StateEstimate(np.array([-.001, 0.]), np.eye(2), t)
        np.testing.assert_array_equal(c(state).mean, state.mean)


def test_deterministic_infeasible_prior_does_not_silently_gain_certainty():
    """Deterministic infeasible prior does not silently gain certainty."""
    with pytest.raises(ValueError, match='variance'):
        conditioner()(StateEstimate(np.array([-.1, 0.]), np.zeros((2, 2)), 1.))


def test_real_filter_contact_and_release_remains_psd_and_responsive():
    """A constrained filter must release when encoders report real inward motion."""
    from robot_estimation.common.measurement import Measurement
    from robot_estimation.robot.models.measurement_model import (
        ImuYawRateMeasurementModel, JointEncoderMeasurementModel, WheelOdometryMeasurementModel)
    from robot_estimation.robot.models.process_model import RobotProcessModel
    from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
    c = JointBoundsConditioner([5], [0.], [1.], 1e-9, .2)
    initial = StateEstimate(np.array([0., 0., 0., 0., 0., .001, -.1]), np.eye(7)*.01, 0.)
    estimator = RobotStateEstimator(
        initial, RobotProcessModel(1), WheelOdometryMeasurementModel(1),
        ImuYawRateMeasurementModel(1), JointEncoderMeasurementModel(1), c)
    positions = np.r_[np.zeros(20), np.linspace(0., .05, 21)]
    for i, q in enumerate(positions):
        estimator.predict(.01)
        estimator.update_joint_encoders(Measurement(
            np.array([q, 0. if i < 20 else .25]),
            np.diag([.0001, .001]), (i+1)*.01))
        assert 0 <= estimator.state.mean[5] <= 1
        assert np.linalg.eigvalsh(estimator.state.covariance).min() >= -1e-12
    assert estimator.state.mean[5] > .04
    assert c.conditionings > 0
