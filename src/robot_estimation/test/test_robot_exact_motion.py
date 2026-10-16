"""Planar propagation and covariance Jacobian must describe the same finite step."""

import numpy as np

import pytest

from robot_estimation.robot.models.process_model import RobotProcessModel


def test_constant_turn_follows_exact_circle_over_long_steps():
    """Constant turn follows exact circle over long steps."""
    model = RobotProcessModel(1)
    state = np.zeros(7)
    state[3:5] = [1., .2]
    actual = model.propagate(state, 1.)
    np.testing.assert_allclose(actual[:2], [np.sin(.2) / .2, (1 - np.cos(.2)) / .2])


@pytest.mark.parametrize('yaw_rate', [0., 1e-10, .3, -.5])
def test_exact_motion_jacobian_matches_numerical_differentiation(yaw_rate):
    """Exact motion jacobian matches numerical differentiation."""
    model = RobotProcessModel(1)
    state = np.array([.2, -.1, .7, .3, yaw_rate, .1, .02])
    columns = []
    for i in range(7):
        d = np.eye(7)[i] * 1e-6
        columns.append((model.propagate(state + d, .1) -
                        model.propagate(state - d, .1)) / 2e-6)
    np.testing.assert_allclose(model.jacobian(state, .1), np.array(columns).T, atol=1e-8)


def test_continuous_noise_covariance_is_invariant_to_stationary_time_partition():
    """Accepting more timestamps must not silently reduce process uncertainty."""
    model = RobotProcessModel(1)
    state = np.zeros(7)
    p = np.eye(7)
    f = model.jacobian(state, .1)
    one = f @ p @ f.T + model.process_covariance(.1)
    many = p.copy()
    for _ in range(10):
        f = model.jacobian(state, .01)
        many = f @ many @ f.T + model.process_covariance(.01)
    np.testing.assert_allclose(one, many, atol=1e-12)


def test_coupled_continuous_covariance_matches_noise_impulse_integral():
    """A closed form must retain all moving-base cross-covariances and PSD."""
    model = RobotProcessModel(1, .2, .3, .4)
    x = np.array([0., 0., .7, .4, .2, .2, -.1])
    dt = .1
    a = np.zeros((7, 7))
    a[0, 2], a[1, 2] = -.4 * np.sin(.7), .4 * np.cos(.7)
    a[0, 3], a[1, 3] = np.cos(.7), np.sin(.7)
    a[2, 4] = a[5, 6] = 1.
    spectral = np.diag([0., 0., 0., .2**2, .3**2, 0., .4**2])
    points, weights = np.polynomial.legendre.leggauss(8)
    expected = np.zeros((7, 7))
    for point, weight in zip(points, weights):
        t = (point + 1) * dt / 2
        transition = np.eye(7) + a * t + a @ a * t**2 / 2
        expected += weight * dt / 2 * transition @ spectral @ transition.T
    actual = model.process_covariance(dt, x)
    np.testing.assert_allclose(actual, expected, atol=1e-13)
    assert np.linalg.eigvalsh(actual).min() >= -1e-14
