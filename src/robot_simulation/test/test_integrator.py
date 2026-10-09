
"""Tests for the generic RK4 integrator."""

import numpy as np
import pytest

from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.interfaces.dynamics import Dynamics


class LinearDynamics(Dynamics):
    """Linear system with a configurable state matrix."""

    def __init__(self, matrix):
        self.matrix = np.asarray(matrix, dtype=float)

    def derivative(self, state, control, time, context):
        return self.matrix @ state


def test_zero_dynamics():
    integrator = RK4Integrator()
    dynamics = LinearDynamics(np.zeros((3, 3)))

    state = np.array([1.0, 2.0, 3.0])

    result = integrator.step(
        dynamics, state, np.empty(0), 0.0, 0.1, {}
    )

    np.testing.assert_allclose(result, state)


def test_constant_velocity():
    integrator = RK4Integrator()

    matrix = np.block([
        [np.zeros((3, 3)), np.eye(3)],
        [np.zeros((3, 3)), np.zeros((3, 3))],
    ])

    initial_state = np.array([
        1.0, 2.0, 3.0,
        0.5, -1.0, 2.0,
    ])

    result = integrator.step(
        LinearDynamics(matrix),
        initial_state,
        np.empty(0),
        0.0,
        0.2,
        {},
    )

    expected = initial_state.copy()
    expected[:3] += 0.2 * expected[3:]

    np.testing.assert_allclose(result, expected)


def test_exponential_dynamics():
    integrator = RK4Integrator()
    dynamics = LinearDynamics(np.array([[-1.0]]))

    result = integrator.step(
        dynamics, np.array([1.0]), np.empty(0), 0.0, 0.1, {}
    )

    np.testing.assert_allclose(
        result,
        np.array([np.exp(-0.1)]),
        rtol=1e-6,
    )


def test_invalid_timestep():
    integrator = RK4Integrator()
    dynamics = LinearDynamics(np.eye(2))

    with pytest.raises(ValueError):
        integrator.step(
            dynamics,
            np.ones(2),
            np.empty(0),
            0.0,
            -0.1,
            {},
        )
