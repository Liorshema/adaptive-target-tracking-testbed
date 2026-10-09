
"""Tests for combined Fight / Flight and environmental dynamics."""

import numpy as np

from robot_simulation.dynamics.target.fight_flight import (
    FightFlightDynamics,
    FightFlightParameters,
)
from robot_simulation.disturbances.vector_field import (
    RestoringVectorField,
    VectorFieldParameters,
)
from robot_simulation.integration.rk4 import RK4Integrator


def make_dynamics():
    """Construct combined dynamics for testing."""
    field = RestoringVectorField(
        VectorFieldParameters(
            center=np.zeros(3),
            gain_matrix=2.0 * np.eye(3),
        )
    )

    return FightFlightDynamics(
        parameters=FightFlightParameters(
            alpha=1.0,
            preferred_distance=1.0,
            damping=0.5,
            distance_epsilon=1e-6,
        ),
        environment_field=field,
    )


def test_combined_acceleration():
    """Verify the sum of radial, restoring and damping terms."""
    dynamics = make_dynamics()

    state = np.array([2.0, 0.0, 0.0, 1.0, 0.0, 0.0])

    derivative = dynamics.derivative(
        state=state,
        control=np.empty(0),
        time=0.0,
        context={"tracking_position_world": np.zeros(3)},
    )

    np.testing.assert_allclose(derivative[:3], state[3:])

    # Radial +1, restoring -4, damping -0.5
    np.testing.assert_allclose(
        derivative[3:],
        [-3.5, 0.0, 0.0],
        atol=1e-10,
    )


def test_combined_rk4_step():
    """Verify 3D state integration with the combined dynamics."""
    dynamics = make_dynamics()

    initial_state = np.array([
        2.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
    ])

    result = RK4Integrator().step(
        dynamics=dynamics,
        state=initial_state,
        control=np.empty(0),
        time=0.0,
        dt=0.01,
        context={"tracking_position_world": np.zeros(3)},
    )

    assert result.shape == (6,)
    assert np.all(np.isfinite(result))
    assert result[2] < initial_state[2]
