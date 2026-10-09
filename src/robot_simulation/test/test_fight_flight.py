
"""Tests for Fight / Flight target dynamics."""

import numpy as np

from robot_simulation.dynamics.target.fight_flight import (
    FightFlightDynamics,
    FightFlightParameters,
)


def make_dynamics():
    return FightFlightDynamics(
        FightFlightParameters(
            alpha=2.0,
            preferred_distance=1.0,
            damping=0.0,
            distance_epsilon=1e-6,
        )
    )


def compute_acceleration(position):
    state = np.concatenate((position, np.zeros(3)))

    derivative = make_dynamics().derivative(
        state=state,
        control=np.empty(0),
        time=0.0,
        context={
            "tracking_position_world": np.zeros(3),
        },
    )

    return derivative[3:]


def test_flight_outside_preferred_distance():
    acceleration = compute_acceleration(
        np.array([2.0, 0.0, 0.0])
    )

    assert acceleration[0] > 0.0


def test_fight_inside_preferred_distance():
    acceleration = compute_acceleration(
        np.array([0.5, 0.0, 0.0])
    )

    assert acceleration[0] < 0.0


def test_zero_interaction_at_preferred_distance():
    acceleration = compute_acceleration(
        np.array([1.0, 0.0, 0.0])
    )

    np.testing.assert_allclose(
        acceleration,
        np.zeros(3),
        atol=1e-10,
    )


def test_three_dimensional_behavior():
    acceleration = compute_acceleration(
        np.array([0.0, 0.0, 2.0])
    )

    assert acceleration[2] > 0.0
    np.testing.assert_allclose(
        acceleration[:2],
        np.zeros(2),
    )


def test_coincident_positions_remain_finite():
    acceleration = compute_acceleration(
        np.zeros(3)
    )

    assert np.all(np.isfinite(acceleration))
