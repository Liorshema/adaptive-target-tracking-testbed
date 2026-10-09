
"""Tests for the complete target simulation pipeline."""

import numpy as np
import pytest

from robot_models.types import TargetState
from robot_simulation.dynamics.target.fight_flight import (
    FightFlightDynamics,
    FightFlightParameters,
)
from robot_simulation.disturbances.vector_field import (
    RestoringVectorField,
    VectorFieldParameters,
)
from robot_simulation.engine.simulation_engine import SimulationEngine
from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.state.simulation_state import SimulationState


def make_engine():
    """Construct a deterministic combined dynamics engine."""
    field = RestoringVectorField(
        VectorFieldParameters(
            center=np.zeros(3),
            gain_matrix=2.0 * np.eye(3),
        )
    )

    dynamics = FightFlightDynamics(
        FightFlightParameters(
            alpha=1.0,
            preferred_distance=1.0,
            damping=0.5,
            distance_epsilon=1e-6,
        ),
        environment_field=field,
    )

    return SimulationEngine(dynamics, RK4Integrator())


def make_state():
    """Create a 3D initial simulation state."""
    return SimulationState(
        target=TargetState(
            position=np.array([2.0, 0.0, 1.0]),
            velocity=np.zeros(3),
        ),
        tracking_position_world=np.zeros(3),
        time=0.0,
    )


def test_engine_advances_time():
    """Simulation time must advance by dt."""
    result = make_engine().step(make_state(), dt=0.01)

    assert result.time == pytest.approx(0.01)


def test_engine_advances_target():
    """Target position must change under the configured forces."""
    initial = make_state()
    result = make_engine().step(initial, dt=0.01)

    assert result.target.position.shape == (3,)
    assert result.target.velocity.shape == (3,)
    assert not np.allclose(
        result.target.position,
        initial.target.position,
    )


def test_engine_preserves_input_state():
    """A simulation step must not modify its input snapshot."""
    initial = make_state()
    original_position = initial.target.position.copy()

    make_engine().step(initial, dt=0.01)

    np.testing.assert_array_equal(
        initial.target.position,
        original_position,
    )

    assert initial.time == 0.0


def test_multiple_steps():
    """The engine must support repeated integration."""
    engine = make_engine()
    state = make_state()

    for _ in range(100):
        state = engine.step(state, dt=0.01)

    assert state.time == pytest.approx(1.0)
    assert np.all(np.isfinite(state.target.position))
    assert np.all(np.isfinite(state.target.velocity))


def test_invalid_tracking_position():
    """Reject a tracking-frame vector with an invalid dimension."""
    with pytest.raises(ValueError):
        SimulationState(
            target=make_state().target,
            tracking_position_world=np.zeros(2),
            time=0.0,
        )
