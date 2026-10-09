
"""Tests for advancing simulation to a requested time."""

import numpy as np
import pytest

from robot_models.types import TargetState

from robot_simulation.engine.simulation_engine import SimulationEngine
from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.integration.time_stepper import TimeStepper
from robot_simulation.interfaces.dynamics import Dynamics
from robot_simulation.state.simulation_state import SimulationState


class ConstantVelocityDynamics(Dynamics):
    """Simple 3D constant-velocity dynamics."""

    def derivative(self, state, control, time, context):
        """Return position and velocity derivatives."""
        dimension = state.size // 2
        velocity = state[dimension:]

        return np.concatenate((
            velocity,
            np.zeros(dimension),
        ))


def make_stepper(dt=0.01, max_steps=100):
    """Construct a deterministic test stepper."""
    engine = SimulationEngine(
        dynamics=ConstantVelocityDynamics(),
        integrator=RK4Integrator(),
    )

    return TimeStepper(
        engine=engine,
        dt=dt,
        max_steps=max_steps,
    )


def make_state(time=0.0):
    """Create a target with known constant velocity."""
    return SimulationState(
        target=TargetState(
            position=np.zeros(3),
            velocity=np.array([1.0, 2.0, 3.0]),
        ),
        tracking_position_world=np.zeros(3),
        time=time,
    )


def test_advances_exact_target_time():
    """The numerical state must reach the requested time."""
    result = make_stepper().advance(
        make_state(),
        target_time=0.08,
    )

    assert result.time == pytest.approx(0.08)

    np.testing.assert_allclose(
        result.target.position,
        [0.08, 0.16, 0.24],
        atol=1e-12,
    )


def test_partial_final_step():
    """A final shorter step must reach a non-multiple of dt."""
    result = make_stepper().advance(
        make_state(),
        target_time=0.025,
    )

    assert result.time == pytest.approx(0.025)

    np.testing.assert_allclose(
        result.target.position,
        [0.025, 0.05, 0.075],
        atol=1e-12,
    )


def test_zero_elapsed_time():
    """No time change must leave the state untouched."""
    state = make_state()

    assert make_stepper().advance(state, 0.0) is state


def test_rejects_backward_time():
    """Backward simulation time must be rejected."""
    with pytest.raises(ValueError, match="backwards"):
        make_stepper().advance(
            make_state(time=1.0),
            target_time=0.5,
        )


def test_rejects_excessive_gap():
    """Large gaps must not trigger unbounded integration."""
    stepper = make_stepper(dt=0.01, max_steps=3)

    with pytest.raises(ValueError, match="step limit"):
        stepper.advance(
            make_state(),
            target_time=0.1,
        )


def test_preserves_input_state():
    """Integration must not modify the original snapshot."""
    state = make_state()
    original_position = state.target.position.copy()

    make_stepper().advance(state, 0.05)

    np.testing.assert_array_equal(
        state.target.position,
        original_position,
    )
    assert state.time == 0.0

def test_time_stepper_handles_floating_point_boundaries():
    """Stepping across a floating-point boundary must stay positive."""

    from unittest.mock import Mock

    import numpy as np

    from robot_models.types import TargetState
    from robot_simulation.integration.time_stepper import TimeStepper
    from robot_simulation.state.simulation_state import SimulationState

    engine = Mock()

    def step(state, dt):
        assert np.isfinite(dt)
        assert dt > 0.0

        return replace(
            state,
            time=state.time + dt,
        )

    from dataclasses import replace

    engine.step.side_effect = step

    stepper = TimeStepper(
        engine=engine,
        dt=0.01,
        max_steps=1000,
    )

    state = SimulationState(
        target=TargetState(
            position=np.zeros(3),
            velocity=np.zeros(3),
        ),
        tracking_position_world=np.zeros(3),
        time=1000.0,
    )

    for index in range(1, 101):
        target_time = 1000.0 + index * 0.01
        state = stepper.advance(state, target_time)

        assert state.time == target_time
