
"""Advance a simulation state to a requested simulation time."""

import math
from dataclasses import replace

import numpy as np

from robot_simulation.engine.simulation_engine import SimulationEngine
from robot_simulation.state.simulation_state import SimulationState


class TimeStepper:
    """Integrate to a requested time using bounded positive substeps."""

    def __init__(
        self,
        engine: SimulationEngine,
        dt: float,
        max_steps: int,
    ) -> None:
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be positive and finite.")

        if (
            isinstance(max_steps, bool)
            or not isinstance(max_steps, int)
            or max_steps <= 0
        ):
            raise ValueError("max_steps must be a positive integer.")

        self._engine = engine
        self._dt = float(dt)
        self._max_steps = max_steps

    def advance(
        self,
        state: SimulationState,
        target_time: float,
    ) -> SimulationState:
        """Advance without creating zero or negative integration steps."""
        if not np.isfinite(target_time):
            raise ValueError("Target time must be finite.")

        target_time = float(target_time)
        elapsed = target_time - state.time

        if elapsed < 0.0:
            raise ValueError("Simulation time moved backwards.")

        if elapsed == 0.0:
            return state

        # Account for floating-point rounding in time arithmetic.
        tolerance = max(
            16.0 * math.ulp(
                max(abs(state.time), abs(target_time), 1.0)
            ),
            self._dt * 1e-9,
        )

        steps_required = max(
            0,
            math.ceil((elapsed - tolerance) / self._dt),
        )

        if steps_required > self._max_steps:
            raise ValueError(
                "Simulation time gap exceeds configured "
                "integration step limit."
            )

        current = state

        for _ in range(self._max_steps):
            remaining = target_time - current.time

            if remaining <= tolerance:
                break

            step_dt = min(self._dt, remaining)

            if step_dt <= 0.0:
                break

            next_state = self._engine.step(
                state=current,
                dt=step_dt,
            )

            if next_state.time <= current.time:
                raise ValueError(
                    "Simulation time failed to advance."
                )

            current = next_state

        remaining = target_time - current.time

        if remaining > tolerance:
            raise ValueError(
                "Integration step limit reached before target time."
            )

        # Snap away insignificant accumulated time-rounding error.
        if current.time != target_time:
            current = replace(current, time=target_time)

        return current
