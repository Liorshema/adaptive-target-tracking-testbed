
"""Generic target simulation execution engine."""

import numpy as np

from robot_simulation.interfaces.dynamics import Dynamics
from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.state.simulation_state import SimulationState
from robot_simulation.state.target_state import (
    state_to_vector,
    vector_to_state,
)


class SimulationEngine:
    """Advance the target simulation using configured dynamics."""

    def __init__(
        self,
        dynamics: Dynamics,
        integrator: RK4Integrator,
    ) -> None:
        self.dynamics = dynamics
        self.integrator = integrator

    def step(
        self,
        state: SimulationState,
        dt: float,
        control: np.ndarray | None = None,
    ) -> SimulationState:
        """Advance simulation state by one timestep."""
        if not isinstance(state, SimulationState):
            raise TypeError("state must be a SimulationState.")

        if control is None:
            control = np.empty(0, dtype=float)

        control = np.asarray(control, dtype=float)

        context = {
            "tracking_position_world": (
                state.tracking_position_world.copy()
            ),
        }

        next_vector = self.integrator.step(
            dynamics=self.dynamics,
            state=state_to_vector(state.target),
            control=control,
            time=state.time,
            dt=dt,
            context=context,
        )

        if not np.all(np.isfinite(next_vector)):
            raise ValueError("Non-finite simulation state.")

        return SimulationState(
            target=vector_to_state(next_vector),
            tracking_position_world=state.tracking_position_world,
            time=state.time + dt,
        )
