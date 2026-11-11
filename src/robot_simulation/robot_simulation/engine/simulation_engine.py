
"""Generic target simulation execution engine."""

import numpy as np

from robot_simulation.interfaces.dynamics import Dynamics
from robot_simulation.integration.rk4 import RK4Integrator
from robot_simulation.state.simulation_state import SimulationState
from robot_simulation.adapters.target_state import TargetStateAdapter



class SimulationEngine:
    """Advance the target simulation using configured dynamics."""

    def __init__(
        self,
        dynamics: Dynamics,
        integrator: RK4Integrator,
    ) -> None:
        self.dynamics = dynamics
        self.integrator = integrator
        self.output_adapter = TargetStateAdapter(dynamics)

    def _context(self, tracking_position_world, physical_base=None):
        context = {'tracking_position_world': tracking_position_world.copy(),
                   'physical_base': physical_base}
        if hasattr(self.dynamics, 'frame_id'):
            context['frame_id'] = self.dynamics.frame_id
        return context

    def initialize(self, target, tracking_position_world, time):
        """Initialize full state from its authoritative owner at actual simulation time."""
        vector = self.output_adapter.initial_vector(target)
        physical = self.output_adapter.physical_state(
            vector, np.empty(0), time, self._context(tracking_position_world))
        return SimulationState(
            target=physical, tracking_position_world=tracking_position_world,
            time=time, internal_state=vector if self.output_adapter.mapped else None)

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

        context = self._context(state.tracking_position_world, state.physical_base)

        next_vector = self.integrator.step(
            dynamics=self.dynamics,
            state=self.output_adapter.vector(state),
            control=control,
            time=state.time,
            dt=dt,
            context=context,
        )

        if not np.all(np.isfinite(next_vector)):
            raise ValueError("Non-finite simulation state.")

        return SimulationState(
            target=self.output_adapter.physical_state(next_vector, control, state.time + dt, context),
            tracking_position_world=state.tracking_position_world,
            time=state.time + dt,
            physical_base=state.physical_base,
            internal_state=next_vector if self.output_adapter.mapped else None,
        )
