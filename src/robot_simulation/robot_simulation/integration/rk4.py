
"""Generic fourth-order Runge-Kutta numerical integrator."""

from typing import Any, Mapping

import numpy as np

from robot_simulation.interfaces.dynamics import Dynamics


class RK4Integrator:
    """Integrate continuous-time state-space dynamics using RK4."""

    def step(
        self,
        dynamics: Dynamics,
        state: np.ndarray,
        control: np.ndarray,
        time: float,
        dt: float,
        context: Mapping[str, Any],
    ) -> np.ndarray:
        """Advance the state by one integration step."""
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive.")

        state = np.asarray(state, dtype=float)

        if state.ndim != 1 or not np.all(np.isfinite(state)):
            raise ValueError("state must be a finite 1D vector.")

        def derivative(x: np.ndarray, t: float) -> np.ndarray:
            result = np.asarray(
                dynamics.derivative(x, control, t, context),
                dtype=float,
            )

            if result.shape != state.shape:
                raise ValueError("Dynamics derivative dimension mismatch.")

            if not np.all(np.isfinite(result)):
                raise ValueError("Dynamics returned a non-finite derivative.")

            return result

        k1 = derivative(state, time)
        k2 = derivative(state + 0.5 * dt * k1, time + 0.5 * dt)
        k3 = derivative(state + 0.5 * dt * k2, time + 0.5 * dt)
        k4 = derivative(state + dt * k3, time + dt)

        return state + (dt / 6.0) * (
            k1 + 2.0 * k2 + 2.0 * k3 + k4
        )
