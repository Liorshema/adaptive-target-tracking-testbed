
"""Continuous-time Fight / Flight target dynamics."""

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from robot_simulation.interfaces.dynamics import Dynamics
from robot_simulation.interfaces.disturbance import AccelerationField


@dataclass(frozen=True)
class FightFlightParameters:
    """Parameters of the radial Fight / Flight interaction."""

    alpha: float
    preferred_distance: float
    damping: float
    distance_epsilon: float

    def __post_init__(self) -> None:
        if self.alpha < 0.0:
            raise ValueError("alpha must be non-negative.")
        if self.preferred_distance <= 0.0:
            raise ValueError("preferred_distance must be positive.")
        if self.damping < 0.0:
            raise ValueError("damping must be non-negative.")
        if self.distance_epsilon <= 0.0:
            raise ValueError("distance_epsilon must be positive.")


class FightFlightDynamics(Dynamics):
    """Target dynamics with optional environmental acceleration."""

    def __init__(
        self,
        parameters: FightFlightParameters,
        environment_field: AccelerationField | None = None,
    ) -> None:
        self.parameters = parameters
        self.environment_field = environment_field

    def derivative(
        self,
        state: np.ndarray,
        control: np.ndarray,
        time: float,
        context: Mapping[str, Any],
    ) -> np.ndarray:
        """Compute the continuous-time target-state derivative."""
        del control

        state = np.asarray(state, dtype=float)

        if state.ndim != 1 or state.size == 0 or state.size % 2 != 0:
            raise ValueError("state must contain [position, velocity].")

        dimension = state.size // 2
        position = state[:dimension]
        velocity = state[dimension:]

        tracking_position = np.asarray(
            context["tracking_position_world"],
            dtype=float,
        )

        if tracking_position.shape != position.shape:
            raise ValueError("tracking position dimension mismatch.")

        displacement = position - tracking_position
        distance = np.linalg.norm(displacement)

        direction = displacement / np.sqrt(
            distance**2 + self.parameters.distance_epsilon**2
        )

        acceleration = (
            self.parameters.alpha
            * (distance - self.parameters.preferred_distance)
            * direction
            - self.parameters.damping * velocity
        )

        if self.environment_field is not None:
            external_acceleration = np.asarray(
                self.environment_field.acceleration(position, time),
                dtype=float,
            )

            if external_acceleration.shape != position.shape:
                raise ValueError("External acceleration dimension mismatch.")

            if not np.all(np.isfinite(external_acceleration)):
                raise ValueError("External acceleration must be finite.")

            acceleration = acceleration + external_acceleration

        return np.concatenate((velocity, acceleration))
