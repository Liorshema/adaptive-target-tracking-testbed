
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

    def radial_acceleration(self, distance: float) -> float:
        """Return the original linear radial interaction in acceleration units."""
        return self.alpha * (distance - self.preferred_distance)


@dataclass(frozen=True)
class SaturatedFightFlightParameters:
    """Bounded interaction: acceleration m/s², lengths m, damping s⁻¹."""

    a_max: float
    transition_length: float
    preferred_distance: float
    damping: float
    distance_epsilon: float

    def __post_init__(self) -> None:
        """Reject non-finite or nonphysical acceleration and distance parameters."""
        if not np.all(np.isfinite([
            self.a_max, self.transition_length, self.preferred_distance,
            self.damping, self.distance_epsilon,
        ])):
            raise ValueError('Saturated interaction parameters must be finite.')
        if min(self.a_max, self.transition_length,
               self.preferred_distance, self.distance_epsilon) <= 0.0:
            raise ValueError('Acceleration and distance parameters must be positive.')
        if self.damping < 0.0:
            raise ValueError('damping must be non-negative.')

    def radial_acceleration(self, distance: float) -> float:
        """Return smooth attraction/repulsion bounded by a_max."""
        return self.a_max * np.tanh(
            (distance - self.preferred_distance) / self.transition_length
        )


@dataclass(frozen=True)
class TangentialInteractionParameters:
    """Spatial circulation: acceleration in m/s² and a world-frame axis."""

    acceleration: float
    axis_world: tuple[float, float, float]

    def __post_init__(self) -> None:
        """Normalize the configured axis; reject undefined circulation geometry."""
        axis = np.asarray(self.axis_world, dtype=float)
        if not np.isfinite(self.acceleration) or self.acceleration < 0.0:
            raise ValueError('Tangential acceleration must be finite and non-negative.')
        if axis.shape != (3,) or not np.isfinite(axis).all():
            raise ValueError('Tangential axis must be a finite 3D vector.')
        norm = np.linalg.norm(axis)
        if not np.isfinite(norm) or norm <= 0.0:
            raise ValueError('Tangential axis must have nonzero finite length.')
        object.__setattr__(self, 'axis_world', tuple(axis / norm))

    def contribution(self, displacement: np.ndarray, epsilon: float) -> np.ndarray:
        """Bounded tangent, smoothly zero at coincidence and circulation poles."""
        if displacement.shape != (3,):
            raise ValueError('Tangential interaction requires 3D positions.')
        tangent = np.cross(self.axis_world, displacement)
        return self.acceleration * tangent / np.sqrt(tangent @ tangent + epsilon**2)


class FightFlightDynamics(Dynamics):
    """Target dynamics with optional environmental acceleration."""

    def __init__(
        self,
        parameters: FightFlightParameters | SaturatedFightFlightParameters,
        environment_field: AccelerationField | None = None,
        tangential: TangentialInteractionParameters | None = None,
    ) -> None:
        self.parameters = parameters
        self.environment_field = environment_field
        self.tangential = tangential

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
            self.parameters.radial_acceleration(distance)
            * direction
            - self.parameters.damping * velocity
        )

        if self.tangential is not None and self.tangential.acceleration > 0.0:
            acceleration += self.tangential.contribution(
                displacement, self.parameters.distance_epsilon)

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
