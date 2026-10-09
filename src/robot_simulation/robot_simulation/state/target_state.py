
"""Conversions between structured target states and state vectors."""

import numpy as np

from robot_models.types import TargetState


def state_to_vector(state: TargetState) -> np.ndarray:
    """Convert a target state to [position, velocity]."""
    return np.concatenate((state.position, state.velocity))


def vector_to_state(vector: np.ndarray) -> TargetState:
    """Convert a [position, velocity] vector to a target state."""
    vector = np.asarray(vector, dtype=float)

    if vector.shape != (6,):
        raise ValueError(
            f"Expected a 6D target state, got {vector.shape}."
        )

    return TargetState(
        position=vector[:3].copy(),
        velocity=vector[3:].copy(),
    )
