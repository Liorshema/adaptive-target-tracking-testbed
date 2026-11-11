
"""Conversions between structured target states and state vectors."""

import numpy as np

from robot_models.target.target_state import TargetState


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


class TargetStateAdapter:
    """Keep complete declared mathematical state separate from physical target output."""

    def __init__(self, dynamics):
        """Select the owner's declared mapping capability, not a motion category."""
        self.dynamics = dynamics
        self.mapped = bool(getattr(dynamics, 'uses_mapped_output', False))

    def initial_vector(self, physical_initial):
        """Use model initials for mapped motion and existing p/v overrides for Cartesian."""
        return (self.dynamics.initial_state if self.mapped
                else state_to_vector(physical_initial))

    def vector(self, state):
        """Never reconstruct or truncate a mapped vector from its physical output."""
        if self.mapped:
            if state.internal_state is None:
                raise ValueError('Mapped target requires complete initialized internal_state')
            return state.internal_state
        if state.internal_state is not None:
            raise ValueError('Cartesian target cannot reinterpret an internal mapped state')
        return state_to_vector(state.target)

    def physical_state(self, vector, control, time, context):
        """Evaluate position and analytical velocity at the identical committed epoch."""
        if not self.mapped:
            return vector_to_state(vector)
        position, velocity = self.dynamics.physical_output(vector, control, time, context)
        return TargetState(position=position, velocity=velocity)
