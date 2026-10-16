"""Joint-position constraint utilities."""

import numpy as np

from robot_models.constraints.constraint_set import ConstraintSet


class JointLimits:
    """Represent manipulator joint-position limits."""

    def __init__(
        self,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> None:
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)

        if self.lower.ndim != 1:
            raise ValueError(
                'Lower joint limits must be a one-dimensional vector.'
            )

        if self.upper.shape != self.lower.shape:
            raise ValueError(
                'Upper and lower joint limits must have matching shapes.'
            )

        if np.any(self.lower > self.upper):
            raise ValueError(
                'Lower joint limits cannot exceed upper joint limits.'
            )

    @property
    def dimension(self) -> int:
        """Return the number of constrained joints."""
        return self.lower.size

    def as_constraint_set(self) -> ConstraintSet:
        """Return the limits in linear constraint form."""
        return ConstraintSet(
            matrix=np.eye(self.dimension),
            lower=self.lower.copy(),
            upper=self.upper.copy(),
        )

    def residual(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Return non-negative residuals when joint limits are satisfied.

        The output is stacked as

            [q - q_min,
             q_max - q].
        """
        joint_positions = np.asarray(
            joint_positions,
            dtype=float,
        )

        if joint_positions.shape != (self.dimension,):
            raise ValueError(
                'Joint-position vector has incompatible shape.'
            )

        return np.concatenate(
            (
                joint_positions - self.lower,
                self.upper - joint_positions,
            )
        )

    def contains(self, joint_positions: np.ndarray) -> bool:
        """Return whether all joint positions satisfy the limits."""
        return bool(
            np.all(
                self.residual(joint_positions) >= 0.0
            )
        )
