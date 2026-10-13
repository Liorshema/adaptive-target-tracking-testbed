
"""Joint-position-aware limiting of generalized velocity commands."""

import numpy as np
from numpy.typing import NDArray
from robot_models.constraints.joint_limits import JointLimits

from robot_control.interfaces.control_data import ControlMode, ControlOutput

Array = NDArray[np.float64]


def _vector(value: Array, name: str) -> Array:
    """Validate and copy a finite one-dimensional vector."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.ndim != 1 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 1D vector")

    return result


class JointPositionLimiter:
    """Limit joint velocities using next-step position constraints.

    The base/generalized velocity layout is supplied externally.
    Only the configured joint-velocity components are modified.

    The current joint state must already satisfy the joint limits.
    """

    def __init__(
        self,
        joint_limits: JointLimits,
        joint_velocity_indices: Array,
    ) -> None:
        """Configure joint limits and generalized velocity indices."""
        if not isinstance(joint_limits, JointLimits):
            raise TypeError("joint_limits must be JointLimits")

        indices = np.asarray(joint_velocity_indices)

        if indices.ndim != 1:
            raise ValueError("joint_velocity_indices must be 1D")

        if not np.issubdtype(indices.dtype, np.integer):
            raise ValueError("joint_velocity_indices must be integers")

        if indices.size != joint_limits.dimension:
            raise ValueError(
                "Number of joint indices must match joint limits"
            )

        if np.any(indices < 0) or np.unique(indices).size != indices.size:
            raise ValueError(
                "Joint indices must be nonnegative and unique"
            )

        self._limits = joint_limits
        self._indices = indices.astype(int, copy=True)

        self._lower = _vector(joint_limits.lower, "joint lower limits")
        self._upper = _vector(joint_limits.upper, "joint upper limits")

    def limit(
        self,
        output: ControlOutput,
        joint_positions: Array,
        dt: float,
    ) -> ControlOutput:
        """Limit velocities to keep predicted joint positions valid.

        Args:
            output: Requested generalized velocity command.
            joint_positions: Current joint positions in limit order.
            dt: Positive control interval in seconds.

        Returns:
            A new ControlOutput with position-limited joint rates.

        Raises:
            ValueError: On invalid state, timing, or dimensions.
        """
        if not isinstance(output, ControlOutput):
            raise TypeError("output must be ControlOutput")

        if output.mode != ControlMode.VELOCITY:
            raise ValueError("Only velocity commands are supported")

        command = _vector(output.command, "command")
        positions = _vector(joint_positions, "joint_positions")

        if positions.shape != self._lower.shape:
            raise ValueError("Joint position dimension mismatch")

        if self._indices.size and np.max(self._indices) >= command.size:
            raise ValueError("Joint index exceeds command dimension")

        if not np.isscalar(dt) or not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")

        if not self._limits.contains(positions):
            raise ValueError(
                "Joint state is outside its position limits"
            )

        lower_velocity = (self._lower - positions) / dt
        upper_velocity = (self._upper - positions) / dt

        limited = command.copy()

        limited[self._indices] = np.clip(
            command[self._indices],
            lower_velocity,
            upper_velocity,
        )

        predicted_positions = (
            positions + dt * limited[self._indices]
        )

        if not self._limits.contains(
            np.clip(predicted_positions, self._lower, self._upper)
        ):
            raise ValueError("Joint position verification failed")

        tolerance = 1e-10
        if (
            np.any(predicted_positions < self._lower - tolerance)
            or np.any(predicted_positions > self._upper + tolerance)
        ):
            raise ValueError("Predicted joint positions exceed limits")

        return ControlOutput(
            command=limited,
            mode=ControlMode.VELOCITY,
        )
