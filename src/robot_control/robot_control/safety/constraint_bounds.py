
"""Convert robot-model constraints into generalized velocity bounds.

This module performs constraint conversion only.
Projection is handled separately by ConstraintProjector.
"""

import numpy as np
from numpy.typing import NDArray

from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits

Array = NDArray[np.float64]
Bounds = tuple[Array, Array]


class JointPositionViolation(ValueError):
    """Carry exact model-order violations for optional named transport diagnostics."""

    def __init__(self, positions, lower, upper):
        """Retain only violated joints in authoritative model order."""
        self.violations = [(i, float(q), float(lo), float(hi))
                           for i, (q, lo, hi) in enumerate(zip(positions, lower, upper))
                           if q < lo or q > hi]
        super().__init__(self.describe())

    def describe(self, names=None):
        """Use externally supplied names; algorithms know only model joint order."""
        details = []
        for i, q, lo, hi in self.violations:
            name = names[i] if names is not None and i < len(names) else f'joint[{i}]'
            details.append(f'{name} q={q:.12g} rad bounds=[{lo:.12g},{hi:.12g}] rad '
                           f'excess={max(lo - q, q - hi):.6g} rad')
        return 'Current joint positions violate limits: ' + '; '.join(details)


def _vector(value: Array, name: str) -> Array:
    """Validate and copy a finite one-dimensional vector."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.ndim != 1 or result.size == 0:
        raise ValueError(f"{name} must be a nonempty 1D vector")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")

    return result


def _positive_dt(dt: float) -> float:
    """Validate a positive, finite control interval."""
    if not np.isscalar(dt) or not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive and finite")

    return float(dt)


def velocity_bounds(limits: VelocityLimits) -> Bounds:
    """Return generalized velocity bounds from the robot model."""
    if not isinstance(limits, VelocityLimits):
        raise TypeError("limits must be VelocityLimits")

    lower = _vector(limits.lower, "velocity lower")
    upper = _vector(limits.upper, "velocity upper")

    if lower.shape != upper.shape or np.any(lower > upper):
        raise ValueError("Invalid velocity limits")

    return lower, upper


def acceleration_bounds(
    limits: AccelerationLimits,
    previous_velocity: Array,
    dt: float,
) -> Bounds:
    """Convert acceleration limits into velocity bounds."""
    if not isinstance(limits, AccelerationLimits):
        raise TypeError("limits must be AccelerationLimits")

    previous = _vector(previous_velocity, "previous_velocity")
    lower_acc = _vector(limits.lower, "acceleration lower")
    upper_acc = _vector(limits.upper, "acceleration upper")
    dt = _positive_dt(dt)

    if (
        previous.shape != lower_acc.shape
        or lower_acc.shape != upper_acc.shape
    ):
        raise ValueError("Acceleration dimensions do not match")

    if np.any(lower_acc > upper_acc):
        raise ValueError("Invalid acceleration limits")

    lower = previous + dt * lower_acc
    upper = previous + dt * upper_acc

    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError("Computed acceleration bounds are nonfinite")

    return lower, upper


def joint_position_bounds(
    joint_limits: JointLimits,
    joint_positions: Array,
    joint_velocity_indices: Array,
    velocity_limits: VelocityLimits,
    dt: float,
) -> Bounds:
    """Convert joint-position limits into generalized velocity bounds.

    Unconstrained components retain their physical velocity limits.

    Args:
        joint_limits: Position limits for constrained joints.
        joint_positions: Current positions in joint-limit order.
        joint_velocity_indices: Indices in generalized velocity.
        velocity_limits: Whole-body velocity limits.
        dt: Positive control interval.

    Returns:
        Lower and upper whole-body velocity bounds.
    """
    if not isinstance(joint_limits, JointLimits):
        raise TypeError("joint_limits must be JointLimits")

    positions = _vector(joint_positions, "joint_positions")
    indices = np.asarray(joint_velocity_indices)
    dt = _positive_dt(dt)

    lower, upper = velocity_bounds(velocity_limits)

    if indices.ndim != 1 or not np.issubdtype(
        indices.dtype, np.integer
    ):
        raise ValueError("Joint indices must be a 1D integer array")

    if (
        positions.size != joint_limits.dimension
        or indices.size != joint_limits.dimension
    ):
        raise ValueError("Joint dimensions do not match")

    if (
        np.any(indices < 0)
        or np.any(indices >= lower.size)
        or np.unique(indices).size != indices.size
    ):
        raise ValueError("Joint indices must be valid and unique")

    joint_lower = _vector(joint_limits.lower, "joint lower")
    joint_upper = _vector(joint_limits.upper, "joint upper")

    if not joint_limits.contains(positions):
        raise JointPositionViolation(positions, joint_lower, joint_upper)

    lower[indices] = np.maximum(
        lower[indices],
        (joint_lower - positions) / dt,
    )

    upper[indices] = np.minimum(
        upper[indices],
        (joint_upper - positions) / dt,
    )

    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError("Computed joint bounds are nonfinite")

    # Do not hide infeasibility by clipping or swapping bounds.
    return lower, upper
