
"""Unified safety projection for generalized velocity commands.

This module combines model-defined velocity, acceleration, and
joint-position constraints into one feasible velocity interval.

It is independent of ROS, Gazebo, and specific controllers.
"""

from typing import Optional

import numpy as np
from numpy.typing import NDArray
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits

from robot_control.interfaces.control_data import ControlMode, ControlOutput
from robot_control.safety.constraint_bounds import (
    acceleration_bounds,
    joint_position_bounds,
    velocity_bounds,
)
from robot_control.safety.constraint_projector import ConstraintProjector

Array = NDArray[np.float64]


class SafeControlPipeline:
    """Project a controller output onto configured robot constraints.

    Velocity limits are mandatory. Acceleration and joint-position
    limits are optional and can be configured independently.

    All constraints are intersected before applying the projection.
    No previous command or robot state is stored internally.
    """

    def __init__(
        self,
        velocity_limits: VelocityLimits,
        acceleration_limits: Optional[AccelerationLimits] = None,
        joint_limits: Optional[JointLimits] = None,
        joint_velocity_indices: Optional[Array] = None,
    ) -> None:
        """Configure the safety constraints.

        Args:
            velocity_limits: Whole-body generalized velocity limits.
            acceleration_limits: Optional generalized acceleration limits.
            joint_limits: Optional manipulator joint-position limits.
            joint_velocity_indices: Indices of constrained joint
                velocities in the whole-body command vector.
        """
        if not isinstance(velocity_limits, VelocityLimits):
            raise TypeError("velocity_limits must be VelocityLimits")

        if (
            acceleration_limits is not None
            and not isinstance(acceleration_limits, AccelerationLimits)
        ):
            raise TypeError(
                "acceleration_limits must be AccelerationLimits"
            )

        if (
            joint_limits is not None
            and not isinstance(joint_limits, JointLimits)
        ):
            raise TypeError("joint_limits must be JointLimits")

        if (joint_limits is None) != (joint_velocity_indices is None):
            raise ValueError(
                "joint_limits and joint_velocity_indices "
                "must be supplied together"
            )

        lower, upper = velocity_bounds(velocity_limits)
        dimension = lower.size

        if acceleration_limits is not None:
            acceleration_lower = np.asarray(
                acceleration_limits.lower,
                dtype=np.float64,
            )
            acceleration_upper = np.asarray(
                acceleration_limits.upper,
                dtype=np.float64,
            )

            if (
                acceleration_lower.shape != (dimension,)
                or acceleration_upper.shape != (dimension,)
            ):
                raise ValueError(
                    "Acceleration limit dimension does not "
                    "match velocity limits"
                )

            if (
                not np.all(np.isfinite(acceleration_lower))
                or not np.all(np.isfinite(acceleration_upper))
                or np.any(acceleration_lower > acceleration_upper)
            ):
                raise ValueError("Invalid acceleration limits")

        indices = None

        if joint_limits is not None:
            indices = np.asarray(joint_velocity_indices)

            if (
                indices.ndim != 1
                or not np.issubdtype(indices.dtype, np.integer)
            ):
                raise ValueError(
                    "joint_velocity_indices must be a 1D integer array"
                )

            if indices.size != joint_limits.dimension:
                raise ValueError(
                    "Joint indices must match joint-limit dimension"
                )

            if (
                np.any(indices < 0)
                or np.any(indices >= dimension)
                or np.unique(indices).size != indices.size
            ):
                raise ValueError(
                    "Joint velocity indices must be valid and unique"
                )

            indices = indices.astype(int, copy=True)

        self._velocity_limits = velocity_limits
        self._acceleration_limits = acceleration_limits
        self._joint_limits = joint_limits
        self._joint_velocity_indices = indices
        self._projector = ConstraintProjector()

    def apply(
        self,
        output: ControlOutput,
        previous_velocity: Optional[Array] = None,
        joint_positions: Optional[Array] = None,
        dt: Optional[float] = None,
    ) -> ControlOutput:
        """Return the nearest feasible generalized velocity command.

        Args:
            output: Raw controller output.
            previous_velocity: Previous velocity command, required
                when acceleration limits are configured.
            joint_positions: Current joint positions, required
                when joint-position limits are configured.
            dt: Control interval, required for acceleration
                and joint-position constraints.

        Returns:
            Velocity-mode ControlOutput satisfying all constraints.

        Raises:
            TypeError: If the control output has the wrong type.
            ValueError: If inputs or constraints are invalid or
                their feasible intersection is empty.

        Notes:
            Only the current command is projected. Any optional
            predicted command sequence is not preserved.
        """
        if not isinstance(output, ControlOutput):
            raise TypeError("output must be ControlOutput")

        if output.mode != ControlMode.VELOCITY:
            raise ValueError(
                "SafeControlPipeline supports velocity mode only"
            )

        return self._projector.project(
            output=output,
            bounds=[self.feasible_bounds(previous_velocity, joint_positions, dt)],
        )

    @property
    def velocity_limits(self):
        """Return the configured physical/operational velocity contract."""
        return self._velocity_limits

    @property
    def joint_limits(self):
        """Return the optional physical position contract."""
        return self._joint_limits

    @property
    def joint_velocity_indices(self):
        """Return the named-order joint indices, independent of robot dimension."""
        return self._joint_velocity_indices

    def feasible_bounds(self, previous_velocity=None, joint_positions=None, dt=None):
        """Share one feasible interval with optimization and final safety projection."""
        bounds = [velocity_bounds(self._velocity_limits)]

        if self._acceleration_limits is not None:
            if previous_velocity is None or dt is None:
                raise ValueError(
                    "Acceleration constraints require "
                    "previous_velocity and dt"
                )

            bounds.append(
                acceleration_bounds(
                    limits=self._acceleration_limits,
                    previous_velocity=previous_velocity,
                    dt=dt,
                )
            )

        if self._joint_limits is not None:
            if joint_positions is None or dt is None:
                raise ValueError(
                    "Joint-position constraints require "
                    "joint_positions and dt"
                )

            bounds.append(
                joint_position_bounds(
                    joint_limits=self._joint_limits,
                    joint_positions=joint_positions,
                    joint_velocity_indices=self._joint_velocity_indices,
                    velocity_limits=self._velocity_limits,
                    dt=dt,
                )
            )

        lower = np.maximum.reduce([bound[0] for bound in bounds])
        upper = np.minimum.reduce([bound[1] for bound in bounds])
        if np.any(lower > upper):
            raise ValueError('Infeasible constraints: empty intersection')
        return lower, upper
