
"""Model-based limits for generalized velocity commands.

This module is independent of ROS, Gazebo, and control algorithms.
"""

import numpy as np
from numpy.typing import NDArray
from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.velocity_limits import VelocityLimits

from robot_control.interfaces.control_data import ControlMode, ControlOutput

Array = NDArray[np.float64]


def _finite_vector(value: Array, name: str) -> Array:
    """Return a validated copy of a finite vector."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.ndim != 1 or result.size == 0:
        raise ValueError(f"{name} must be a nonempty 1D vector")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")

    return result


class CommandLimiter:
    """Apply velocity and optional acceleration constraints.

    Limits are supplied by robot_models. The limiter computes
    the nearest admissible generalized velocity command.

    The previous velocity is supplied by the caller, keeping
    the limiter stateless and independently testable.
    """

    def __init__(
        self,
        velocity_limits: VelocityLimits,
        acceleration_limits: AccelerationLimits | None = None,
    ) -> None:
        """Configure velocity and optional acceleration bounds."""
        if not isinstance(velocity_limits, VelocityLimits):
            raise TypeError("velocity_limits must be VelocityLimits")

        self._velocity_limits = velocity_limits
        self._acceleration_limits = acceleration_limits

        if (
            acceleration_limits is not None
            and not isinstance(acceleration_limits, AccelerationLimits)
        ):
            raise TypeError(
                "acceleration_limits must be AccelerationLimits"
            )

        velocity_lower = _finite_vector(
            velocity_limits.lower, "velocity lower"
        )
        velocity_upper = _finite_vector(
            velocity_limits.upper, "velocity upper"
        )

        if velocity_lower.shape != velocity_upper.shape:
            raise ValueError("Velocity limit dimensions differ")

        if np.any(velocity_lower > velocity_upper):
            raise ValueError("Invalid velocity limit ordering")

        self._velocity_lower = velocity_lower
        self._velocity_upper = velocity_upper

        if acceleration_limits is not None:
            acceleration_lower = _finite_vector(
                acceleration_limits.lower, "acceleration lower"
            )
            acceleration_upper = _finite_vector(
                acceleration_limits.upper, "acceleration upper"
            )

            if acceleration_lower.shape != velocity_lower.shape:
                raise ValueError(
                    "Velocity and acceleration dimensions differ"
                )

            if np.any(acceleration_lower > acceleration_upper):
                raise ValueError("Invalid acceleration limit ordering")

            self._acceleration_lower = acceleration_lower
            self._acceleration_upper = acceleration_upper
        else:
            self._acceleration_lower = None
            self._acceleration_upper = None

    def limit(
        self,
        output: ControlOutput,
        previous_velocity: Array | None = None,
        dt: float | None = None,
    ) -> ControlOutput:
        """Return an admissible generalized velocity command.

        Args:
            output: Controller output in velocity mode.
            previous_velocity: Velocity at the previous control step.
            dt: Time interval since that step, in seconds.

        Returns:
            ControlOutput containing the bounded current command.

        Raises:
            ValueError: If inputs or constraints are inconsistent.
        """
        if not isinstance(output, ControlOutput):
            raise TypeError("output must be ControlOutput")

        if output.mode != ControlMode.VELOCITY:
            raise ValueError(
                "CommandLimiter currently supports velocity mode only"
            )

        requested = _finite_vector(output.command, "command")

        if requested.shape != self._velocity_lower.shape:
            raise ValueError("Command dimension does not match limits")

        lower = self._velocity_lower.copy()
        upper = self._velocity_upper.copy()

        if self._acceleration_limits is not None:
            if previous_velocity is None or dt is None:
                raise ValueError(
                    "Acceleration limiting requires previous_velocity and dt"
                )

            previous = _finite_vector(
                previous_velocity, "previous_velocity"
            )

            if previous.shape != requested.shape:
                raise ValueError("Previous velocity dimension mismatch")

            if not np.isfinite(dt) or dt <= 0.0:
                raise ValueError("dt must be positive and finite")

            lower = np.maximum(
                lower,
                previous + dt * self._acceleration_lower,
            )
            upper = np.minimum(
                upper,
                previous + dt * self._acceleration_upper,
            )

        if np.any(lower > upper):
            raise ValueError(
                "Velocity and acceleration constraints are infeasible"
            )

        command = np.clip(requested, lower, upper)

        # Verify against the configured model constraints.
        if not self._velocity_limits.contains(command):
            raise ValueError("Velocity constraint verification failed")

        if self._acceleration_limits is not None:
            if not self._acceleration_limits.contains(
                command, previous, dt
            ):
                raise ValueError(
                    "Acceleration constraint verification failed"
                )

        return ControlOutput(
            command=command,
            mode=ControlMode.VELOCITY,
        )
