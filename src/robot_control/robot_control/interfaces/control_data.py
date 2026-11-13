
"""Generic mathematical data contracts for robot control.

This module is independent of ROS, Gazebo, robot geometry,
and specific control algorithms.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np
from numpy.typing import NDArray


Array = NDArray[np.float64]


def _vector(value: Array, name: str) -> Array:
    """Return a validated, independent 1D numerical array."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.ndim != 1 or result.size == 0:
        raise ValueError(f"{name} must be a nonempty 1D vector")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains nonfinite values")

    return result


def _matrix(value: Array, name: str) -> Array:
    """Return a validated, independent 2D numerical array."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.ndim != 2 or 0 in result.shape:
        raise ValueError(f"{name} must be a nonempty 2D matrix")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains nonfinite values")

    return result


def _timestamp(value: float, name: str) -> float:
    """Validate a finite scalar timestamp."""
    if not np.isscalar(value) or not np.isfinite(value):
        raise ValueError(f"{name} must be a finite scalar")

    return float(value)


@dataclass(frozen=True)
class RobotState:
    """Robot configuration and generalized velocity."""

    configuration: Array
    velocity: Array
    timestamp: float
    frame_id: str

    def __post_init__(self) -> None:
        """Validate and normalize the robot state after initialization."""
        object.__setattr__(
            self, "configuration",
            _vector(self.configuration, "configuration"),
        )
        object.__setattr__(
            self, "velocity",
            _vector(self.velocity, "velocity"),
        )
        object.__setattr__(
            self, "timestamp",
            _timestamp(self.timestamp, "timestamp"),
        )

        if not self.frame_id:
            raise ValueError("frame_id cannot be empty")


@dataclass(frozen=True)
class TrackingReference:
    """Desired task-space reference at a single instant."""

    position: Array
    velocity: Optional[Array] = None

    def __post_init__(self) -> None:
        """Validate reference position and optional velocity."""
        position = _vector(self.position, "position")
        object.__setattr__(self, "position", position)

        if self.velocity is not None:
            velocity = _vector(self.velocity, "velocity")

            if velocity.shape != position.shape:
                raise ValueError(
                    "Reference position and velocity dimensions differ"
                )

            object.__setattr__(self, "velocity", velocity)


@dataclass(frozen=True)
class ReferenceTrajectory:
    """Time-indexed sequence of task-space references."""

    positions: Array
    timestamps: Array
    frame_id: str
    velocities: Optional[Array] = None

    def __post_init__(self) -> None:
        """Validate reference samples, timestamps, and velocities."""
        positions = _matrix(self.positions, "positions")
        timestamps = _vector(self.timestamps, "timestamps")

        if positions.shape[0] != timestamps.size:
            raise ValueError(
                "Reference trajectory and timestamps length differ"
            )

        if np.any(np.diff(timestamps) <= 0):
            raise ValueError(
                "Reference timestamps must be strictly increasing"
            )

        if not self.frame_id:
            raise ValueError("frame_id cannot be empty")

        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "timestamps", timestamps)

        if self.velocities is not None:
            velocities = _matrix(self.velocities, "velocities")

            if velocities.shape != positions.shape:
                raise ValueError(
                    "Reference velocities must match positions shape"
                )

            object.__setattr__(self, "velocities", velocities)


@dataclass(frozen=True)
class PredictedTargetTrajectory:
    """Time-indexed predicted target states."""

    states: Array
    timestamps: Array
    frame_id: str

    def __post_init__(self) -> None:
        """Validate predicted states and their temporal ordering."""
        states = _matrix(self.states, "states")
        timestamps = _vector(self.timestamps, "timestamps")

        if states.shape[0] != timestamps.size:
            raise ValueError(
                "Predicted states and timestamps length differ"
            )

        if np.any(np.diff(timestamps) <= 0):
            raise ValueError(
                "Prediction timestamps must be strictly increasing"
            )

        if not self.frame_id:
            raise ValueError("frame_id cannot be empty")

        object.__setattr__(self, "states", states)
        object.__setattr__(self, "timestamps", timestamps)

    def sample(self, query_times: Array) -> Array:
        """Interpolate position/velocity, permitting one initial forecast interval.

        Forecasts begin after their reference epoch. Only that initial interval
        may be backprojected using supplied velocity; never extrapolate the end.
        Reference-epoch freshness remains the transport InputValidator's job.
        """
        times = _vector(self.timestamps, 'prediction timestamps')
        states = _matrix(self.states, 'prediction states')
        queries = _vector(query_times, 'prediction query times')
        if (states.shape[0] != len(times) or states.shape[1] not in (3, 6)
                or len(times) < 2 or np.any(np.diff(times) <= 0)):
            raise ValueError('Prediction requires ordered position[/velocity] samples')
        if queries.max() > times[-1] + 1e-9:
            raise ValueError('Insufficient prediction horizon')
        early = queries < times[0]
        if np.any(early) and (
                states.shape[1] != 6
                or times[0] - queries.min() > times[1] - times[0] + 1e-9):
            raise ValueError('Query precedes the supported prediction interval')
        result = np.column_stack([np.interp(queries, times, states[:, i])
                                  for i in range(states.shape[1])])
        if states.shape[1] == 3:
            result = np.column_stack([result, np.zeros((len(queries), 3))])
        if np.any(early):
            result[early, :3] += (queries[early] - times[0])[:, None] * states[0, 3:]
        return result


class ControlMode(str, Enum):
    """Physical interpretation of the commanded quantities."""

    VELOCITY = "velocity"
    ACCELERATION = "acceleration"
    EFFORT = "effort"


@dataclass(frozen=True)
class ControlInput:
    """Common inputs for interchangeable control algorithms."""

    robot_state: RobotState
    reference: TrackingReference
    reference_trajectory: Optional[ReferenceTrajectory] = None
    target_prediction: Optional[PredictedTargetTrajectory] = None
    target_state: Optional[Array] = None
    dt: Optional[float] = None
    previous_velocity: Optional[Array] = None
    evaluation_time: Optional[float] = None


@dataclass(frozen=True)
class ControlOutput:
    """Controller command and its physical interpretation."""

    command: Array
    mode: ControlMode
    predicted_commands: Optional[Array] = None

    def __post_init__(self) -> None:
        """Validate the command and optional predicted command sequence."""
        command = _vector(self.command, "command")
        object.__setattr__(self, "command", command)

        if not isinstance(self.mode, ControlMode):
            raise ValueError("mode must be a ControlMode")

        if self.predicted_commands is not None:
            commands = _matrix(
                self.predicted_commands, "predicted_commands"
            )

            if commands.shape[1] != command.size:
                raise ValueError(
                    "Predicted command dimension must match command"
                )

            object.__setattr__(
                self, "predicted_commands", commands
            )
