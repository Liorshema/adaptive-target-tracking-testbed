
"""Validate temporal and frame consistency of control inputs."""

from dataclasses import dataclass

import numpy as np

from robot_control.interfaces.control_data import (
    PredictedTargetTrajectory,
    RobotState,
)


@dataclass(frozen=True)
class InputValidationConfig:
    """Configuration for control-input freshness checks."""

    max_robot_state_age: float
    max_prediction_age: float
    future_tolerance: float = 0.0
    lookahead_time: float = 0.0

    def __post_init__(self) -> None:
        """Validate nonnegative finite timing thresholds."""
        for name in (
            "max_robot_state_age",
            "max_prediction_age",
            "future_tolerance",
            "lookahead_time",
        ):
            value = getattr(self, name)

            if not np.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be finite and nonnegative"
                )


class InputValidator:
    """Check control input timestamps and coordinate-frame consistency."""

    def __init__(self, config: InputValidationConfig) -> None:
        """Initialize the validator with external timing thresholds."""
        self._config = config

    @staticmethod
    def _validate_age(
        timestamp: float,
        now: float,
        max_age: float,
        future_tolerance: float,
        name: str,
    ) -> None:
        """Reject stale or unexpectedly future-dated data."""
        if not np.isfinite(now) or not np.isfinite(timestamp):
            raise ValueError(f"{name}: timestamps must be finite")

        age = now - timestamp

        if age > max_age:
            raise ValueError(
                f"{name} is stale: age={age:.6f}s"
            )

        if age < -future_tolerance:
            raise ValueError(
                f"{name} timestamp is in the future"
            )

    def validate(
        self,
        robot_state: RobotState,
        prediction: PredictedTargetTrajectory,
        prediction_reference_timestamp: float,
        now: float,
    ) -> None:
        """Validate the robot state and target prediction together."""
        if robot_state.frame_id != prediction.frame_id:
            raise ValueError(
                "Robot and prediction frames do not match"
            )

        self._validate_age(
            timestamp=robot_state.timestamp,
            now=now,
            max_age=self._config.max_robot_state_age,
            future_tolerance=self._config.future_tolerance,
            name="Robot state",
        )

        self._validate_age(
            timestamp=prediction_reference_timestamp,
            now=now,
            max_age=self._config.max_prediction_age,
            future_tolerance=self._config.future_tolerance,
            name="Target prediction",
        )

        if prediction.timestamps[-1] < now + self._config.lookahead_time:
            raise ValueError(
                "Target prediction horizon has expired for requested lookahead"
            )
