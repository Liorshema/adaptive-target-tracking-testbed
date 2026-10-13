
"""Compose prediction-based tracking with command safety.

This module is independent of ROS, Gazebo, and actuator interfaces.
It calculates and constrains commands without executing them.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from robot_control.interfaces.control_data import (
    ControlOutput,
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.pipeline.tracking_control_orchestrator import (
    TrackingControlOrchestrator,
)
from robot_control.safety.safe_control_pipeline import SafeControlPipeline

Array = NDArray[np.float64]


@dataclass(frozen=True)
class SafeTrackingResult:
    """Preserve both the requested and constrained control outputs."""

    raw_output: ControlOutput
    safe_output: ControlOutput


class SafeTrackingController:
    """Combine tracking computation and independent command constraints."""

    def __init__(
        self,
        tracking: TrackingControlOrchestrator,
        safety: SafeControlPipeline,
    ) -> None:
        """Inject tracking and safety implementations."""
        if not isinstance(tracking, TrackingControlOrchestrator):
            raise TypeError(
                "tracking must be TrackingControlOrchestrator"
            )

        if not isinstance(safety, SafeControlPipeline):
            raise TypeError("safety must be SafeControlPipeline")

        self._tracking = tracking
        self._safety = safety

    def compute(
        self,
        robot_state: RobotState,
        prediction: PredictedTargetTrajectory,
        transform_world_base: Array,
        joint_positions: Array,
        now: float,
        previous_velocity: Optional[Array] = None,
        dt: Optional[float] = None,
    ) -> SafeTrackingResult:
        """Compute tracking output and project it onto safety limits.

        Args:
            robot_state: Current estimated robot state.
            prediction: Predicted target trajectory.
            transform_world_base: Current 4x4 base pose in world.
            joint_positions: Current manipulator configuration.
            now: Current time in the prediction clock.
            previous_velocity: Velocity at the previous control step.
            dt: Positive interval between control steps.

        Returns:
            Raw and safety-constrained commands.

        Raises:
            ValueError: If tracking or constraints cannot be satisfied.

        Notes:
            Data freshness must be checked by the upstream validator.
            This component never publishes or executes commands.
        """
        raw_output = self._tracking.compute(
            robot_state=robot_state,
            prediction=prediction,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
            now=now,
        )

        safe_output = self._safety.apply(
            output=raw_output,
            previous_velocity=previous_velocity,
            joint_positions=joint_positions,
            dt=dt,
        )

        return SafeTrackingResult(
            raw_output=raw_output,
            safe_output=safe_output,
        )
