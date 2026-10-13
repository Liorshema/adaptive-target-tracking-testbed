
"""Bridge robot-state kinematics and safe tracking computation.

This module is independent of ROS, Gazebo, and actuator interfaces.
It computes raw and constrained commands without executing them.
"""

from typing import Optional

from numpy.typing import NDArray
import numpy as np

from robot_control.adapters.planar_kinematics_adapter import (
    PlanarKinematicsAdapter,
)
from robot_control.interfaces.control_data import (
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.pipeline.safe_tracking_controller import (
    SafeTrackingController,
    SafeTrackingResult,
)

Array = NDArray[np.float64]


class ControlExecutionPipeline:
    """Adapt robot state and delegate safe tracking computation.

    The kinematics adapter and safe tracking implementation are
    injected externally, allowing alternative robot-state layouts
    without modifying this component.
    """

    def __init__(
        self,
        kinematics_adapter: PlanarKinematicsAdapter,
        controller: SafeTrackingController,
    ) -> None:
        """Configure the execution pipeline.

        Args:
            kinematics_adapter: Converts robot state into model inputs.
            controller: Computes tracking and constrained outputs.
        """
        if not isinstance(kinematics_adapter, PlanarKinematicsAdapter):
            raise TypeError(
                "kinematics_adapter must be PlanarKinematicsAdapter"
            )

        if not isinstance(controller, SafeTrackingController):
            raise TypeError(
                "controller must be SafeTrackingController"
            )

        self._kinematics_adapter = kinematics_adapter
        self._controller = controller

    def compute(
        self,
        robot_state: RobotState,
        prediction: PredictedTargetTrajectory,
        now: float,
        previous_velocity: Optional[Array] = None,
        dt: Optional[float] = None,
    ) -> SafeTrackingResult:
        """Compute raw and safe commands without executing them.

        Args:
            robot_state: Current validated control-state estimate.
            prediction: Validated future target trajectory.
            now: Current time in the prediction clock.
            previous_velocity: Previous applied command or velocity,
                according to the configured acceleration policy.
            dt: Positive time interval used by safety constraints.

        Returns:
            SafeTrackingResult containing raw and constrained commands.

        Notes:
            Input freshness and frame consistency must be validated
            upstream. This function does not publish commands.
        """
        if not isinstance(robot_state, RobotState):
            raise TypeError("robot_state must be RobotState")

        if not isinstance(prediction, PredictedTargetTrajectory):
            raise TypeError(
                "prediction must be PredictedTargetTrajectory"
            )

        kinematics = self._kinematics_adapter.convert(robot_state)

        return self._controller.compute(
            robot_state=robot_state,
            prediction=prediction,
            transform_world_base=kinematics.transform_world_base,
            joint_positions=kinematics.joint_positions,
            now=now,
            previous_velocity=previous_velocity,
            dt=dt,
        )
