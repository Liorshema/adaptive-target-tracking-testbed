
"""Coordinate prediction-based tracking and control computation.

This module is independent of ROS, Gazebo, and actuator interfaces.
It generates control commands but never executes or publishes them.
"""

from typing import Optional

import numpy as np
from numpy.typing import NDArray
from robot_models.interfaces.robot_model import RobotModel
from robot_models.whole_body.kinematics import WholeBodyKinematics

from robot_control.interfaces.control_data import (
    ControlInput,
    ControlOutput,
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.interfaces.controller import Controller
from robot_control.pipeline.control_pipeline import ControlPipeline
from robot_control.tracking.tracking_frame_kinematics import (
    TrackingFrameKinematics,
)
from robot_control.tracking.tracking_reference_builder import (
    TrackingReferenceBuilder,
)

Array = NDArray[np.float64]


class TrackingControlOrchestrator:
    """Coordinate reference generation and interchangeable control."""

    def __init__(
        self,
        model: RobotModel,
        controller: Controller,
        reference_builder: TrackingReferenceBuilder,
        lookahead_time: float = 0.0,
        tracking_frame_kinematics: Optional[
            TrackingFrameKinematics
        ] = None,
    ) -> None:
        """Configure tracking using externally supplied components."""
        if not isinstance(model, RobotModel):
            raise TypeError("model must implement RobotModel")

        if not isinstance(controller, Controller):
            raise TypeError("controller must implement Controller")

        if not isinstance(
            reference_builder,
            TrackingReferenceBuilder,
        ):
            raise TypeError(
                "reference_builder must be TrackingReferenceBuilder"
            )

        if not np.isfinite(lookahead_time) or lookahead_time < 0.0:
            raise ValueError(
                "lookahead_time must be finite and nonnegative"
            )

        if (
            tracking_frame_kinematics is not None
            and not isinstance(
                tracking_frame_kinematics,
                TrackingFrameKinematics,
            )
        ):
            raise TypeError(
                "tracking_frame_kinematics must be "
                "TrackingFrameKinematics or None"
            )

        self._model = model
        self._reference_builder = reference_builder
        self._lookahead_time = float(lookahead_time)
        self._tracking_frame_kinematics = tracking_frame_kinematics

        self._pipeline = ControlPipeline(
            model=model,
            controller=controller,
            tracking_frame_kinematics=tracking_frame_kinematics,
        )

    def _select_target_position(
        self,
        prediction: PredictedTargetTrajectory,
        query_time: float,
    ) -> Array:
        """Select the first prediction sample at or after query time."""
        if not isinstance(prediction, PredictedTargetTrajectory):
            raise TypeError(
                "prediction must be PredictedTargetTrajectory"
            )

        if not np.isfinite(query_time):
            raise ValueError("query_time must be finite")

        if prediction.states.shape[1] < 3:
            raise ValueError(
                "Prediction must contain at least 3 position components"
            )

        index = int(
            np.searchsorted(
                prediction.timestamps,
                query_time,
                side="left",
            )
        )

        if index >= prediction.timestamps.size:
            raise ValueError(
                "Requested time exceeds prediction horizon"
            )

        return prediction.states[index, :3].copy()

    def compute(
        self,
        robot_state: RobotState,
        prediction: PredictedTargetTrajectory,
        transform_world_base: Array,
        joint_positions: Array,
        now: float,
    ) -> ControlOutput:
        """Compute a tracking command without actuator execution."""
        if not isinstance(robot_state, RobotState):
            raise TypeError("robot_state must be RobotState")

        if not isinstance(prediction, PredictedTargetTrajectory):
            raise TypeError(
                "prediction must be PredictedTargetTrajectory"
            )

        if robot_state.frame_id != prediction.frame_id:
            raise ValueError(
                "Robot state and prediction frames must match"
            )

        if not np.isfinite(now):
            raise ValueError("now must be finite")

        query_time = float(now) + self._lookahead_time

        target_position = self._select_target_position(
            prediction=prediction,
            query_time=query_time,
        )

        transform_world_base = np.asarray(
            transform_world_base,
            dtype=np.float64,
        )

        joint_positions = np.asarray(
            joint_positions,
            dtype=np.float64,
        )

        # 1. Compute the model end-effector pose.
        current_pose = WholeBodyKinematics.compute(
            robot_model=self._model,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
        )

        # 2. Map the pose to the tracking frame, if configured.
        if self._tracking_frame_kinematics is not None:
            current_pose = (
                self._tracking_frame_kinematics.transform_pose(
                    current_pose
                )
            )

        # 3. Use the controlled frame for reference generation.
        current_position = np.asarray(
            current_pose[:3, 3],
            dtype=np.float64,
        )

        bundle = self._reference_builder.build(
            current_position=current_position,
            target_position=target_position,
        )

        control_input = ControlInput(
            robot_state=robot_state,
            reference=bundle.reference,
            target_prediction=prediction,
        )

        # 4. Compute control with the same tracking-frame geometry.
        return self._pipeline.compute(
            control_input=control_input,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
            desired_end_effector_transform_world=bundle.desired_pose,
            desired_end_effector_twist_world=bundle.desired_twist,
        )
