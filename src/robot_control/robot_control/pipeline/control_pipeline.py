
"""Generic, ROS-independent robot control computation pipeline.

The pipeline evaluates robot model quantities and delegates control
computation to an interchangeable controller.

It does not validate data freshness, enforce actuator constraints,
or publish commands.
"""

from typing import Optional

import numpy as np
from numpy.typing import NDArray
from robot_models.interfaces.robot_model import RobotModel

from robot_control.adapters.model_data_builder import ModelDataBuilder
from robot_control.interfaces.control_data import (
    ControlInput,
    ControlOutput,
)
from robot_control.interfaces.controller import Controller
from robot_control.tracking.tracking_frame_kinematics import (
    TrackingFrameKinematics,
)

Array = NDArray[np.float64]


class ControlPipeline:
    """Connect robot model evaluation to a control algorithm.

    Robot-specific state conversion and reference generation are
    performed upstream. Command safety and execution are downstream.

    An optional tracking-frame transform changes the controlled frame
    without changing the robot model or controller interface.
    """

    def __init__(
        self,
        model: RobotModel,
        controller: Controller,
        tracking_frame_kinematics: Optional[
            TrackingFrameKinematics
        ] = None,
    ) -> None:
        """Configure the pipeline using injected components.

        Args:
            model: Robot model supplying whole-body kinematics.
            controller: Interchangeable control algorithm.
            tracking_frame_kinematics: Optional mapping from the
                model end-effector frame to the tracking frame.
        """
        if not isinstance(model, RobotModel):
            raise TypeError("model must implement RobotModel")

        if not isinstance(controller, Controller):
            raise TypeError("controller must implement Controller")

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
        self._controller = controller
        self._model_data_builder = ModelDataBuilder(
            model=model,
            tracking_frame_kinematics=tracking_frame_kinematics,
        )

    def compute(
        self,
        control_input: ControlInput,
        transform_world_base: Array,
        joint_positions: Array,
        desired_end_effector_transform_world: Array,
        desired_end_effector_twist_world: Array,
    ) -> ControlOutput:
        """Compute a generalized control command without executing it.

        Args:
            control_input: Current robot state and tracking reference.
            transform_world_base: Current homogeneous base transform.
            joint_positions: Current manipulator joint positions.
            desired_end_effector_transform_world: Desired controlled
                frame pose in world coordinates.
            desired_end_effector_twist_world: Desired controlled
                frame spatial twist in world coordinates.

        Returns:
            ControlOutput from the configured controller.

        Raises:
            TypeError: If the input or output contract is invalid.
            ValueError: If model dimensions or kinematic data are invalid.

        Notes:
            The existing 'end_effector' argument names are retained
            for backward compatibility. When tracking-frame kinematics
            is supplied, these desired quantities describe the
            tracking frame.
        """
        if not isinstance(control_input, ControlInput):
            raise TypeError("control_input must be ControlInput")

        model_data = self._model_data_builder.build(
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
            generalized_velocity=control_input.robot_state.velocity,
            desired_end_effector_transform_world=(
                desired_end_effector_transform_world
            ),
            desired_end_effector_twist_world=(
                desired_end_effector_twist_world
            ),
        )

        output = self._controller.compute(
            control_input=control_input,
            model=self._model,
            model_data=model_data,
        )

        if not isinstance(output, ControlOutput):
            raise TypeError(
                "Controller must return a ControlOutput"
            )

        if output.command.shape != (
            self._model.generalized_velocity_dimension,
        ):
            raise ValueError(
                "Control output dimension does not match robot model"
            )

        return output
