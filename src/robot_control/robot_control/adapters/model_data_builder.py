
"""Build controller-facing data from whole-body robot kinematics."""

from typing import Optional

import numpy as np
from numpy.typing import NDArray
from robot_models.interfaces.model_data import ModelData
from robot_models.interfaces.robot_model import RobotModel
from robot_models.whole_body.jacobian import WholeBodyJacobian
from robot_models.whole_body.kinematics import WholeBodyKinematics

from robot_control.tracking.tracking_frame_kinematics import (
    TrackingFrameKinematics,
)

Array = NDArray[np.float64]


def _array(value: Array, shape: tuple, name: str) -> Array:
    """Return a validated, independent numerical array."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")

    return result


def _transform(value: Array, name: str) -> Array:
    """Validate a rigid-body homogeneous transformation."""
    transform = _array(value, (4, 4), name)

    if not np.allclose(
        transform[3, :],
        [0.0, 0.0, 0.0, 1.0],
        atol=1e-8,
        rtol=0.0,
    ):
        raise ValueError(f"{name} has an invalid homogeneous row")

    rotation = transform[:3, :3]

    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=1e-6,
        rtol=0.0,
    ):
        raise ValueError(f"{name} rotation is not orthonormal")

    if not np.isclose(
        np.linalg.det(rotation),
        1.0,
        atol=1e-6,
        rtol=0.0,
    ):
        raise ValueError(f"{name} rotation must be proper")

    return transform


class ModelDataBuilder:
    """Evaluate whole-body kinematics for a controller.

    By default, the task frame coincides with the model's
    end-effector frame.

    An optional TrackingFrameKinematics maps the computed
    end-effector pose and Jacobian to another rigidly attached
    task frame.

    This class is independent of ROS, Gazebo, and robot-specific
    state representations.
    """

    def __init__(
        self,
        model: RobotModel,
        tracking_frame_kinematics: Optional[
            TrackingFrameKinematics
        ] = None,
    ) -> None:
        """Store the robot model and optional tracking-frame mapping."""
        if not isinstance(model, RobotModel):
            raise TypeError("model must implement RobotModel")

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
        self._tracking_frame_kinematics = tracking_frame_kinematics

    def build(
        self,
        transform_world_base: Array,
        joint_positions: Array,
        generalized_velocity: Array,
        desired_end_effector_transform_world: Array,
        desired_end_effector_twist_world: Array,
    ) -> ModelData:
        """Compute actual and desired task-space model quantities.

        Args:
            transform_world_base: Current 4x4 base pose in world.
            joint_positions: Manipulator joint configuration.
            generalized_velocity: Complete robot velocity vector.
            desired_end_effector_transform_world: Desired world-frame
                pose of the controlled task frame.
            desired_end_effector_twist_world: Desired world-frame
                spatial twist of the controlled task frame.

        Returns:
            ModelData with the current task-frame pose, twist,
            Jacobian, and desired quantities.

        Notes:
            Existing ModelData field names use 'end_effector'
            for the controlled frame. When tracking-frame
            kinematics is supplied, those fields refer to
            the tracking frame instead.
        """
        transform_world_base = _transform(
            transform_world_base,
            "transform_world_base",
        )

        joint_positions = _array(
            joint_positions,
            (self._model.manipulator.dof,),
            "joint_positions",
        )

        generalized_velocity = _array(
            generalized_velocity,
            (self._model.generalized_velocity_dimension,),
            "generalized_velocity",
        )

        desired_transform = _transform(
            desired_end_effector_transform_world,
            "desired_end_effector_transform_world",
        )

        desired_twist = _array(
            desired_end_effector_twist_world,
            (6,),
            "desired_end_effector_twist_world",
        )

        # 1. Evaluate original end-effector kinematics.
        current_transform = WholeBodyKinematics.compute(
            robot_model=self._model,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
        )

        jacobian = WholeBodyJacobian.compute(
            robot_model=self._model,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
        )

        current_transform = _transform(
            current_transform,
            "end_effector_transform_world",
        )

        jacobian = _array(
            jacobian,
            (6, self._model.generalized_velocity_dimension),
            "whole_body_jacobian_world",
        )

        # 2. Optionally shift the task frame.
        if self._tracking_frame_kinematics is not None:
            current_transform, jacobian = (
                self._tracking_frame_kinematics.compute(
                    transform_world_end_effector=current_transform,
                    jacobian_world_end_effector=jacobian,
                )
            )

        # 3. Compute twist at the controlled frame.
        current_twist = jacobian @ generalized_velocity

        return ModelData(
            end_effector_transform_world=current_transform,
            end_effector_twist_world=current_twist,
            desired_end_effector_transform_world=desired_transform,
            desired_end_effector_twist_world=desired_twist,
            whole_body_jacobian_world=jacobian,
        )
