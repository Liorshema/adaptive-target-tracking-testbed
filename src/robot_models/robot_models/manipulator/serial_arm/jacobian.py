"""Geometric Jacobian for the 4-DOF serial manipulator."""

import numpy as np

from robot_models.common.rotations import axis_angle_rotation
from robot_models.common.transforms import make_transform


class ArmJacobian:
    """Compute the geometric Jacobian of the manipulator end-effector."""

    DOF = 4

    def __init__(
        self,
        joint_axes: np.ndarray,
        body_height: float,
        arm_base_height: float,
        link_1_length: float,
        link_2_length: float,
        wrist_length: float,
    ) -> None:
        joint_axes = np.asarray(
            joint_axes,
            dtype=float,
        )

        if joint_axes.shape != (self.DOF, 3):
            raise ValueError(
                'joint_axes must have shape (4, 3)'
            )

        axis_norms = np.linalg.norm(
            joint_axes,
            axis=1,
        )

        if np.any(np.isclose(axis_norms, 0.0)):
            raise ValueError(
                'joint axes must be non-zero'
            )

        self.joint_axes = (
            joint_axes
            / axis_norms[:, None]
        )

        self.body_height = float(body_height)
        self.arm_base_height = float(arm_base_height)
        self.link_1_length = float(link_1_length)
        self.link_2_length = float(link_2_length)
        self.wrist_length = float(wrist_length)

    def compute(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return the 6x4 geometric Jacobian in the robot base frame."""
        joint_positions = np.asarray(
            joint_positions,
            dtype=float,
        )

        if joint_positions.shape != (self.DOF,):
            raise ValueError(
                'joint_positions must have shape (4,)'
            )

        base_mount_height = (
            self.body_height / 2.0
            + self.arm_base_height / 2.0
        )

        shoulder_height = (
            self.arm_base_height / 2.0
        )

        joint_offsets = np.array(
            [
                [0.0, 0.0, base_mount_height],
                [0.0, 0.0, shoulder_height],
                [self.link_1_length, 0.0, 0.0],
                [self.link_2_length, 0.0, 0.0],
            ],
            dtype=float,
        )

        transform_base_to_current = np.eye(
            4,
            dtype=float,
        )

        joint_positions_base = []
        joint_axes_base = []

        for axis_local, angle, offset_local in zip(
            self.joint_axes,
            joint_positions,
            joint_offsets,
        ):
            rotation_base_to_current = (
                transform_base_to_current[:3, :3]
            )

            translation_base_to_current = (
                transform_base_to_current[:3, 3]
            )

            joint_position_base = (
                rotation_base_to_current
                @ offset_local
                + translation_base_to_current
            )

            joint_axis_base = (
                rotation_base_to_current
                @ axis_local
            )

            joint_positions_base.append(
                joint_position_base
            )

            joint_axes_base.append(
                joint_axis_base
            )

            joint_transform = make_transform(
                rotation=axis_angle_rotation(
                    axis_local,
                    angle,
                ),
                position=offset_local,
            )

            transform_base_to_current = (
                transform_base_to_current
                @ joint_transform
            )

        end_effector_position_base = (
            transform_base_to_current[:3, 3]
        )

        jacobian = np.zeros(
            (6, self.DOF),
            dtype=float,
        )

        for index, (
            joint_position_base,
            joint_axis_base,
        ) in enumerate(
            zip(
                joint_positions_base,
                joint_axes_base,
            )
        ):
            joint_to_end_effector_base = (
                end_effector_position_base
                - joint_position_base
            )

            jacobian[:3, index] = np.cross(
                joint_axis_base,
                joint_to_end_effector_base,
            )

            jacobian[3:, index] = (
                joint_axis_base
            )

        return jacobian
