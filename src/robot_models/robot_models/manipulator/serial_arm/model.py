"""Serial-arm manipulator model."""

import numpy as np

from robot_models.interfaces.manipulator_model import (
    ManipulatorModel,
)
from robot_models.manipulator.serial_arm.forward_kinematics import (
    ArmForwardKinematics,
)
from robot_models.manipulator.serial_arm.jacobian import (
    ArmJacobian,
)


class SerialArmModel(ManipulatorModel):
    """Serial manipulator composed of arm FK and Jacobian models."""

    def __init__(
        self,
        joint_axes: np.ndarray,
        body_height: float,
        arm_base_height: float,
        link_1_length: float,
        link_2_length: float,
        wrist_length: float,
    ) -> None:
        self._forward_kinematics = ArmForwardKinematics(
            joint_axes=joint_axes,
            body_height=body_height,
            arm_base_height=arm_base_height,
            link_1_length=link_1_length,
            link_2_length=link_2_length,
            wrist_length=wrist_length,
        )

        self._jacobian = ArmJacobian(
            joint_axes=joint_axes,
            body_height=body_height,
            arm_base_height=arm_base_height,
            link_1_length=link_1_length,
            link_2_length=link_2_length,
            wrist_length=wrist_length,
        )

    @property
    def dof(self) -> int:
        """Return the manipulator degrees of freedom."""
        return self._forward_kinematics.DOF

    def forward_kinematics(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return the end-effector transform relative to the base."""
        return self._forward_kinematics.compute(
            joint_positions
        )

    def jacobian(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return the manipulator Jacobian in the base frame."""
        return self._jacobian.compute(
            joint_positions
        )
