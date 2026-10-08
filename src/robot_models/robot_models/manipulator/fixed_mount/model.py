"""Fixed transform attached to a manipulator end-effector."""

import numpy as np

from robot_models.common.rotations import (
    rpy_to_rotation,
)
from robot_models.common.transforms import (
    make_transform,
)
from robot_models.interfaces.manipulator_model import (
    ManipulatorModel,
)


class FixedMountModel(ManipulatorModel):
    """Attach a fixed frame to another manipulator model."""

    def __init__(
        self,
        parent: ManipulatorModel,
        translation: np.ndarray,
        rotation_rpy: np.ndarray,
    ) -> None:
        translation = np.asarray(
            translation,
            dtype=float,
        )

        rotation_rpy = np.asarray(
            rotation_rpy,
            dtype=float,
        )

        if translation.shape != (3,):
            raise ValueError(
                'translation must have shape (3,)'
            )

        if rotation_rpy.shape != (3,):
            raise ValueError(
                'rotation_rpy must have shape (3,)'
            )

        self._parent = parent

        self._transform_parent_mount = make_transform(
            rotation=rpy_to_rotation(
                rotation_rpy
            ),
            position=translation,
        )

    @property
    def dof(self) -> int:
        """Return the parent manipulator degrees of freedom."""
        return self._parent.dof

    def forward_kinematics(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return mounted-frame transform relative to the base."""
        transform_base_parent = (
            self._parent.forward_kinematics(
                joint_positions
            )
        )

        return (
            transform_base_parent
            @ self._transform_parent_mount
        )

    def jacobian(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return mounted-frame geometric Jacobian in the base frame."""
        transform_base_parent = (
            self._parent.forward_kinematics(
                joint_positions
            )
        )

        parent_jacobian = (
            self._parent.jacobian(
                joint_positions
            )
        )

        rotation_base_parent = (
            transform_base_parent[:3, :3]
        )

        offset_parent = (
            self._transform_parent_mount[:3, 3]
        )

        offset_base = (
            rotation_base_parent
            @ offset_parent
        )

        jacobian = parent_jacobian.copy()

        angular_jacobian = (
            parent_jacobian[3:, :]
        )

        jacobian[:3, :] = (
            parent_jacobian[:3, :]
            + np.column_stack(
                [
                    np.cross(
                        angular_jacobian[:, index],
                        offset_base,
                    )
                    for index in range(
                        self.dof
                    )
                ]
            )
        )

        return jacobian
