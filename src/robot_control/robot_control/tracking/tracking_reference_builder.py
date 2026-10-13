
"""Build consistent tracking references for robot control.

This module combines geometry-based position selection with
target-facing pose generation.

It is independent of ROS, Gazebo, and robot-specific kinematics.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from robot_control.interfaces.control_data import TrackingReference
from robot_control.tracking.pose_reference_generator import (
    PoseReferenceGenerator,
)
from robot_control.tracking.reference_selector import ReferenceSelector

Array = NDArray[np.float64]


def _vector3(value: Array, name: str) -> Array:
    """Return an independent finite three-dimensional vector."""
    result = np.array(value, dtype=np.float64, copy=True)

    if result.shape != (3,):
        raise ValueError(f"{name} must have shape (3,)")

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")

    return result


@dataclass(frozen=True)
class TrackingReferenceBundle:
    """Consistent task-space references for a control computation."""

    reference: TrackingReference
    desired_pose: Array
    desired_twist: Array

    def __post_init__(self) -> None:
        """Validate the reference bundle and its consistency."""
        if not isinstance(self.reference, TrackingReference):
            raise TypeError("reference must be TrackingReference")

        pose = np.array(self.desired_pose, dtype=np.float64, copy=True)
        twist = np.array(self.desired_twist, dtype=np.float64, copy=True)

        if pose.shape != (4, 4):
            raise ValueError("desired_pose must have shape (4, 4)")

        if twist.shape != (6,):
            raise ValueError("desired_twist must have shape (6,)")

        if not np.all(np.isfinite(pose)):
            raise ValueError("desired_pose must be finite")

        if not np.all(np.isfinite(twist)):
            raise ValueError("desired_twist must be finite")

        if not np.allclose(
            pose[3, :],
            [0.0, 0.0, 0.0, 1.0],
            atol=1e-8,
            rtol=0.0,
        ):
            raise ValueError("desired_pose has an invalid bottom row")

        rotation = pose[:3, :3]

        if not np.allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1e-6,
            rtol=0.0,
        ):
            raise ValueError("desired_pose rotation is not orthonormal")

        if not np.isclose(
            np.linalg.det(rotation),
            1.0,
            atol=1e-6,
            rtol=0.0,
        ):
            raise ValueError("desired_pose rotation must be proper")

        if self.reference.position.shape != (3,):
            raise ValueError("reference position must have shape (3,)")

        if not np.allclose(
            pose[:3, 3],
            self.reference.position,
            atol=1e-10,
            rtol=0.0,
        ):
            raise ValueError(
                "desired_pose translation differs from reference position"
            )

        if self.reference.velocity is not None:
            if not np.allclose(
                twist[:3],
                self.reference.velocity,
                atol=1e-10,
                rtol=0.0,
            ):
                raise ValueError(
                    "desired_twist linear velocity differs from reference"
                )

        object.__setattr__(self, "desired_pose", pose)
        object.__setattr__(self, "desired_twist", twist)


class TrackingReferenceBuilder:
    """Combine interchangeable reference selection and pose generation."""

    def __init__(
        self,
        selector: ReferenceSelector,
        pose_generator: PoseReferenceGenerator,
    ) -> None:
        """Inject geometry selection and desired-pose generation."""
        if not isinstance(selector, ReferenceSelector):
            raise TypeError("selector must be ReferenceSelector")

        if not isinstance(pose_generator, PoseReferenceGenerator):
            raise TypeError(
                "pose_generator must be PoseReferenceGenerator"
            )

        self._selector = selector
        self._pose_generator = pose_generator

    def build(
        self,
        current_position: Array,
        target_position: Array,
        desired_linear_velocity: Optional[Array] = None,
    ) -> TrackingReferenceBundle:
        """Generate consistent position, pose, and twist references.

        Args:
            current_position: Current tracking-frame position in world.
            target_position: Selected target position in world.
            desired_linear_velocity: Optional desired world-frame linear
                velocity. Defaults to zero.

        Returns:
            Consistent references for the control pipeline.

        Notes:
            Angular feedforward velocity is zero at this stage.
            A dedicated trajectory module may provide it later.
        """
        current = _vector3(current_position, "current_position")
        target = _vector3(target_position, "target_position")

        if desired_linear_velocity is None:
            linear_velocity = np.zeros(3, dtype=np.float64)
        else:
            linear_velocity = _vector3(
                desired_linear_velocity,
                "desired_linear_velocity",
            )

        selected_reference = self._selector.select(
            current_position=current,
            target_position=target,
        )

        desired_position = _vector3(
            selected_reference.position,
            "selected_reference.position",
        )

        desired_pose = self._pose_generator.generate(
            desired_position=desired_position,
            target_position=target,
        )

        desired_twist = np.zeros(6, dtype=np.float64)
        desired_twist[:3] = linear_velocity

        reference = TrackingReference(
            position=desired_position,
            velocity=linear_velocity,
        )

        return TrackingReferenceBundle(
            reference=reference,
            desired_pose=desired_pose,
            desired_twist=desired_twist,
        )
