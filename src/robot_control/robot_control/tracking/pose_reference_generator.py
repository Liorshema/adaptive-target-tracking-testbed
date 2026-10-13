
"""Generate desired tracking poses using robot_models utilities."""

import numpy as np
from numpy.typing import NDArray
from robot_models.reference.look_at import LookAtReference
from robot_models.reference.pose_reference import PoseReference

Array = NDArray[np.float64]


def _validate_vector(value: Array, name: str) -> Array:
    """Return a validated three-dimensional vector."""
    vector = np.array(value, dtype=np.float64, copy=True)

    if vector.shape != (3,):
        raise ValueError(f"{name} must have shape (3,)")

    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain finite values")

    return vector


class PoseReferenceGenerator:
    """Generate a target-facing desired pose in the world frame.

    The tracking-frame axis conventions are injected externally.
    Orientation and pose mathematics are delegated to robot_models.
    """

    def __init__(
        self,
        forward_axis_local: Array,
        up_axis_local: Array,
        world_up: Array,
    ) -> None:
        """Configure local frame axes and the world-up direction."""
        self._forward_axis_local = _validate_vector(
            forward_axis_local,
            "forward_axis_local",
        )

        self._up_axis_local = _validate_vector(
            up_axis_local,
            "up_axis_local",
        )

        self._world_up = _validate_vector(
            world_up,
            "world_up",
        )

    def generate(
        self,
        desired_position: Array,
        target_position: Array,
    ) -> Array:
        """Return a homogeneous desired tracking pose.

        Args:
            desired_position: Desired tracking-frame position
                expressed in the world frame.
            target_position: Target position expressed in
                the same world frame.

        Returns:
            A 4x4 homogeneous transformation T_W_C_d.

        Raises:
            ValueError: If positions or frame axes are invalid.
        """
        desired = _validate_vector(
            desired_position,
            "desired_position",
        )

        target = _validate_vector(
            target_position,
            "target_position",
        )

        rotation = LookAtReference.compute(
            frame_position_world=desired,
            target_position_world=target,
            forward_axis_local=self._forward_axis_local,
            up_axis_local=self._up_axis_local,
            world_up=self._world_up,
        )

        return PoseReference.compute(
            desired_position_world=desired,
            desired_rotation_world_frame=rotation,
        )
