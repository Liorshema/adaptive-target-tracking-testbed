
"""Adapt project robot description to control and safety configuration."""

from collections.abc import Mapping
from typing import Any

import numpy as np

from robot_models.common.rotations import rpy_to_rotation
from robot_models.common.transforms import make_transform

from robot_control.factory.project_components import (
    robot_configuration_from_description,
)


def _bounds(
    lower: Any,
    upper: Any,
    name: str,
) -> tuple[np.ndarray, np.ndarray]:
    """Validate compatible lower and upper bounds."""
    low = np.asarray(lower, dtype=np.float64)
    high = np.asarray(upper, dtype=np.float64)

    if (
        low.ndim != 1
        or low.size == 0
        or low.shape != high.shape
        or not np.all(np.isfinite(low))
        or not np.all(np.isfinite(high))
        or np.any(low > high)
    ):
        raise ValueError(f"Invalid {name} bounds")

    return low, high


def project_control_configuration(
    description: Mapping[str, Any],
    base_velocity_lower: Any,
    base_velocity_upper: Any,
) -> tuple[dict[str, Any], dict[str, Any], np.ndarray]:
    """Extract model, safety limits and tracking-frame transform.

    Args:
        description: Parsed robot description.
        base_velocity_lower: Lower limits for [v, omega].
        base_velocity_upper: Upper limits for [v, omega].

    Returns:
        Tuple containing:
            - Robot model configuration.
            - Safety configuration.
            - Fixed transform T_E_T.

    Notes:
        Joint ordering follows the robot description.

        Base velocity limits must be supplied explicitly and are
        not inferred from wheel geometry.

        The current SerialArmModel end-effector frame is expected
        to coincide with wrist_link.
    """
    if not isinstance(description, Mapping):
        raise TypeError("description must be a mapping")

    robot = description["robot"]

    model_config = robot_configuration_from_description(
        description
    )

    # --------------------------------------------------------
    # 1. Base velocity limits
    # --------------------------------------------------------

    base_lower, base_upper = _bounds(
        base_velocity_lower,
        base_velocity_upper,
        "base velocity",
    )

    if base_lower.shape != (2,):
        raise ValueError(
            "Base velocity bounds must have shape (2,)"
        )

    # --------------------------------------------------------
    # 2. Manipulator configuration
    # --------------------------------------------------------

    joints = list(
        robot["manipulator"]["joints"].values()
    )

    joint_count = len(joints)

    if joint_count == 0:
        raise ValueError(
            "Manipulator must contain joints"
        )

    joint_lower, joint_upper = _bounds(
        [joint["lower"] for joint in joints],
        [joint["upper"] for joint in joints],
        "joint position",
    )

    joint_speeds = np.asarray(
        [joint["velocity"] for joint in joints],
        dtype=np.float64,
    )

    if (
        not np.all(np.isfinite(joint_speeds))
        or np.any(joint_speeds <= 0.0)
    ):
        raise ValueError(
            "Joint velocity limits must be positive"
        )

    # --------------------------------------------------------
    # 3. Whole-body velocity bounds
    # --------------------------------------------------------

    # Generalized velocity ordering:
    # [v, omega, dq1, ..., dqn]

    velocity_lower = np.concatenate(
        (base_lower, -joint_speeds)
    )

    velocity_upper = np.concatenate(
        (base_upper, joint_speeds)
    )

    safety_config = {
        "velocity": {
            "lower": velocity_lower.tolist(),
            "upper": velocity_upper.tolist(),
        },
        "joint": {
            "lower": joint_lower.tolist(),
            "upper": joint_upper.tolist(),
            "velocity_indices": list(
                range(
                    base_lower.size,
                    base_lower.size + joint_count,
                )
            ),
        },
    }

    # --------------------------------------------------------
    # 4. Tracking-frame transform
    # --------------------------------------------------------

    frame = robot["tracking_frame"]

    if frame["parent"] != "wrist_link":
        raise ValueError(
            "Tracking-frame parent does not match the current "
            "SerialArmModel end-effector frame"
        )

    translation = np.asarray(
        frame["translation"],
        dtype=np.float64,
    )

    rpy = np.asarray(
        frame["rotation_rpy"],
        dtype=np.float64,
    )

    if (
        translation.shape != (3,)
        or rpy.shape != (3,)
        or not np.all(np.isfinite(translation))
        or not np.all(np.isfinite(rpy))
    ):
        raise ValueError(
            "Invalid tracking-frame pose"
        )

    # Reuse existing SO(3) and SE(3) utilities.
    transform_et = make_transform(
        rotation=rpy_to_rotation(rpy),
        position=translation,
    )

    return model_config, safety_config, transform_et
