"""Adapt authoritative robot descriptions to reusable configuration."""

from collections.abc import Mapping
from typing import Any

from robot_description.model_parameters import (
    robot_model_parameters_from_description,
)


def robot_component_configuration(
    description: Mapping[str, Any],
) -> dict[str, Any]:
    """
    Return base and manipulator specs from validated physical parameters.

    These specs contain normalized Python values and preserve the description's
    joint order. Tracking-frame mounting is a separate composition step, so its
    offset is not included in either component. The concrete ``serial_arm``
    implementation currently supports four joints; this adapter imposes no
    robot-specific dimension constraint.
    """
    parameters = robot_model_parameters_from_description(description)
    base = parameters.base
    manipulator = parameters.manipulator
    return {
        'base': {
            'type': 'differential_drive',
            'parameters': {
                'wheel_radius': base.wheel_radius,
                'track_width': base.track_width,
            },
        },
        'manipulator': {
            'type': 'serial_arm',
            'parameters': {
                'joint_axes': manipulator.joint_axes.tolist(),
                'body_height': manipulator.body_height,
                'arm_base_height': manipulator.arm_base_height,
                'link_1_length': manipulator.link_1_length,
                'link_2_length': manipulator.link_2_length,
                'wrist_length': manipulator.wrist_length,
            },
        },
    }


def robot_joint_configuration(
    description: Mapping[str, Any],
) -> dict[str, Any]:
    """Return joint names, count, and limits in description order."""
    parameters = robot_model_parameters_from_description(description)
    names = list(description['robot']['manipulator']['joints'])
    limits = parameters.joint_limits
    return {
        'names': names,
        'count': len(names),
        'lower': limits.lower.tolist(),
        'upper': limits.upper.tolist(),
        'velocity': limits.velocity.tolist(),
        'effort': limits.effort.tolist(),
        'damping': limits.damping.tolist(),
    }
