
"""Register project components and adapt robot-description geometry.

Physical robot parameters are read from the existing robot description.
No ROS, Gazebo, or actuator dependencies are introduced.
"""

from collections.abc import Mapping
from typing import Any

from robot_models.base.differential_drive.kinematics import (
    DifferentialDriveKinematics,
)
from robot_models.manipulator.serial_arm.model import SerialArmModel
from robot_models.reference.geometry.sphere_reference import (
    SphereReference,
)

from robot_control.adapters.planar_kinematics_adapter import (
    PlanarKinematicsAdapter,
)
from robot_control.baseline.jacobian_controller import (
    JacobianController,
)
from robot_control.factory.component_registry import ComponentRegistry
from robot_control.factory.control_factory import ControlFactory
from robot_control.tracking.radial_direction_policy import (
    RadialDirectionPolicy,
)


def create_project_registries() -> dict[str, ComponentRegistry]:
    """Register implementations currently available in the project."""
    registries = {
        name: ComponentRegistry()
        for name in ControlFactory.REGISTRY_NAMES
    }

    registries["base"].register(
        "differential_drive",
        DifferentialDriveKinematics,
    )
    registries["manipulator"].register(
        "serial_arm",
        SerialArmModel,
    )
    registries["geometry"].register(
        "sphere",
        SphereReference,
    )
    registries["policy"].register(
        "radial",
        RadialDirectionPolicy,
    )
    registries["controller"].register(
        "jacobian",
        JacobianController,
    )
    registries["adapter"].register(
        "planar",
        PlanarKinematicsAdapter,
    )

    return registries


def robot_configuration_from_description(
    description: Mapping[str, Any],
) -> dict[str, Any]:
    """Extract model parameters from robot_description/robot.yaml.

    Args:
        description: Parsed YAML document with a top-level robot key.

    Returns:
        Robot specifications accepted by ControlConfigAdapter.

    Notes:
        Joint ordering follows the configured description order.
        The runtime estimator must use the same order.
    """
    if not isinstance(description, Mapping):
        raise TypeError("Robot description must be a mapping")

    robot = description["robot"]
    body = robot["body"]
    wheels = robot["wheels"]
    manipulator = robot["manipulator"]

    joints = manipulator["joints"]
    joint_axes = [
        joint["axis"]
        for joint in joints.values()
    ]

    if len(joint_axes) != 4:
        raise ValueError(
            "SerialArmModel requires exactly four joints"
        )

    return {
        "base": {
            "type": "differential_drive",
            "parameters": {
                "wheel_radius": float(wheels["radius"]),
                "track_width": 2.0 * abs(
                    float(wheels["y_offset"])
                ),
            },
        },
        "manipulator": {
            "type": "serial_arm",
            "parameters": {
                "joint_axes": joint_axes,
                "body_height": float(body["height"]),
                "arm_base_height": float(
                    manipulator["base"]["height"]
                ),
                "link_1_length": float(
                    manipulator["link1"]["length"]
                ),
                "link_2_length": float(
                    manipulator["link2"]["length"]
                ),
                "wrist_length": float(
                    manipulator["wrist"]["length"]
                ),
            },
        },
    }
