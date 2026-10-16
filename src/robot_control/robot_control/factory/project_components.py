
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

from robot_control.controllers.weighted_least_squares import WeightedLeastSquaresController
from robot_control.tracking.velocity_task import CartesianVelocityTask, DistancePointingTask
from robot_control.adapters.planar_kinematics_adapter import (
    PlanarKinematicsAdapter,
)
from robot_control.controllers.jacobian_controller import (
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

    registries['task'] = ComponentRegistry()
    registries['task'].register('cartesian', CartesianVelocityTask)
    registries['task'].register('distance_pointing', DistancePointingTask)
    registries['controller'].register('weighted_least_squares', WeightedLeastSquaresController)
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
    from robot_description.adapters.configuration import robot_component_configuration
    return robot_component_configuration(description)
