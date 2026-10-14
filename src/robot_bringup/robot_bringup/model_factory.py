"""Construct configured robot models from robot-description parameters."""

from dataclasses import asdict

from robot_description.model_parameters import (
    load_robot_model_parameters,
    RobotModelParameters,
)

from robot_models.base.differential_drive.kinematics import (
    DifferentialDriveKinematics,
)
from robot_models.manipulator.serial_arm.model import SerialArmModel
from robot_models.whole_body.composer import WholeBodyComposer
from robot_models.whole_body.factory import build_configured_model


def _create_component(category, specification):
    """Select Bringup's existing model implementations for shared composition."""
    implementations = {
        'base': DifferentialDriveKinematics,
        'manipulator': SerialArmModel,
    }
    return implementations[category](**specification['parameters'])


def create_robot_model(
    parameters: RobotModelParameters | None = None,
) -> WholeBodyComposer:
    """
    Construct the configured whole-body robot model.

    If parameters are not supplied, load them from robot_description.
    """
    if parameters is None:
        parameters = load_robot_model_parameters()

    configuration = {
        'base': {'type': 'differential_drive', 'parameters': asdict(parameters.base)},
        'manipulator': {'type': 'serial_arm', 'parameters': asdict(parameters.manipulator)},
        'tracking_frame': asdict(parameters.tracking_frame),
    }
    return build_configured_model(configuration, _create_component)
