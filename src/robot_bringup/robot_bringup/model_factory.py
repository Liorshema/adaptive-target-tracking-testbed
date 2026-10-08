"""Construct configured robot models from robot-description parameters."""

from robot_description.model_parameters import (
    load_robot_model_parameters,
    RobotModelParameters,
)

from robot_models.base.differential_drive.kinematics import (
    DifferentialDriveKinematics,
)
from robot_models.manipulator.serial_arm.model import SerialArmModel
from robot_models.manipulator.fixed_mount.model import FixedMountModel
from robot_models.whole_body.composer import WholeBodyComposer


def create_robot_model(
    parameters: RobotModelParameters | None = None,
) -> WholeBodyComposer:
    """
    Construct the configured whole-body robot model.

    If parameters are not supplied, load them from robot_description.
    """
    if parameters is None:
        parameters = load_robot_model_parameters()

    base = DifferentialDriveKinematics(
        wheel_radius=parameters.base.wheel_radius,
        track_width=parameters.base.track_width,
    )

    arm = SerialArmModel(
        joint_axes=parameters.manipulator.joint_axes,
        body_height=parameters.manipulator.body_height,
        arm_base_height=parameters.manipulator.arm_base_height,
        link_1_length=parameters.manipulator.link_1_length,
        link_2_length=parameters.manipulator.link_2_length,
        wrist_length=parameters.manipulator.wrist_length,
    )

    manipulator = FixedMountModel(
        parent=arm,
        translation=parameters.tracking_frame.translation,
        rotation_rpy=parameters.tracking_frame.rotation_rpy,
    )

    return WholeBodyComposer(
        base=base,
        manipulator=manipulator,
    )
