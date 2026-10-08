"""End-to-end tests for the robot-model pipeline."""

import numpy as np

from robot_models.base.differential_drive.kinematics import (
    DifferentialDriveKinematics,
)
from robot_models.camera.fov import CameraFOV
from robot_models.camera.projection import CameraProjection
from robot_models.common.rotations import rotation_z
from robot_models.common.transforms import make_transform
from robot_models.constraints.visibility import (
    VisibilityConstraint,
)
from robot_models.interfaces.model_data import ModelData
from robot_models.manipulator.serial_arm.model import (
    SerialArmModel,
)
from robot_models.reference.look_at import LookAtReference
from robot_models.reference.pose_reference import PoseReference
from robot_models.task_space.pose_error import PoseError
from robot_models.whole_body.composer import WholeBodyComposer
from robot_models.whole_body.jacobian import WholeBodyJacobian
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_models.whole_body.twist import WholeBodyTwist


def test_model_pipeline() -> None:
    """Verify the complete composed robot-model pipeline."""
    base = DifferentialDriveKinematics(
        wheel_radius=0.15,
        track_width=0.60,
    )

    joint_axes = np.array([
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    manipulator = SerialArmModel(
        joint_axes=joint_axes,
        body_height=0.20,
        arm_base_height=0.08,
        link_1_length=0.40,
        link_2_length=0.36,
        wrist_length=0.12,
    )

    robot = WholeBodyComposer(
        base=base,
        manipulator=manipulator,
    )

    joint_positions = np.array([
        0.20,
        0.10,
        -0.15,
        0.05,
    ])

    joint_velocities = np.array([
        0.10,
        -0.05,
        0.02,
        0.01,
    ])

    base_velocity = np.array([
        0.30,
        0.10,
    ])

    transform_world_base = make_transform(
        rotation=rotation_z(0.30),
        position=np.array([
            1.0,
            2.0,
            0.0,
        ]),
    )

    transform_world_end_effector = (
        WholeBodyKinematics.compute(
            robot_model=robot,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
        )
    )

    whole_body_jacobian = (
        WholeBodyJacobian.compute(
            robot_model=robot,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
        )
    )

    generalized_velocity = np.concatenate(
        (
            base_velocity,
            joint_velocities,
        )
    )

    end_effector_twist = WholeBodyTwist.compute(
        whole_body_jacobian,
        generalized_velocity,
    )

    target_position_world = np.array([
        3.0,
        2.5,
        1.0,
    ])

    desired_rotation_world_end_effector = (
        LookAtReference.compute(
            frame_position_world=(
                transform_world_end_effector[:3, 3]
            ),
            target_position_world=target_position_world,
        )
    )

    desired_transform_world_end_effector = (
        PoseReference.compute(
            desired_position_world=(
                transform_world_end_effector[:3, 3]
            ),
            desired_rotation_world_frame=(
                desired_rotation_world_end_effector
            ),
        )
    )

    pose_error = PoseError.compute(
        transform_world_end_effector,
        desired_transform_world_end_effector,
    )

    projection = CameraProjection(
        fx=600.0,
        fy=600.0,
        cx=320.0,
        cy=240.0,
    )

    fov = CameraFOV(
        image_width=640,
        image_height=480,
    )

    visibility = VisibilityConstraint(
        projection=projection,
        fov=fov,
    )

    visibility_residual = visibility.residual(
        desired_transform_world_end_effector,
        target_position_world,
    )

    model_data = ModelData(
        end_effector_transform_world=(
            transform_world_end_effector
        ),
        end_effector_twist_world=end_effector_twist,
        desired_end_effector_transform_world=(
            desired_transform_world_end_effector
        ),
        desired_end_effector_twist_world=np.zeros(6),
        whole_body_jacobian_world=whole_body_jacobian,
        visibility_constraint_residual=visibility_residual,
    )

    assert robot.base.velocity_dimension == 2
    assert robot.manipulator.dof == 4
    assert robot.generalized_velocity_dimension == 6

    assert transform_world_end_effector.shape == (4, 4)
    assert whole_body_jacobian.shape == (6, 6)
    assert generalized_velocity.shape == (6,)
    assert end_effector_twist.shape == (6,)
    assert pose_error.shape == (6,)
    assert visibility_residual.shape == (4,)

    assert model_data.whole_body_jacobian_world.shape == (
        6,
        6,
    )

    assert np.all(
        visibility_residual >= 0.0
    )
