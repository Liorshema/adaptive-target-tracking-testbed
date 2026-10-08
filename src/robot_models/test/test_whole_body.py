"""Tests for whole-body robot composition."""

import numpy as np

from robot_models.base.differential_drive.kinematics import (
    DifferentialDriveKinematics,
)
from robot_models.common.rotations import rotation_z
from robot_models.common.transforms import make_transform
from robot_models.manipulator.fixed_mount.model import (
    FixedMountModel,
)
from robot_models.manipulator.serial_arm.model import (
    SerialArmModel,
)
from robot_models.whole_body.composer import WholeBodyComposer
from robot_models.whole_body.jacobian import WholeBodyJacobian
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_models.whole_body.twist import WholeBodyTwist


def make_robot() -> WholeBodyComposer:
    """Create a composed robot model for testing."""
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

    arm = SerialArmModel(
        joint_axes=joint_axes,
        body_height=0.20,
        arm_base_height=0.08,
        link_1_length=0.40,
        link_2_length=0.36,
        wrist_length=0.12,
    )

    manipulator = FixedMountModel(
        parent=arm,
        translation=np.array([
            0.12,
            0.0,
            0.0,
        ]),
        rotation_rpy=np.zeros(3),
    )

    return WholeBodyComposer(
        base=base,
        manipulator=manipulator,
    )


def test_whole_body_dimensions() -> None:
    """Verify composed generalized-velocity dimension."""
    robot = make_robot()

    assert robot.base.velocity_dimension == 2
    assert robot.manipulator.dof == 4
    assert robot.generalized_velocity_dimension == 6


def test_whole_body_forward_kinematics() -> None:
    """Verify world-frame tracking-frame kinematics."""
    robot = make_robot()

    transform_world_base = make_transform(
        rotation=rotation_z(0.30),
        position=np.array([
            1.0,
            2.0,
            0.0,
        ]),
    )

    joint_positions = np.zeros(
        robot.manipulator.dof
    )

    transform_world_tracking = (
        WholeBodyKinematics.compute(
            robot_model=robot,
            transform_world_base=transform_world_base,
            joint_positions=joint_positions,
        )
    )

    assert transform_world_tracking.shape == (
        4,
        4,
    )

    expected_position_world = (
        transform_world_base[:3, :3]
        @ np.array([
            0.88,
            0.0,
            0.18,
        ])
        + transform_world_base[:3, 3]
    )

    assert np.allclose(
        transform_world_tracking[:3, 3],
        expected_position_world,
    )


def test_whole_body_jacobian_shape() -> None:
    """Verify whole-body Jacobian dimensions."""
    robot = make_robot()

    transform_world_base = make_transform(
        rotation=rotation_z(0.30),
        position=np.array([
            1.0,
            2.0,
            0.0,
        ]),
    )

    joint_positions = np.array([
        0.20,
        0.10,
        -0.15,
        0.05,
    ])

    jacobian = WholeBodyJacobian.compute(
        robot_model=robot,
        transform_world_base=transform_world_base,
        joint_positions=joint_positions,
    )

    assert jacobian.shape == (
        6,
        robot.generalized_velocity_dimension,
    )


def test_whole_body_twist_mapping() -> None:
    """Verify generalized velocity maps to tracking-frame twist."""
    robot = make_robot()

    transform_world_base = make_transform(
        rotation=rotation_z(0.30),
        position=np.array([
            1.0,
            2.0,
            0.0,
        ]),
    )

    joint_positions = np.array([
        0.20,
        0.10,
        -0.15,
        0.05,
    ])

    generalized_velocity = np.array([
        0.30,
        0.10,
        0.10,
        -0.05,
        0.02,
        0.01,
    ])

    jacobian = WholeBodyJacobian.compute(
        robot_model=robot,
        transform_world_base=transform_world_base,
        joint_positions=joint_positions,
    )

    twist = WholeBodyTwist.compute(
        jacobian,
        generalized_velocity,
    )

    expected = (
        jacobian
        @ generalized_velocity
    )

    assert twist.shape == (6,)
    assert np.allclose(
        twist,
        expected,
    )
