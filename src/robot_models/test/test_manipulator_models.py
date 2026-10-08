"""Tests for manipulator models."""

import numpy as np

from robot_models.manipulator.fixed_mount.model import (
    FixedMountModel,
)
from robot_models.manipulator.serial_arm.model import (
    SerialArmModel,
)


def make_serial_arm() -> SerialArmModel:
    """Create a serial-arm model for testing."""
    joint_axes = np.array([
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    return SerialArmModel(
        joint_axes=joint_axes,
        body_height=0.20,
        arm_base_height=0.08,
        link_1_length=0.40,
        link_2_length=0.36,
        wrist_length=0.12,
    )


def make_tracking_manipulator() -> FixedMountModel:
    """Create a serial arm with a fixed tracking-frame mount."""
    return FixedMountModel(
        parent=make_serial_arm(),
        translation=np.array([
            0.12,
            0.0,
            0.0,
        ]),
        rotation_rpy=np.zeros(3),
    )


def test_serial_arm_dof() -> None:
    """Verify manipulator degree-of-freedom count."""
    model = make_serial_arm()

    assert model.dof == 4


def test_serial_arm_forward_kinematics_shape() -> None:
    """Verify forward-kinematics output shape."""
    model = make_serial_arm()

    transform_base_end_effector = (
        model.forward_kinematics(
            np.zeros(model.dof)
        )
    )

    assert transform_base_end_effector.shape == (
        4,
        4,
    )

    assert np.allclose(
        transform_base_end_effector[3],
        np.array([
            0.0,
            0.0,
            0.0,
            1.0,
        ]),
    )


def test_serial_arm_zero_configuration_position() -> None:
    """Verify wrist position at zero configuration."""
    model = make_serial_arm()

    transform_base_wrist = (
        model.forward_kinematics(
            np.zeros(model.dof)
        )
    )

    expected_position = np.array([
        0.76,
        0.0,
        0.18,
    ])

    assert np.allclose(
        transform_base_wrist[:3, 3],
        expected_position,
    )


def test_serial_arm_jacobian_shape() -> None:
    """Verify manipulator Jacobian dimensions."""
    model = make_serial_arm()

    joint_positions = np.array([
        0.20,
        0.10,
        -0.15,
        0.05,
    ])

    jacobian = model.jacobian(
        joint_positions
    )

    assert jacobian.shape == (
        6,
        model.dof,
    )


def test_serial_arm_jacobian_position_finite_difference() -> None:
    """Verify serial-arm Jacobian using finite differences."""
    model = make_serial_arm()

    joint_positions = np.array([
        0.20,
        0.10,
        -0.15,
        0.05,
    ])

    jacobian = model.jacobian(
        joint_positions
    )

    epsilon = 1e-7

    numerical_jacobian = np.zeros(
        (
            3,
            model.dof,
        )
    )

    for index in range(model.dof):
        perturbation = np.zeros(model.dof)
        perturbation[index] = epsilon

        transform_plus = (
            model.forward_kinematics(
                joint_positions + perturbation
            )
        )

        transform_minus = (
            model.forward_kinematics(
                joint_positions - perturbation
            )
        )

        numerical_jacobian[:, index] = (
            transform_plus[:3, 3]
            - transform_minus[:3, 3]
        ) / (
            2.0 * epsilon
        )

    assert np.allclose(
        jacobian[:3, :],
        numerical_jacobian,
        atol=1e-6,
    )


def test_fixed_mount_preserves_dof() -> None:
    """Verify fixed mount preserves parent manipulator DOF."""
    model = make_tracking_manipulator()

    assert model.dof == 4


def test_fixed_mount_zero_configuration_position() -> None:
    """Verify tracking-frame position at zero configuration."""
    model = make_tracking_manipulator()

    transform_base_tracking = (
        model.forward_kinematics(
            np.zeros(model.dof)
        )
    )

    expected_position = np.array([
        0.88,
        0.0,
        0.18,
    ])

    assert np.allclose(
        transform_base_tracking[:3, 3],
        expected_position,
    )


def test_fixed_mount_jacobian_position_finite_difference() -> None:
    """Verify mounted-frame Jacobian using finite differences."""
    model = make_tracking_manipulator()

    joint_positions = np.array([
        0.20,
        0.10,
        -0.15,
        0.05,
    ])

    jacobian = model.jacobian(
        joint_positions
    )

    epsilon = 1e-7

    numerical_jacobian = np.zeros(
        (
            3,
            model.dof,
        )
    )

    for index in range(model.dof):
        perturbation = np.zeros(model.dof)
        perturbation[index] = epsilon

        transform_plus = (
            model.forward_kinematics(
                joint_positions + perturbation
            )
        )

        transform_minus = (
            model.forward_kinematics(
                joint_positions - perturbation
            )
        )

        numerical_jacobian[:, index] = (
            transform_plus[:3, 3]
            - transform_minus[:3, 3]
        ) / (
            2.0 * epsilon
        )

    assert np.allclose(
        jacobian[:3, :],
        numerical_jacobian,
        atol=1e-6,
    )
