
"""Tests for planar robot-state to kinematic-state conversion."""

import numpy as np
import pytest

from robot_control.adapters.planar_kinematics_adapter import (
    KinematicState,
    PlanarKinematicsAdapter,
)
from robot_control.interfaces.control_data import RobotState


def make_state(x=1.0, y=2.0, yaw=0.0, joints=None):
    """Create a consistent planar-base robot state."""
    if joints is None:
        joints = np.array([0.2, -0.3])

    joints = np.asarray(joints, dtype=np.float64)

    return RobotState(
        configuration=np.concatenate(([x, y, yaw], joints)),
        velocity=np.zeros(2 + joints.size),
        timestamp=10.0,
        frame_id="odom",
    )


def test_base_translation():
    """Map planar position into world-frame base translation."""
    adapter = PlanarKinematicsAdapter(joint_count=2)
    result = adapter.convert(make_state())

    np.testing.assert_allclose(
        result.transform_world_base[:3, 3],
        [1.0, 2.0, 0.0],
    )


def test_zero_yaw_rotation():
    """Use identity orientation for a zero-heading base."""
    result = PlanarKinematicsAdapter(2).convert(make_state())

    np.testing.assert_allclose(
        result.transform_world_base[:3, :3],
        np.eye(3),
        atol=1e-12,
    )


def test_ninety_degree_rotation():
    """Rotate the base frame by ninety degrees around world z."""
    state = make_state(yaw=np.pi / 2.0)
    result = PlanarKinematicsAdapter(2).convert(state)

    expected = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])

    np.testing.assert_allclose(
        result.transform_world_base[:3, :3],
        expected,
        atol=1e-12,
    )


def test_negative_yaw_rotation():
    """Support clockwise rotations without changing conventions."""
    state = make_state(yaw=-np.pi / 2.0)
    result = PlanarKinematicsAdapter(2).convert(state)

    expected = np.array([
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])

    np.testing.assert_allclose(
        result.transform_world_base[:3, :3],
        expected,
        atol=1e-12,
    )


def test_joint_positions():
    """Preserve the manipulator configuration in the correct order."""
    state = make_state(joints=[0.4, -0.1, 0.8])

    result = PlanarKinematicsAdapter(3).convert(state)

    np.testing.assert_allclose(
        result.joint_positions,
        [0.4, -0.1, 0.8],
    )


@pytest.mark.parametrize("joint_count", [0, 1, 2, 4, 7])
def test_variable_joint_count(joint_count):
    """Support different manipulator sizes without fixed dimensions."""
    joints = np.linspace(0.1, 0.5, joint_count)
    state = make_state(joints=joints)

    result = PlanarKinematicsAdapter(joint_count).convert(state)

    assert result.joint_positions.shape == (joint_count,)
    np.testing.assert_allclose(result.joint_positions, joints)


def test_reject_configuration_dimension():
    """Reject mismatched configuration dimensions."""
    state = make_state(joints=[0.1, 0.2])

    with pytest.raises(ValueError, match="configuration"):
        PlanarKinematicsAdapter(3).convert(state)


def test_reject_velocity_dimension():
    """Reject mismatched generalized velocity dimensions."""
    state = RobotState(
        configuration=np.array([0.0, 0.0, 0.0, 0.1, 0.2]),
        velocity=np.zeros(3),
        timestamp=10.0,
        frame_id="odom",
    )

    with pytest.raises(ValueError, match="velocity"):
        PlanarKinematicsAdapter(2).convert(state)


@pytest.mark.parametrize("joint_count", [-1, 1.5, True, None])
def test_reject_invalid_joint_count(joint_count):
    """Require a valid nonnegative integer joint count."""
    with pytest.raises(ValueError, match="joint_count"):
        PlanarKinematicsAdapter(joint_count)


def test_reject_non_robot_state():
    """Require input using the generic RobotState contract."""
    with pytest.raises(TypeError, match="RobotState"):
        PlanarKinematicsAdapter(2).convert(None)


def test_homogeneous_last_row():
    """Return a proper rigid-body homogeneous transformation."""
    result = PlanarKinematicsAdapter(2).convert(
        make_state(yaw=0.7)
    )

    np.testing.assert_allclose(
        result.transform_world_base[3, :],
        [0.0, 0.0, 0.0, 1.0],
    )

    rotation = result.transform_world_base[:3, :3]

    np.testing.assert_allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=1e-12,
    )

    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_kinematic_state_copies_arrays():
    """Ensure the result does not share writable input arrays."""
    transform = np.eye(4)
    joints = np.array([0.1, 0.2])

    result = KinematicState(
        transform_world_base=transform,
        joint_positions=joints,
    )

    transform[0, 3] = 100.0
    joints[0] = 99.0

    assert result.transform_world_base[0, 3] == pytest.approx(0.0)
    assert result.joint_positions[0] == pytest.approx(0.1)


def test_reject_invalid_transform():
    """Reject non-rigid transforms in the kinematic-state contract."""
    transform = np.eye(4)
    transform[0, 0] = 2.0

    with pytest.raises(ValueError, match="orthonormal"):
        KinematicState(
            transform_world_base=transform,
            joint_positions=np.zeros(2),
        )


def test_reject_nonfinite_joints():
    """Reject nonfinite manipulator configurations."""
    with pytest.raises(ValueError, match="finite"):
        KinematicState(
            transform_world_base=np.eye(4),
            joint_positions=np.array([0.1, np.nan]),
        )
