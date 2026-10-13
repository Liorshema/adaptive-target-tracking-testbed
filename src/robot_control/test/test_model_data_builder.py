
"""Tests for controller-facing whole-body model data."""

import numpy as np
import pytest
from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.whole_body.composer import WholeBodyComposer

from robot_control.adapters.model_data_builder import ModelDataBuilder


class ExampleBase(BaseModel):
    """Minimal base with one generalized velocity."""

    @property
    def velocity_dimension(self):
        """Return base velocity dimension."""
        return 1

    def pose_derivative(self, pose, generalized_velocity):
        """Return a simple pose derivative."""
        return np.zeros_like(pose)

    def spatial_jacobian(self, point_base):
        """Map base velocity to translation along local x."""
        result = np.zeros((6, 1))
        result[0, 0] = 1.0
        return result


class ExampleManipulator(ManipulatorModel):
    """Two-DOF manipulator with known kinematic quantities."""

    @property
    def dof(self):
        """Return manipulator degrees of freedom."""
        return 2

    def forward_kinematics(self, joint_positions):
        """Return a fixed translation plus joint displacement."""
        result = np.eye(4)
        result[:3, 3] = [
            joint_positions[0],
            joint_positions[1],
            0.5,
        ]
        return result

    def jacobian(self, joint_positions):
        """Map joint velocities to local x and y translation."""
        result = np.zeros((6, 2))
        result[0, 0] = 1.0
        result[1, 1] = 1.0
        return result


@pytest.fixture
def model():
    """Construct a small interchangeable robot model."""
    return WholeBodyComposer(
        base=ExampleBase(),
        manipulator=ExampleManipulator(),
    )


@pytest.fixture
def inputs():
    """Create a consistent test configuration."""
    return {
        "transform_world_base": np.eye(4),
        "joint_positions": np.array([0.2, 0.3]),
        "generalized_velocity": np.array([1.0, 2.0, 3.0]),
        "desired_end_effector_transform_world": np.eye(4),
        "desired_end_effector_twist_world": np.zeros(6),
    }


def test_end_effector_pose(model, inputs):
    """Compute the expected world-frame end-effector pose."""
    data = ModelDataBuilder(model).build(**inputs)

    np.testing.assert_allclose(
        data.end_effector_transform_world[:3, 3],
        [0.2, 0.3, 0.5],
    )


def test_whole_body_jacobian(model, inputs):
    """Combine base and joint Jacobians in the correct order."""
    data = ModelDataBuilder(model).build(**inputs)

    expected = np.zeros((6, 3))
    expected[0, 0] = 1.0
    expected[0, 1] = 1.0
    expected[1, 2] = 1.0

    np.testing.assert_allclose(
        data.whole_body_jacobian_world,
        expected,
    )


def test_end_effector_twist(model, inputs):
    """Compute current twist from Jacobian and robot velocity."""
    data = ModelDataBuilder(model).build(**inputs)

    np.testing.assert_allclose(
        data.end_effector_twist_world,
        [3.0, 3.0, 0.0, 0.0, 0.0, 0.0],
    )


def test_world_frame_rotation(model, inputs):
    """Rotate local kinematics into world coordinates."""
    transform = np.eye(4)
    transform[:3, :3] = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    transform[:3, 3] = [4.0, 5.0, 6.0]

    inputs["transform_world_base"] = transform

    data = ModelDataBuilder(model).build(**inputs)

    np.testing.assert_allclose(
        data.end_effector_transform_world[:3, 3],
        [3.7, 5.2, 6.5],
    )

    np.testing.assert_allclose(
        data.end_effector_twist_world,
        [-3.0, 3.0, 0.0, 0.0, 0.0, 0.0],
    )


def test_desired_reference_is_preserved(model, inputs):
    """Pass the desired pose and twist to the controller unchanged."""
    desired_pose = np.eye(4)
    desired_pose[:3, 3] = [1.0, 2.0, 3.0]

    desired_twist = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])

    inputs["desired_end_effector_transform_world"] = desired_pose
    inputs["desired_end_effector_twist_world"] = desired_twist

    data = ModelDataBuilder(model).build(**inputs)

    np.testing.assert_allclose(
        data.desired_end_effector_transform_world,
        desired_pose,
    )
    np.testing.assert_allclose(
        data.desired_end_effector_twist_world,
        desired_twist,
    )


@pytest.mark.parametrize(
    ("field", "invalid_value"),
    [
        ("joint_positions", np.zeros(3)),
        ("generalized_velocity", np.zeros(2)),
        ("desired_end_effector_twist_world", np.zeros(3)),
        ("transform_world_base", np.eye(3)),
    ],
)
def test_invalid_dimensions(model, inputs, field, invalid_value):
    """Reject arrays inconsistent with model dimensions."""
    inputs[field] = invalid_value

    with pytest.raises(ValueError):
        ModelDataBuilder(model).build(**inputs)


def test_invalid_base_rotation(model, inputs):
    """Reject a non-rigid base transform."""
    transform = np.eye(4)
    transform[0, 0] = 2.0

    inputs["transform_world_base"] = transform

    with pytest.raises(ValueError, match="orthonormal"):
        ModelDataBuilder(model).build(**inputs)


def test_invalid_desired_rotation(model, inputs):
    """Reject a non-rigid desired end-effector transform."""
    transform = np.eye(4)
    transform[0, 0] = -1.0

    inputs["desired_end_effector_transform_world"] = transform

    with pytest.raises(ValueError, match="proper"):
        ModelDataBuilder(model).build(**inputs)
