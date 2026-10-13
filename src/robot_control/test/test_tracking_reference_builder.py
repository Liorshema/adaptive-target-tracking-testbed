
"""Tests for consistent tracking reference generation."""

import numpy as np
import pytest
from robot_models.reference.geometry.point_reference import PointReference
from robot_models.reference.geometry.sphere_reference import SphereReference

from robot_control.interfaces.control_data import TrackingReference
from robot_control.tracking.pose_reference_generator import (
    PoseReferenceGenerator,
)
from robot_control.tracking.radial_direction_policy import (
    RadialDirectionPolicy,
)
from robot_control.tracking.reference_selector import (
    FixedGeometryParameters,
    ReferenceSelector,
)
from robot_control.tracking.tracking_reference_builder import (
    TrackingReferenceBuilder,
    TrackingReferenceBundle,
)


@pytest.fixture
def pose_generator():
    """Create a camera with configurable optical axes."""
    return PoseReferenceGenerator(
        forward_axis_local=np.array([0.0, 0.0, 1.0]),
        up_axis_local=np.array([0.0, -1.0, 0.0]),
        world_up=np.array([0.0, 0.0, 1.0]),
    )


@pytest.fixture
def builder(pose_generator):
    """Create a spherical tracking reference builder."""
    selector = ReferenceSelector(
        geometry=SphereReference(),
        policy=RadialDirectionPolicy(radius=2.0),
    )

    return TrackingReferenceBuilder(
        selector=selector,
        pose_generator=pose_generator,
    )


@pytest.fixture
def inputs():
    """Provide positions without geometric singularities."""
    return {
        "current_position": np.array([4.0, 0.0, 0.0]),
        "target_position": np.array([1.0, 0.0, 0.0]),
    }


def test_spherical_reference_position(builder, inputs):
    """Place the desired camera on the selected tracking sphere."""
    result = builder.build(**inputs)

    np.testing.assert_allclose(
        result.reference.position,
        [3.0, 0.0, 0.0],
        atol=1e-10,
    )

    np.testing.assert_allclose(
        result.desired_pose[:3, 3],
        result.reference.position,
    )


def test_spherical_reference_radius(builder, inputs):
    """Maintain the configured distance from the target."""
    result = builder.build(**inputs)

    distance = np.linalg.norm(
        result.reference.position - inputs["target_position"]
    )

    assert distance == pytest.approx(2.0)


def test_camera_faces_target(builder, inputs):
    """Align the camera's local forward axis with the target."""
    result = builder.build(**inputs)

    forward_local = np.array([0.0, 0.0, 1.0])
    forward_world = result.desired_pose[:3, :3] @ forward_local

    direction = (
        inputs["target_position"] - result.reference.position
    )
    direction /= np.linalg.norm(direction)

    np.testing.assert_allclose(forward_world, direction, atol=1e-10)


def test_default_velocity_is_zero(builder, inputs):
    """Use zero feedforward velocity when none is supplied."""
    result = builder.build(**inputs)

    np.testing.assert_allclose(result.reference.velocity, np.zeros(3))
    np.testing.assert_allclose(result.desired_twist, np.zeros(6))


def test_feedforward_velocity_consistency(builder, inputs):
    """Use the same linear velocity in both reference contracts."""
    velocity = np.array([0.1, -0.2, 0.3])

    result = builder.build(
        **inputs,
        desired_linear_velocity=velocity,
    )

    np.testing.assert_allclose(result.reference.velocity, velocity)
    np.testing.assert_allclose(result.desired_twist[:3], velocity)
    np.testing.assert_allclose(result.desired_twist[3:], np.zeros(3))


def test_alternative_geometry(pose_generator, inputs):
    """Allow another geometry without changing builder logic."""
    selector = ReferenceSelector(
        geometry=PointReference(),
        policy=FixedGeometryParameters(
            np.array([0.0, 2.0, 0.0])
        ),
    )

    builder = TrackingReferenceBuilder(selector, pose_generator)
    result = builder.build(**inputs)

    np.testing.assert_allclose(
        result.reference.position,
        [1.0, 2.0, 0.0],
    )


def test_invalid_position_dimension(builder, inputs):
    """Reject input positions that are not three-dimensional."""
    inputs["current_position"] = np.zeros(2)

    with pytest.raises(ValueError, match="current_position"):
        builder.build(**inputs)


def test_invalid_feedforward_velocity(builder, inputs):
    """Reject nonfinite desired velocities."""
    with pytest.raises(ValueError, match="desired_linear_velocity"):
        builder.build(
            **inputs,
            desired_linear_velocity=np.array([np.nan, 0.0, 0.0]),
        )


def test_undefined_radial_direction(builder):
    """Reject a coincident target and current camera position."""
    with pytest.raises(ValueError, match="undefined"):
        builder.build(
            current_position=np.zeros(3),
            target_position=np.zeros(3),
        )


def test_bundle_rejects_inconsistent_translation():
    """Prevent two different desired positions in one bundle."""
    pose = np.eye(4)
    pose[:3, 3] = [2.0, 0.0, 0.0]

    reference = TrackingReference(
        position=np.array([1.0, 0.0, 0.0]),
    )

    with pytest.raises(ValueError, match="translation differs"):
        TrackingReferenceBundle(
            reference=reference,
            desired_pose=pose,
            desired_twist=np.zeros(6),
        )


def test_bundle_rejects_inconsistent_velocity():
    """Prevent conflicting feedforward linear velocities."""
    reference = TrackingReference(
        position=np.zeros(3),
        velocity=np.array([1.0, 0.0, 0.0]),
    )

    with pytest.raises(ValueError, match="linear velocity differs"):
        TrackingReferenceBundle(
            reference=reference,
            desired_pose=np.eye(4),
            desired_twist=np.zeros(6),
        )


def test_bundle_rejects_invalid_rotation():
    """Reject a reflection instead of a valid rotation."""
    pose = np.eye(4)
    pose[0, 0] = -1.0

    with pytest.raises(ValueError, match="proper"):
        TrackingReferenceBundle(
            reference=TrackingReference(position=np.zeros(3)),
            desired_pose=pose,
            desired_twist=np.zeros(6),
        )
