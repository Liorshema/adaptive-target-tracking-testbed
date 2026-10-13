
"""Unit tests for desired tracking-pose generation."""

import numpy as np
import pytest

from robot_control.tracking.pose_reference_generator import (
    PoseReferenceGenerator,
)

FORWARD = np.array([0.0, 0.0, 1.0])
UP = np.array([0.0, -1.0, 0.0])
WORLD_UP = np.array([0.0, 0.0, 1.0])


def make_generator():
    """Create a tracking-pose generator with standard test axes."""
    return PoseReferenceGenerator(
        forward_axis_local=FORWARD,
        up_axis_local=UP,
        world_up=WORLD_UP,
    )


@pytest.mark.parametrize(
    "desired,target",
    [
        ([2.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ([0.0, 3.0, 0.0], [0.0, 0.0, 0.0]),
        ([0.0, 0.0, 2.0], [0.0, 0.0, 0.0]),
        ([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
        ([-2.0, 1.0, 4.0], [3.0, -1.0, 2.0]),
    ],
)
def test_pose_faces_target(desired, target):
    """Verify forward-axis alignment for different target directions."""
    desired = np.asarray(desired, dtype=float)
    target = np.asarray(target, dtype=float)

    T = make_generator().generate(desired, target)

    forward_world = T[:3, :3] @ FORWARD
    expected = target - desired
    expected /= np.linalg.norm(expected)

    np.testing.assert_allclose(
        forward_world,
        expected,
        atol=1e-12,
    )


def test_homogeneous_transform():
    """Verify the mathematical properties of SE(3)."""
    desired = np.array([2.0, 3.0, 1.0])
    target = np.zeros(3)

    T = make_generator().generate(desired, target)
    R = T[:3, :3]

    assert T.shape == (4, 4)

    np.testing.assert_allclose(
        T[:3, 3],
        desired,
    )

    np.testing.assert_allclose(
        T[3, :],
        [0.0, 0.0, 0.0, 1.0],
    )

    np.testing.assert_allclose(
        R.T @ R,
        np.eye(3),
        atol=1e-12,
    )

    np.testing.assert_allclose(
        np.linalg.det(R),
        1.0,
        atol=1e-12,
    )


def test_translation_invariance():
    """Translating the world preserves desired orientation."""
    generator = make_generator()

    desired = np.array([2.0, 0.0, 1.0])
    target = np.array([0.0, 1.0, 2.0])
    translation = np.array([5.0, -3.0, 4.0])

    T1 = generator.generate(desired, target)
    T2 = generator.generate(
        desired + translation,
        target + translation,
    )

    np.testing.assert_allclose(
        T2[:3, :3],
        T1[:3, :3],
        atol=1e-12,
    )

    np.testing.assert_allclose(
        T2[:3, 3],
        T1[:3, 3] + translation,
    )


def test_custom_local_forward_axis():
    """Support a non-default tracking-frame forward axis."""
    generator = PoseReferenceGenerator(
        forward_axis_local=np.array([1.0, 0.0, 0.0]),
        up_axis_local=np.array([0.0, 0.0, 1.0]),
        world_up=WORLD_UP,
    )

    desired = np.array([0.0, 0.0, 0.0])
    target = np.array([0.0, 1.0, 0.0])

    T = generator.generate(desired, target)

    np.testing.assert_allclose(
        T[:3, :3] @ np.array([1.0, 0.0, 0.0]),
        [0.0, 1.0, 0.0],
        atol=1e-12,
    )


def test_coincident_target_and_reference():
    """Reject undefined look-at directions."""
    generator = make_generator()

    with pytest.raises(ValueError):
        generator.generate(
            desired_position=np.ones(3),
            target_position=np.ones(3),
        )


def test_invalid_position_dimension():
    """Reject position vectors with invalid dimensions."""
    generator = make_generator()

    with pytest.raises(ValueError):
        generator.generate(
            desired_position=np.zeros(2),
            target_position=np.zeros(3),
        )


def test_nonfinite_position():
    """Reject NaN and infinite position values."""
    generator = make_generator()

    with pytest.raises(ValueError):
        generator.generate(
            desired_position=np.array([np.nan, 0.0, 0.0]),
            target_position=np.zeros(3),
        )


def test_invalid_local_axes():
    """Reject nonorthogonal local tracking-frame axes."""
    generator = PoseReferenceGenerator(
        forward_axis_local=np.array([0.0, 0.0, 1.0]),
        up_axis_local=np.array([0.0, 0.0, 1.0]),
        world_up=WORLD_UP,
    )

    with pytest.raises(ValueError):
        generator.generate(
            desired_position=np.array([1.0, 0.0, 0.0]),
            target_position=np.zeros(3),
        )
