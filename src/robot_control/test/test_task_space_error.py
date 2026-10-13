
"""Unit tests for controller-independent task-space errors."""

import numpy as np
import pytest

from robot_control.tracking.task_space_error import TaskSpaceError


def make_pose(position=None, rotation=None):
    """Construct a homogeneous transform for testing."""
    pose = np.eye(4)

    if position is not None:
        pose[:3, 3] = position

    if rotation is not None:
        pose[:3, :3] = rotation

    return pose


def rotation_z(angle):
    """Return a rotation matrix around the Z axis."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def test_zero_error():
    """Identical poses must produce zero error."""
    pose = make_pose(position=[1.0, 2.0, 3.0])

    error = TaskSpaceError.full_pose().compute(pose, pose)

    np.testing.assert_allclose(error, np.zeros(6), atol=1e-12)


def test_position_only():
    """Select position error without orientation components."""
    current = make_pose(position=[1.0, 2.0, 3.0])
    desired = make_pose(position=[4.0, 6.0, 8.0])

    error = TaskSpaceError.position_only().compute(
        current, desired
    )

    np.testing.assert_allclose(error, [3.0, 4.0, 5.0])


def test_orientation_error():
    """Verify the rotation-log error for a Z-axis rotation."""
    angle = np.pi / 4.0

    current = make_pose()
    desired = make_pose(rotation=rotation_z(angle))

    error = TaskSpaceError.full_pose().compute(
        current, desired
    )

    np.testing.assert_allclose(error[:3], np.zeros(3))
    np.testing.assert_allclose(
        error[3:],
        [0.0, 0.0, angle],
        atol=1e-12,
    )


def test_orientation_error_sign():
    """Reversing the desired rotation reverses the error."""
    angle = np.pi / 6.0

    current = make_pose()
    desired = make_pose(rotation=rotation_z(-angle))

    error = TaskSpaceError.full_pose().compute(
        current, desired
    )

    np.testing.assert_allclose(
        error[3:],
        [0.0, 0.0, -angle],
        atol=1e-12,
    )


def test_custom_task_selection():
    """Select X, Y and rotational Z error components."""
    selection = np.zeros((3, 6))
    selection[0, 0] = 1.0
    selection[1, 1] = 1.0
    selection[2, 5] = 1.0

    task = TaskSpaceError(selection)

    current = make_pose()
    desired = make_pose(
        position=[1.0, 2.0, 3.0],
        rotation=rotation_z(0.5),
    )

    error = task.compute(current, desired)

    np.testing.assert_allclose(
        error,
        [1.0, 2.0, 0.5],
        atol=1e-12,
    )


@pytest.mark.parametrize("dof", [2, 4, 6, 8])
def test_jacobian_selection(dof):
    """Support arbitrary generalized velocity dimensions."""
    task = TaskSpaceError.position_only()

    jacobian = np.arange(
        6 * dof, dtype=float
    ).reshape(6, dof)

    selected = task.select_jacobian(jacobian)

    assert selected.shape == (3, dof)

    np.testing.assert_array_equal(
        selected,
        jacobian[:3, :],
    )


def test_custom_linear_selection():
    """Allow a weighted linear projection of the task error."""
    selection = np.zeros((2, 6))
    selection[0, 0] = 1.0
    selection[0, 1] = 0.5
    selection[1, 5] = 2.0

    task = TaskSpaceError(selection)

    current = make_pose()
    desired = make_pose(
        position=[2.0, 4.0, 0.0],
        rotation=rotation_z(0.25),
    )

    error = task.compute(current, desired)

    np.testing.assert_allclose(
        error,
        [4.0, 0.5],
        atol=1e-12,
    )


def test_invalid_selection_shape():
    """Reject selection matrices without six columns."""
    with pytest.raises(ValueError):
        TaskSpaceError(np.eye(3))


def test_nonfinite_selection():
    """Reject NaN in the task selection matrix."""
    selection = np.eye(6)
    selection[0, 0] = np.nan

    with pytest.raises(ValueError):
        TaskSpaceError(selection)


def test_invalid_jacobian():
    """Reject Jacobians with invalid spatial dimensions."""
    task = TaskSpaceError.full_pose()

    with pytest.raises(ValueError):
        task.select_jacobian(np.zeros((3, 4)))


def test_nonfinite_pose():
    """Reject nonfinite pose data."""
    current = np.eye(4)
    current[0, 3] = np.nan

    with pytest.raises(ValueError):
        TaskSpaceError.full_pose().compute(
            current, np.eye(4)
        )


def test_selection_matrix_is_copied():
    """Protect the task selection from external modifications."""
    original = np.eye(6)
    task = TaskSpaceError(original)

    original[0, 0] = 100.0

    assert task.selection_matrix[0, 0] == 1.0

    retrieved = task.selection_matrix
    retrieved[0, 0] = 200.0

    assert task.selection_matrix[0, 0] == 1.0
