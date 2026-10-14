"""Check task geometry with independent world-twist finite differences."""

import importlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def geometry():
    """Load the new geometry lazily so absent behavior is a clear assertion."""
    module = 'robot_models.task_space.distance_pointing'
    assert importlib.util.find_spec(module), 'Distance/pointing geometry is not implemented'
    return importlib.import_module(module).DistancePointingGeometry(
        radius=1., forward_axis_local=[0., 0., 1.], tangent_axis_local=[0., 1., 0.],
        minimum_distance=1e-4, antipodal_tolerance=1e-6)


def test_distance_sign_and_aligned_pointing():
    """Positive range error means the robot is too far; alignment has zero error."""
    task = geometry()
    error, jacobian, target_rate = task.evaluate(np.eye(4), [0., 0., 2.], np.eye(6), [0., 0., .3])
    assert np.allclose(error, [1., 0., 0.])
    assert jacobian[0, 2] == -1.
    assert target_rate[0] == .3
    assert np.allclose(jacobian[:, 5], 0.), 'Optical roll must be free when aligned'


@pytest.mark.parametrize('rotvec,target', [
    ([.4, -.3, .2], [1.3, -.4, .8]),
    ([.1, .2, -.2], [1e-2, .03, -2.]),
])
def test_twist_and_target_jacobians_match_finite_differences(rotvec, target):
    """Differentiate physical position and spatial rotation independently."""
    task = geometry()
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    pose[:3, 3] = [.1, -.2, .05]
    _, analytic, _ = task.evaluate(pose, target, np.eye(6), np.zeros(3))
    h = 1e-6
    numerical = np.zeros((3, 6))
    for i in range(6):
        plus, minus = pose.copy(), pose.copy()
        axis = np.eye(3)[i % 3]
        if i < 3:
            plus[:3, 3] += h * axis
            minus[:3, 3] -= h * axis
        else:
            plus[:3, :3] = Rotation.from_rotvec(h * axis).as_matrix() @ pose[:3, :3]
            minus[:3, :3] = Rotation.from_rotvec(-h * axis).as_matrix() @ pose[:3, :3]
        numerical[:, i] = (task.evaluate(plus, target, np.eye(6), np.zeros(3))[0] -
                           task.evaluate(minus, target, np.eye(6), np.zeros(3))[0]) / (2 * h)
    assert np.allclose(analytic, numerical, atol=2e-6)
    velocity = np.array([.3, -.2, .1])
    _, _, target_rate = task.evaluate(pose, target, np.eye(6), velocity)
    plus = task.evaluate(pose, np.array(target) + h * velocity, np.eye(6), velocity)[0]
    minus = task.evaluate(pose, np.array(target) - h * velocity, np.eye(6), velocity)[0]
    numerical_rate = (plus - minus) / (2 * h)
    assert np.allclose(target_rate, numerical_rate, atol=2e-6)


def test_antialignment_has_nonzero_finite_escape_error():
    """A camera looking directly away cannot be mistaken for alignment."""
    error, jacobian, _ = geometry().evaluate(np.eye(4), [0., 0., -2.], np.eye(6), [0., 0., 0.])
    assert np.isclose(np.linalg.norm(error[1:]), np.pi)
    assert np.all(np.isfinite(jacobian))
    command = -np.linalg.pinv(jacobian) @ error
    assert np.linalg.norm(command[3:]) > .1


def test_antipodal_escape_uses_an_available_rotation_direction():
    """A one-axis camera must escape even if the configured first tangent is blocked."""
    spatial = np.zeros((6, 1))
    spatial[4, 0] = 1.
    task = geometry()
    error, jacobian, _ = task.evaluate(np.eye(4), [0., 0., -2.], spatial, np.zeros(3))
    command = -np.linalg.pinv(jacobian) @ error
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_rotvec([0., .01 * command[0], 0.]).as_matrix()
    next_error = task.evaluate(pose, [0., 0., -2.], spatial, np.zeros(3))[0]
    assert np.linalg.norm(next_error[1:]) < np.linalg.norm(error[1:]) - .01


def test_coincident_target_is_rejected():
    """Direction at coincidence is undefined, so safe upstream fallback is required."""
    with pytest.raises(ValueError, match='distance'):
        geometry().evaluate(np.eye(4), [0., 0., 0.], np.eye(6), [0., 0., 0.])
