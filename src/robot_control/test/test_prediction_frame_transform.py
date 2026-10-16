"""Fixed inertial-frame conversion must rotate velocity and translate position only."""

import numpy as np
import pytest
from robot_control.interfaces.control_data import PredictedTargetTrajectory
from robot_control.adapters.prediction_frame_transform import transform_target_trajectory


def test_rigid_frame_conversion_preserves_times_velocity_norm_and_source():
    states = np.array([[1., 2., 3., .4, .5, .6], [2., 3., 4., .5, .6, .7]])
    prediction = PredictedTargetTrajectory(states.copy(), np.array([1., 2.]), 'world')
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    result = transform_target_trajectory(prediction, rotation, np.array([10., 20., 30.]), 'odom')
    np.testing.assert_allclose(result.states[0], [8., 21., 33., -.5, .4, .6])
    np.testing.assert_array_equal(prediction.states, states)
    np.testing.assert_array_equal(result.timestamps, prediction.timestamps)
    assert result.frame_id == 'odom'
    np.testing.assert_allclose(np.linalg.norm(result.states[:, 3:], axis=1), np.linalg.norm(states[:, 3:], axis=1))


def test_frame_conversion_rejects_a_different_state_layout():
    prediction = PredictedTargetTrajectory(np.zeros((2, 4)), np.array([1., 2.]), 'world')
    with pytest.raises(ValueError, match='Frame conversion'):
        transform_target_trajectory(prediction, np.eye(3), np.zeros(3), 'odom')
