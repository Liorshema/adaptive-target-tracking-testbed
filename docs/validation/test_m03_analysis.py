"""Protect state-time comparison and frame alignment from analysis bias."""

import numpy as np
from scipy.spatial.transform import Rotation

from m03_analysis import estimate_errors, planar
from m03_replay import run


def test_initial_world_to_odom_alignment_and_yaw_unwrap():
    """A translated/rotated spawn must not count as localization error."""
    spawn = np.array([2., 3., .25, 0., 0., np.pi/2])
    samples = [[0., [2., 4., .25], Rotation.from_euler('z', 3.1).as_quat()],
               [1., [2., 5., .25], Rotation.from_euler('z', -3.1).as_quat()]]
    _, poses = planar(samples, spawn)
    np.testing.assert_allclose(poses[:, :2], [[1., 0.], [2., 0.]], atol=1e-14)
    assert 0 < poses[1, 2]-poses[0, 2] < .1


def test_errors_use_bracketed_source_stamps_not_receipt_time():
    """Delayed publication must not create an artificial moving-state error."""
    poses = [[t, [t, 0., 0.], [0., 0., 0., 1.]] for t in [0., 2.]]
    streams = dict(physical_base=poses, odometry=poses,
                   joints=[[0., [0.]], [2., [2.]]])
    snapshot = dict(robot=dict(pose=[0.]*6), description=dict(
        manipulator=dict(joints=dict(arbitrary=dict(lower=0., upper=3.)))))
    estimates = [[-1., [999.]*7], [1., [1., 0., 0., 0., 0., 1., 0.]], [3., [999.]*7]]
    result = estimate_errors(estimates, streams, snapshot, -2., 4.)
    assert result['samples'] == 1
    assert result['mean_base_position_error_m'] == 0
    assert result['joint_rmse_rad'] == [0.]


def test_replay_uses_effective_deployment_measurement_covariance():
    """Recorded ROS parameter overrides must also govern offline replay."""
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[2]
    cfg = yaml.safe_load((root / 'src/robot_estimation/config/robot_estimator.yaml').read_text())
    cfg = cfg['estimation_node']['ros__parameters']
    snapshot = dict(estimator=cfg, joint_order=['a', 'b', 'c', 'd'],
                    estimation_overrides=dict(wheel_linear_velocity_variance=1e-6))
    events = [dict(source='wheel', stamp=1., now=1., values=[1., 0.])]
    samples, _, _ = run(events, snapshot, before=True)
    assert samples[0][1][3] > .99999
