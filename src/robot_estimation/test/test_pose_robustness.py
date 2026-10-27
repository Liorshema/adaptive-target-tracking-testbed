"""Causal localization startup/recovery tests with the actual robot EKF."""
from robot_estimation.configuration import load_estimation_parameters
from pathlib import Path
import numpy as np
import pytest
import yaml
from robot_estimation.factory.estimator_factory import create_robot_estimator
from robot_estimation.measurement import Measurement


def estimator(yaw=0.):
    parameters = load_estimation_parameters(Path(__file__).parents[1]/'config/estimation.yaml', 'robot_ekf')['estimation_node']['ros__parameters']
    parameters.update(joint_bounds_enabled=False, planar_slip_noise_std=.15, yaw_slip_noise_std=.2,
        localization_enabled=True, localization_nis_limit=11.345,
        localization_consensus_window=5, localization_consensus_inliers=4,
        localization_consensus_span=.3, localization_consensus_max_age=.8,
        localization_trust_timeout=.35, localization_recovery_nis_target=3.)
    parameters['robot_initial_state'][2] = yaw
    result, _ = create_robot_estimator(parameters)
    result.update_wheel_odometry(Measurement([0., 0.], np.diag([.02,.04]), 0.))
    return result


def observe(e, time, value):
    if time > e.state.timestamp:
        e.predict(time-e.state.timestamp)
    e.update_planar_pose(Measurement(value, np.diag([.0017,.0017,.0013]), time), 11.345)


@pytest.mark.parametrize('bad_indices', [[], [0], [0,1], [0,1,2], [0,3]])
def test_initial_corruption_cannot_establish_the_reference(bad_indices):
    e = estimator()
    for i in range(10):
        before = e.state.mean.copy()
        corrupt = i in bad_indices
        observe(e, (i+1)*.1, [1.,1.,.5] if corrupt else [.01,-.01,.01])
        if corrupt:
            assert not e.pose_diagnostics['accepted']
            np.testing.assert_array_equal(e.state.mean,before)
        if i < 4:
            assert not e.pose_diagnostics['accepted']
    assert e.pose_diagnostics['updates'] > 0
    assert np.linalg.norm(e.state.mean[:2]) < .1
    assert abs(e.state.mean[2]) < .1


def test_confirmed_heading_mismatch_reacquires_without_a_truth_reset():
    e = estimator()
    for i in range(5): observe(e,(i+1)*.1,[0.,0.,0.])
    assert e.localization_health(.5)['trusted']
    # An externally observed turn disagrees with the ordinary wheel-only model.
    for i in range(5): observe(e,.6+i*.1,[.05,-.03,1.])
    assert e.localization_health(1.)['trusted']
    assert abs(e.state.mean[2]-1.) < .1
    assert e.pose_diagnostics['recoveries'] == 1
    assert e.pose_diagnostics['nis_before_recovery'] > 11.345
    assert e.pose_diagnostics['nis'] <= 11.345
    np.testing.assert_allclose(e.state.covariance,e.state.covariance.T,atol=1e-12)
    assert np.linalg.eigvalsh(e.state.covariance).min() > 0


def test_outage_and_duplicate_require_fresh_source_evidence():
    e = estimator()
    for i in range(5):observe(e,.1*(i+1),[0.,0.,0.])
    before=e.state
    observe(e,.5,[1.,1.,1.])
    np.testing.assert_array_equal(e.state.mean,before.mean)
    np.testing.assert_array_equal(e.state.covariance,before.covariance)
    assert not e.localization_health(1.)['trusted']
    for i in range(5):observe(e,2.+i*.1,[.1,.1,.3])
    assert e.localization_health(2.4)['trusted']


def test_wrap_and_no_source_keep_uncertainty_honest():
    e=estimator(np.pi-.01)
    assert not e.localization_health(0.)['trusted']
    assert np.isclose(e.state.covariance[0,0],1.)
    for i in range(5):observe(e,.1*(i+1),[0.,0.,-np.pi+.01])
    assert e.localization_health(.5)['trusted']
    assert abs(np.arctan2(np.sin(e.state.mean[2]-np.pi),np.cos(e.state.mean[2]-np.pi))) < .03


def test_invalid_or_past_pose_does_not_mutate_trusted_filter():
    e=estimator()
    for i in range(5):observe(e,.1*(i+1),[0.,0.,0.])
    before=e.state
    for measurement in [Measurement([1.,1.,1.],np.eye(3),.4),
                        Measurement([np.nan,0.,0.],np.eye(3),.5),
                        Measurement([1.,1.,1.],np.diag([1.,-1.,1.]),.5)]:
        with pytest.raises(ValueError):e.update_planar_pose(measurement,11.345)
        np.testing.assert_array_equal(e.state.mean,before.mean)
        np.testing.assert_array_equal(e.state.covariance,before.covariance)


def test_long_outage_and_no_consensus_never_force_acceptance():
    e=estimator()
    for i in range(5):observe(e,.1*(i+1),[0.,0.,0.])
    e.predict(10.)
    before=e.state
    assert not e.localization_health(10.5)['trusted']
    for i in range(20):
        # Alternating separated clusters cannot supply four compatible observations.
        observe(e,10.6+i*.1,[10.*(i%2),0.,0.])
        assert not e.localization_health(10.6+i*.1)['trusted']
        assert len(e._pose_acceptance.history)<=5
    np.testing.assert_array_equal(e.state.mean,before.mean)


def test_invalid_uncertainty_increment_is_nonmutating():
    e=estimator();before=e.state
    for increment in [np.eye(13)*-1., np.full((13,13),np.nan), np.eye(3)]:
        with pytest.raises(ValueError):e._filter.add_process_uncertainty(increment)
        np.testing.assert_array_equal(e.state.covariance,before.covariance)
