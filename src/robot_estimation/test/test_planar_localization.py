"""Independent-pose contract and EKF regression tests."""
import numpy as np
import pytest
from robot_estimation.measurement import Measurement
from robot_estimation.robot.planar_pose import PlanarPoseModel, transform_observation
from robot_estimation.state import StateEstimate
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.models.measurement_model import (
    WheelOdometryMeasurementModel, ImuYawRateMeasurementModel, JointEncoderMeasurementModel)


def create_estimator():
    return RobotStateEstimator(StateEstimate(np.zeros(13),np.eye(13),0.),
        RobotProcessModel(4,.1,.05,.1),WheelOdometryMeasurementModel(4),
        ImuYawRateMeasurementModel(4),JointEncoderMeasurementModel(4))


def test_uncertain_startup_prior_cannot_identify_one_plausible_outlier():
    """Retain the observed limitation rather than claiming NIS solves bootstrap."""
    estimator = create_estimator()
    covariance = np.diag([.04**2+.01**2, .04**2+.01**2,
                          np.deg2rad(2.)**2+np.deg2rad(.5)**2])
    estimator.update_planar_pose(Measurement(np.array([1., 1., .5]), covariance, 0.), 11.345)
    assert estimator.pose_diagnostics['accepted']
    assert estimator.state.mean[0] > .9
    estimator.update_planar_pose(Measurement(np.zeros(3), covariance, 0.), 11.345)
    assert not estimator.pose_diagnostics['accepted']


def test_transform_rotates_covariance_and_handles_nonzero_spawn():
    T=np.eye(4);T[:2,:2]=[[0,-1],[1,0]];T[:3,3]=[2,3,.25]
    value,cov=transform_observation([1,0,.2],np.diag([.01,.04,.02]),T,1e-6)
    np.testing.assert_allclose(value,[2,4,.2+np.pi/2])
    np.testing.assert_allclose(cov,np.diag([.04,.01,.02]))


def test_invalid_covariance_or_nonplanar_transform_rejected():
    for cov in [np.diag([1,-1,1]),np.zeros((3,3)),np.full((3,3),np.nan)]:
        with pytest.raises(ValueError):transform_observation([0,0,0],cov,np.eye(4),1e-6)
    T=np.eye(4);T[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    with pytest.raises(ValueError):transform_observation([0,0,0],np.eye(3),T,1e-6)


def test_pose_jacobian_and_wrapped_innovation():
    model=PlanarPoseModel(4);x=np.zeros(13);x[2]=np.pi-.01
    np.testing.assert_allclose(model.jacobian(x)[:,:3],np.eye(3))
    np.testing.assert_allclose(model.innovation(np.array([0,0,-np.pi+.01]),x),[0,0,.02],atol=1e-12)


def test_ekf_fuses_pose_preserves_joints_and_valid_covariance():
    e=create_estimator();before=e.state.mean.copy();z=Measurement([.1,-.1,.02],np.diag([.01,.01,.001]),0.)
    e.update_planar_pose(z,11.345)
    assert e.pose_diagnostics['accepted']
    assert np.linalg.norm(e.state.mean[:2]-z.value[:2])<.01
    np.testing.assert_array_equal(e.state.mean[5:],before[5:])
    assert np.linalg.eigvalsh(e.state.covariance).min()>0


def test_outlier_is_rejected_without_measurement_mutation():
    e=create_estimator();before=e.state;z=Measurement([100,100,0],np.eye(3)*.01,0.);raw=z.value.copy()
    e.update_planar_pose(z,11.345)
    assert not e.pose_diagnostics['accepted'];np.testing.assert_array_equal(e.state.mean,before.mean)
    np.testing.assert_array_equal(e.state.covariance,before.covariance);np.testing.assert_array_equal(z.value,raw)
def test_contact_slip_diffusion_is_optional_and_discretized():
    from robot_estimation.robot.models.process_model import RobotProcessModel
    base = RobotProcessModel(4, 0., 0., 0.)
    slip = RobotProcessModel(4, 0., 0., 0., planar_slip_noise_std=.15,
                            yaw_slip_noise_std=.2)
    state = np.zeros(13)
    assert np.array_equal(base.process_covariance(.1, state), np.zeros((13, 13)))
    q = slip.process_covariance(.1, state)
    assert np.allclose(q.diagonal()[:3], [.15**2*.1, .15**2*.1, .2**2*.1])
    assert np.allclose(q, 2*slip.process_covariance(.05, state))
    assert np.linalg.eigvalsh(q).min() >= 0
    with pytest.raises(ValueError):
        RobotProcessModel(4, planar_slip_noise_std=-1.)

