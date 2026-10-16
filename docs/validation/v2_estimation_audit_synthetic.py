"""Quantify cadence sensitivity without changing production parameters."""
import json
import numpy as np
from robot_estimation.state import StateEstimate
from robot_estimation.measurement import Measurement
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.models.measurement_model import WheelOdometryMeasurementModel,ImuYawRateMeasurementModel,JointEncoderMeasurementModel

def build():
 return RobotStateEstimator(StateEstimate(np.zeros(13),np.eye(13),timestamp=0.),RobotProcessModel(4,.1,.05,.1),WheelOdometryMeasurementModel(4),ImuYawRateMeasurementModel(4),JointEncoderMeasurementModel(4))
def run(joint_hz):
 est=build();previous=0.;errors=[];truth=np.zeros(3);rates=[]
 for step in range(1,10001):
  t=step*.002;v=.3*np.sin(.6*t);w=.2*np.sin(.4*t)
  truth[0]+=.002*v*np.cos(truth[2]);truth[1]+=.002*v*np.sin(truth[2]);truth[2]+=.002*w
  wheel=(step%10==0);joint=(joint_hz==500 or (joint_hz==50 and wheel))
  if not (wheel or joint):continue
  est.predict(t-previous);previous=t
  if wheel:est.update_wheel_odometry(Measurement(np.array([v,w]),np.diag([.05,.02]),t))
  if joint:est.update_joint_encoders(Measurement(np.zeros(8),np.eye(8)*.02,t))
  if wheel and t>5:
   errors.append(est.state.mean[:3]-truth);rates.append(est.state.mean[3:5]-np.array([v,w]))
 errors=np.array(errors);rates=np.array(rates)
 return {'joint_hz':joint_hz,'mean_position_error':float(np.linalg.norm(errors[:,:2],axis=1).mean()),'max_position_error':float(np.linalg.norm(errors[:,:2],axis=1).max()),'mean_abs_yaw_error':float(np.abs(errors[:,2]).mean()),'mean_abs_velocity_errors':np.abs(rates).mean(axis=0).tolist(),'final_truth':truth.tolist(),'final_estimate':est.state.mean[:5].tolist()}
print(json.dumps([run(50),run(500)],indent=2))
