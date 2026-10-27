"""Replay captured simulator feedback through the actual estimator offline."""
from robot_estimation.configuration import load_estimation_parameters
import json
from pathlib import Path
import numpy as np
import yaml
from robot_estimation.state import StateEstimate
from robot_estimation.measurement import Measurement
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.models.measurement_model import WheelOdometryMeasurementModel,ImuYawRateMeasurementModel,JointEncoderMeasurementModel
ROOT=Path('/home/user/ros2_ws');OUT=ROOT/'results/validation/v2_estimation_audit';data=json.loads((OUT/'streams.json').read_text());cfg=load_estimation_parameters(ROOT/'src/robot_estimation/config/estimation.yaml', 'robot_ekf')['estimation_node']['ros__parameters']
odom=data['odom'];ot=np.array([r['t'] for r in odom]);op=np.array([r['base'] for r in odom]);op[:,2]=np.unwrap(op[:,2]);ov=np.array([r['velocity'] for r in odom]);jt=np.array([r['t'] for r in data['joint']]);q=np.array([r['q'] for r in data['joint']])
events=[(row['t'],row['received'],kind,row) for kind in ['joint','odom'] for row in data[kind]]
def run(order,scale):
 process=RobotProcessModel(4,cfg['linear_acceleration_noise_std'],cfg['angular_acceleration_noise_std'],cfg['joint_acceleration_noise_std'])
 est=RobotStateEstimator(StateEstimate(np.array(cfg['robot_initial_state']),np.diag(cfg['robot_initial_covariance_diagonal']),timestamp=None),process,WheelOdometryMeasurementModel(4),ImuYawRateMeasurementModel(4),JointEncoderMeasurementModel(4))
 last=None;accepted={'odom':0,'joint':0};dropped={'odom':0,'joint':0};errors=[];ve=[];qe=[];laststate=None
 sorted_events=sorted(events,key=lambda r:r[0] if order=='timestamp' else r[1])
 for t,wall,kind,row in sorted_events:
  if last is not None and t<last:dropped[kind]+=1;continue
  if last is not None and t>last:est.predict(t-last)
  last=t;accepted[kind]+=1
  if kind=='odom':
   est.update_wheel_odometry(Measurement(np.array(row['velocity']),np.diag([cfg['wheel_linear_velocity_variance'],cfg['wheel_angular_velocity_variance']])*scale,t))
   state=est.state.mean
   if t>ot[0]+5:
    truth=np.array(row['base']);errors.append(state[:3]-truth);ve.append(state[3:5]-row['velocity']);qe.append(state[5:9]-np.array([np.interp(t,jt,q[:,i]) for i in range(4)]))
  else:
   est.update_joint_encoders(Measurement(np.array(row['q']+row['dq']),np.diag([cfg['joint_position_variance']]*4+[cfg['joint_velocity_variance']]*4),t))
 laststate=est.state.mean
 errors=np.array(errors);ve=np.array(ve);qe=np.array(qe)
 return {'order':order,'wheel_R_scale':scale,'accepted':accepted,'dropped':dropped,'mean_base_position_error':float(np.linalg.norm(errors[:,:2],axis=1).mean()),'max_base_position_error':float(np.linalg.norm(errors[:,:2],axis=1).max()),'mean_abs_yaw_error':float(np.abs(errors[:,2]).mean()),'mean_abs_velocity_error':np.abs(ve).mean(axis=0).tolist(),'mean_abs_joint_errors':np.abs(qe).mean(axis=0).tolist(),'last_estimated_base':laststate[:5].tolist()}
results=[]
for order,scale in [('timestamp',1.),('arrival',1.),('timestamp',.001),('arrival',.001),('timestamp',.00002),('arrival',.00002)]:
 result=run(order,scale);results.append(result);print(json.dumps(result),flush=True)
(OUT/'replay.json').write_text(json.dumps(results,indent=2)+'\n')
