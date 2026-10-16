"""Compare measured streams at common stamps and identify accepted wheel updates."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from robot_bringup.factory.model_factory import create_robot_model
from robot_models.whole_body.kinematics import WholeBodyKinematics
ROOT=Path('/home/user/ros2_ws');OUT=ROOT/'docs/validation/v2_estimation_audit'
data=json.loads((OUT/'streams.json').read_text());est=data['estimate'];odom=data['odom'];joint=data['joint']
te=np.array([v['t'] for v in est]);to=np.array([v['t'] for v in odom]);tj=np.array([v['t'] for v in joint]);xe=np.array([v['mean'] for v in est]);po=np.array([v['base'] for v in odom]);vo=np.array([v['velocity'] for v in odom]);qj=np.array([v['q'] for v in joint]);dqj=np.array([v['dq'] for v in joint]);pe=np.array([v['pdiag'] for v in est])
po[:,2]=np.unwrap(po[:,2]);oi=np.column_stack([np.interp(te,to,po[:,i]) for i in range(3)]);vi=np.column_stack([np.interp(te,to,vo[:,i]) for i in range(2)]);qi=np.column_stack([np.interp(te,tj,qj[:,i]) for i in range(4)]);dqi=np.column_stack([np.interp(te,tj,dqj[:,i]) for i in range(4)])
mask=(te>=max(to[0],tj[0])+5.)&(te<=min(to[-1],tj[-1]));idx=np.flatnonzero(mask);diff=xe[mask,:3]-oi[mask];dq=xe[mask,5:9]-qi[mask]
model=create_robot_model();fkerrors=[];opticalerrors=[]
for i in idx[::5]:
 actual=np.eye(4);actual[:3,:3]=Rotation.from_euler('z',oi[i,2]).as_matrix();actual[:2,3]=oi[i,:2]
 estimated=np.eye(4);estimated[:3,:3]=Rotation.from_euler('z',xe[i,2]).as_matrix();estimated[:2,3]=xe[i,:2]
 A=WholeBodyKinematics.compute(model,actual,qi[i]);E=WholeBodyKinematics.compute(model,estimated,xe[i,5:9])
 fkerrors.append(np.linalg.norm(A[:3,3]-E[:3,3]));opticalerrors.append(np.degrees(np.arccos(np.clip(A[:3,2]@E[:3,2],-1.,1.))))
wheelupdates=np.flatnonzero(np.diff(pe[:,3]) < -1e-12)+1
wheel_t=te[wheelupdates];r={
 'counts':{k:len(v) for k,v in data.items()},'time_span':[float(te[0]),float(te[-1])],'frames':sorted(set(v['frame'] for v in est)),
 'mean_base_position_error':float(np.linalg.norm(diff[:,:2],axis=1).mean()),'max_base_position_error':float(np.linalg.norm(diff[:,:2],axis=1).max()),
 'mean_abs_base_component_errors':np.abs(diff).mean(axis=0).tolist(),'last_estimated_base':xe[-1,:5].tolist(),'last_odometry_base':oi[-1].tolist(),'last_odometry_velocity':vi[-1].tolist(),
 'mean_abs_velocity_errors':np.abs(xe[mask,3:5]-vi[mask]).mean(axis=0).tolist(),'mean_abs_joint_errors':np.abs(dq).mean(axis=0).tolist(),'max_abs_joint_errors':np.abs(dq).max(axis=0).tolist(),'mean_abs_joint_velocity_errors':np.abs(xe[mask,9:13]-dqi[mask]).mean(axis=0).tolist(),
 'mean_tracking_position_disagreement':float(np.mean(fkerrors)),'max_tracking_position_disagreement':float(np.max(fkerrors)),'mean_optical_axis_disagreement_degrees':float(np.mean(opticalerrors)),
 'inferred_wheel_updates_from_decreasing_velocity_covariance':int(len(wheelupdates)),
 'inferred_wheel_update_timestamp_alignment_max':float(np.max(np.min(np.abs(wheel_t[:,None]-to[None,:]),axis=1))),
 'last_velocity_covariance':pe[-1,3:5].tolist(),
 'nominal_source_periods':{k:float(np.median(np.diff(np.unique([v['t'] for v in rows])))) for k,rows in data.items()},
}
(OUT/'comparison.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
