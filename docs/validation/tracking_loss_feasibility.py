"""Same-recorded-state rate-feasibility counterfactual; no controller/physics edits."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from robot_configuration.yaml_loader import load_yaml
from robot_control.factory.project_configuration import configured_control
from robot_control.factory.control_factory import ControlFactory
from robot_control.factory.project_components import create_project_registries
from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import robot_model_parameters_from_description
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_models.whole_body.jacobian import WholeBodyJacobian
from tracking_loss_analysis import read
from tracking_loss_geometry import pose,planar

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();c=json.loads((a.run/'resolved_configuration.json').read_text())['context'];ctrl=load_yaml(a.run/'inputs/control.yaml')['control_node']['ros__parameters'];manifest=load_yaml(a.run/'inputs/system.yaml');strategy=load_yaml(manifest['control']['strategy']);metadata=json.loads((a.run/'diagnostic_result.json').read_text());config=configured_control(ctrl,c['description'],strategy,lookahead=str(metadata['intervention']['target_lookahead']));pipeline=ControlFactory(create_project_registries()).build(config);wls=pipeline._controller._tracking._controller
 model=create_robot_model(robot_model_parameters_from_description(c['description']));est=read(a.run/'telemetry/robot_estimate.jsonl');controls=[r for r in read(a.run/'telemetry/control.jsonl') if r['data']['valid']];forecasts=read(a.run/'telemetry/prediction.jsonl');spawn=c['robot']['pose'];T=pose(spawn[:3],Rotation.from_euler('xyz',spawn[3:]).as_quat());inv=np.linalg.inv(T);out=[]
 for query in [35,45,55,65,75,85]:
  r=min(controls,key=lambda r:abs(r['source_time']-query));t=r['source_time'];d=r['data'];actual=pose(d['actual_position'],d['actual_orientation']);options=[]
  for state in est:
   if not t-.2<=state['source_time']<=t+.01:continue
   mean=np.array(state['data']['mean']);fk=WholeBodyKinematics.compute(model,planar(*mean[:3]),mean[5:9]);options.append((np.linalg.norm(fk-actual),mean))
  discrepancy,mean=min(options,key=lambda x:x[0]);predoptions=[];querytime=t+config['tracking']['lookahead_time']
  for prediction in forecasts:
   if not t-.5<=prediction['source_time']<=t+.01:continue
   times=np.array(prediction['data']['timestamps']);states=np.array(prediction['data']['states']);index=np.searchsorted(times,querytime)
   if index>=len(times):continue
   if index==0:s=states[0].copy();s[:3]+=(querytime-times[0])*s[3:]
   else:f=(querytime-times[index-1])/(times[index]-times[index-1]);s=states[index-1]*(1-f)+states[index]*f
   s[:3]=(inv@np.r_[s[:3],1])[:3];s[3:]=inv[:3,:3]@s[3:];predoptions.append((np.linalg.norm(s[:3]-d['target_position']),s))
  target_discrepancy,target=min(predoptions,key=lambda x:x[0]);Jsp=WholeBodyJacobian.compute(model,planar(*mean[:3]),mean[5:9]);error,J,rate=wls.task.geometry.evaluate(actual,np.array(d['target_position']),Jsp,target[3:]);desired=-wls.gains*error-rate
  previous=[x for x in controls if x['source_time']<t][-1];dt=max(.05,t-previous['source_time']);lower,upper=wls.safety.feasible_bounds(np.array(previous['data']['safe_velocity']),mean[5:9],dt);free=upper>lower;u=lower.copy()
  if np.any(free):u[free]=lsq_linear(J[:,free],desired-J[:,~free]@u[~free],bounds=(lower[free],upper[free]),method='bvls',tol=1e-10).x
  command=np.array(d['safe_velocity']);out.append({'t':t,'state_pose_reconstruction_error':float(discrepancy),'selected_target_reconstruction_error_m':float(target_discrepancy),'recorded_residual':d['task_residual'],'reconstructed_residual':float(np.linalg.norm(J@command-desired)),'best_unweighted_rate_residual_same_bounds':float(np.linalg.norm(J@u-desired)),'lower':lower.tolist(),'upper':upper.tolist(),'q':mean[5:9].tolist(),'physical_goal':'instantaneous requested error-rate feasibility only, not global pose reachability'})
 a.output.mkdir(parents=True,exist_ok=False);(a.output/'summary.json').write_text(json.dumps(out,indent=2)+'\n')
 for r in out:print(r['t'],'reconstruction',r['state_pose_reconstruction_error'],r['selected_target_reconstruction_error_m'],'residual selected / best',r['reconstructed_residual'],r['best_unweighted_rate_residual_same_bounds'])
 if any(r['state_pose_reconstruction_error']>1e-6 or r['selected_target_reconstruction_error_m']>1e-6 or abs(r['recorded_residual']-r['reconstructed_residual'])>1e-6 for r in out):raise SystemExit('Reconstruction not established; do not attribute feasibility.')
if __name__=='__main__':main()
