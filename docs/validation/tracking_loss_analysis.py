"""Time-aligned read-only reconstruction of physical and controller tracking."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import robot_model_parameters_from_description
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_configuration.yaml_loader import load_yaml
from tracking_loss_geometry import History, pose, planar, wrap, task, interval_fraction


def read(path):
    with path.open() as stream:return [json.loads(line) for line in stream]

def stats(values):
    v=np.asarray(values,dtype=float);v=v[np.isfinite(v)]
    return {'n':len(v),'mean':float(np.mean(v)),'max':float(np.max(v)),
            'rmse':float(np.sqrt(np.mean(v*v))),'p95':float(np.percentile(v,95))} if len(v) else {'n':0}

def interpolate(history,t):
    i=np.searchsorted(history.times,t)
    if i<len(history.times) and abs(history.times[i]-t)<1e-9:return np.array(history.rows[i]['data']['position'])
    if i==0 or i==len(history.times) or history.times[i]-history.times[i-1]>.15:return None
    f=(t-history.times[i-1])/(history.times[i]-history.times[i-1]);return (1-f)*np.array(history.rows[i-1]['data']['position'])+f*np.array(history.rows[i]['data']['position'])

def localization(stream,truth,spawn,max_age):
    output=[]
    for r in stream.rows:
        t=r['source_time'];b=truth.at(t,max_age)
        if b is None:continue
        if 'mean' in r['data']:
            m=r['data']['mean'];estimated=spawn@planar(*m[:3])
        else:estimated=spawn@pose(r['data']['position'],r['data']['orientation'])
        actual=pose(b['data']['position'],b['data']['orientation']);delta=estimated[:3,3]-actual[:3,3];body=actual[:3,:3].T@delta
        yaw=wrap(np.arctan2(estimated[1,0],estimated[0,0])-np.arctan2(actual[1,0],actual[0,0]))
        output.append({'t':t,'translation_m':float(np.linalg.norm(delta[:2])),'yaw_deg':float(yaw*180/np.pi),'longitudinal_m':float(body[0]),'lateral_m':float(body[1]),'truth_age_s':t-b['source_time']})
    return output

def analyze(directory,cfg,output):
    resolved=json.loads((directory/'resolved_configuration.json').read_text());context=resolved['context']
    histories={p.stem:History(read(p)) for p in (directory/'telemetry').glob('*.jsonl') if p.stem in ['base_truth','target_truth','joints','robot_estimate','odometry','target_estimate','target_measurement','prediction','control','actuation']}
    for name,frame in [('base_truth',context['world_frame']),('target_truth',context['world_frame']),('robot_estimate',context['estimator_frame']),('odometry',context['estimator_frame']),('prediction',context['world_frame'])]:
        if any(r['frame_id']!=frame for r in histories[name].rows):raise ValueError(f'{name}: unhandled frame change')
    spawn=context['robot']['pose'];spawn=pose(spawn[:3],Rotation.from_euler('xyz',spawn[3:]).as_quat());parameters=robot_model_parameters_from_description(context['description']);model=create_robot_model(parameters)
    radius=context['control']['tracking_radius'];axis=np.array(context['control']['tracking_forward_axis']);axis=axis/np.linalg.norm(axis);names=context['joint_names'];max_age=cfg['analysis']['max_age']
    loc={name:localization(histories[name],histories['base_truth'],spawn,max_age) for name in ['odometry','robot_estimate']}
    rows=[]
    for r in histories['control'].rows:
        if not r['data']['valid']:continue
        t=r['source_time'];d=r['data'];b=histories['base_truth'].at(t,max_age);j=histories['joints'].at(t,max_age);target=histories['target_truth'].at(t,max_age)
        if b is None or j is None or target is None:continue
        bt=pose(b['data']['position'],b['data']['orientation']);indices=[j['data']['names'].index(n) for n in names];q=np.array(j['data']['position'])[indices];v=np.array(j['data']['velocity'])[indices];camera=WholeBodyKinematics.compute(model,bt,q)
        internal=spawn@pose(d['actual_position'],d['actual_orientation']);used_target=(spawn@np.r_[d['target_position'],1.])[:3];actual_target=np.array(target['data']['position'])
        eR,theta=task(camera,actual_target,axis,radius);iR,ith=task(internal,used_target,axis,radius);used_current=interpolate(histories['target_truth'],t);future=interpolate(histories['target_truth'],t+.25)
        desired=d['desired_pose']['position'];desired_world=(spawn@np.r_[[desired[k] for k in ['x','y','z']],1])[:3]
        row={'t':t,'physical_distance_error_m':eR,'physical_pointing_deg':theta*180/np.pi,'internal_distance_error_m':iR,'internal_pointing_deg':ith*180/np.pi,'task_error_geometry_discrepancy':float(np.linalg.norm(np.r_[iR,ith]-np.r_[d['task_error'][0],np.linalg.norm(d['task_error'][1:])])),
            'camera_model_position_error_m':float(np.linalg.norm(internal[:3,3]-camera[:3,3])),
            'camera_optical_direction_difference_deg':float(np.arccos(np.clip((internal[:3,:3]@axis)@(camera[:3,:3]@axis),-1,1))*180/np.pi),
            'used_target_error_current_m':float(np.linalg.norm(used_target-used_current)) if used_current is not None else np.nan,
            'used_target_error_tplus025_m':float(np.linalg.norm(used_target-future)) if future is not None else np.nan,
            'desired_actual_position_gap_m':float(np.linalg.norm(desired_world-internal[:3,3])),
            'active_bounds':d['active_bounds'],'task_residual':d['task_residual'],'min_scaled_jacobian_singular':min(d['jacobian_singular_values']),'weighted_motion_cost':d['weighted_motion_cost'],'compute_ms':d['computation_time']*1000,
            'min_joint_margin_rad':float(np.min(np.minimum(q-parameters.joint_limits.lower,parameters.joint_limits.upper-q))),
            'target_behind_camera':theta>np.pi/2,'base_truth_age_s':t-b['source_time'],'joint_age_s':t-j['source_time'],'target_truth_age_s':t-target['source_time']}
        for name,value in [('camera',camera[:3,3]),('internal_camera',internal[:3,3]),('target',actual_target),('used_target',used_target),('forward',camera[:3,:3]@axis),('command',d['safe_velocity']),('q',q),('dq',v)]:
            for k,x in enumerate(value):row[f'{name}_{k}']=float(x)
        # Compare ordinary requested velocities with measured response after 0.1 s;
        # this is an observation, not a claim of instantaneous motor realization.
        later=histories['joints'].at(t+.1,max_age)
        if later:
            li=[later['data']['names'].index(n) for n in names];lv=np.array(later['data']['velocity'])[li];row['joint_command_response_mae_rad_s']=float(np.mean(np.abs(lv-np.array(d['safe_velocity'])[2:])))
        rows.append(row)
    prediction_errors={}
    for horizon in [.1,.3,1.]:
        errors=[]
        for r in histories['prediction'].rows:
            t=r['source_time'];d=r['data'];query=t+horizon;times=d['timestamps'];states=np.array(d['states']);truth=interpolate(histories['target_truth'],query)
            if truth is None or not times[0]-1e-9<=query<=times[-1]+1e-9:continue
            predicted=np.array([np.interp(query,times,states[:,k]) for k in range(3)])
            errors.append(np.linalg.norm(predicted-truth))
        prediction_errors[str(horizon)]=stats(errors)
    measurement_errors={}
    for name in ['target_measurement','target_estimate']:
        errors=[]
        for r in histories[name].rows:
            truth=interpolate(histories['target_truth'],r['source_time'])
            if truth is not None:
                value=r['data'].get('position',r['data'].get('mean'))[:3];errors.append(np.linalg.norm(np.array(value)-truth))
        measurement_errors[name]=stats(errors)
    end=min(90.,max(r['t'] for r in rows)) if rows else 90.
    summary={'directory':str(directory),'policy':{'alignment':'latest causal source at most 0.08 s old; no merge of reset epochs','forecast_evaluation':'offline linear target truth interpolation within 0.15s brackets; never used as control input','frames':{'world':context['world_frame'],'odom':context['estimator_frame'],'world_from_odom':spawn.tolist()},'compliance':cfg['analysis'],'optical_axis_local':axis.tolist(),'camera':'authoritative FK from physical base quaternion and measured joints, calibrated by earlier live TF gates'},'stages':{},'prediction_horizon_error_m':prediction_errors,'target_observation_error_m':measurement_errors}
    for low,high in [(0.,10.),(10.,20.),(20.,40.),(40.,90.),(0.,90.)]:
        selected=[r for r in rows if low<=r['t']<high];label=f'{low:g}-{high:g}'
        metrics={key:stats([abs(r[key]) if 'distance_error' in key else r.get(key,np.nan) for r in selected]) for key in ['physical_distance_error_m','physical_pointing_deg','internal_distance_error_m','internal_pointing_deg','camera_model_position_error_m','camera_optical_direction_difference_deg','used_target_error_current_m','used_target_error_tplus025_m','desired_actual_position_gap_m','active_bounds','task_residual','min_scaled_jacobian_singular','compute_ms','joint_command_response_mae_rad_s']}
        compliance=[abs(r['physical_distance_error_m'])<=cfg['analysis']['compliance_distance_m'] and r['physical_pointing_deg']<=cfg['analysis']['compliance_pointing_deg'] for r in selected]
        metrics['compliance']=interval_fraction([r['t'] for r in selected],compliance,low,high,cfg['analysis']['max_gap'])
        metrics['localization']={name:{'translation_m':stats([r['translation_m'] for r in values if low<=r['t']<high]),'absolute_yaw_deg':stats([abs(r['yaw_deg']) for r in values if low<=r['t']<high])} for name,values in loc.items()}
        metrics['saturation_sample_percent']=100*np.mean([r['active_bounds']>0 for r in selected]) if selected else None
        metrics['behind_camera_sample_percent']=100*np.mean([r['target_behind_camera'] for r in selected]) if selected else None
        summary['stages'][label]=metrics
    # Explicit deadband sign-reversal metric, with no smoothing or inferred hidden cycles.
    reversals={}
    for k,name in enumerate(names):
        eligible=[r for r in rows if abs(r[f'dq_{k}'])>.05];sign=np.sign([r[f'dq_{k}'] for r in eligible]);changes=sum(sign[i]!=sign[i-1] and eligible[i]['t']-eligible[i-1]['t']<=.15 for i in range(1,len(sign)))
        reversals[name]={'velocity_sign_changes':int(changes),'deadband_rad_s':.05,'max_eligible_gap_s':.15,'observed_control_window_s':end,'sign_changes_per_second':changes/end}
    summary['oscillation']=reversals
    summary['task_error_reconstruction_max']=max((r['task_error_geometry_discrepancy'] for r in rows),default=None)
    summary['localization_first_divergence']={name:next((r for r in values if r['translation_m']>cfg['analysis']['divergence_translation_m'] or abs(r['yaw_deg'])>cfg['analysis']['divergence_yaw_deg']),None) for name,values in loc.items()}
    output.mkdir(parents=True,exist_ok=False)
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    if rows:
        with (output/'series.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=sorted(set().union(*(r.keys() for r in rows))));writer.writeheader();writer.writerows(rows)
    for name,values in loc.items():
        with (output/(name+'_localization.csv')).open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=['t','translation_m','yaw_deg','longitudinal_m','lateral_m','truth_age_s']);writer.writeheader();writer.writerows(values)
    print(directory.name,'physical vs internal pointing',summary['stages']['0-90']['physical_pointing_deg'],summary['stages']['0-90']['internal_pointing_deg'],flush=True)
    return summary

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path,default=Path(__file__).with_name('tracking_loss_experiments.yaml'));a=p.parse_args();cfg=load_yaml(a.config)
    a.output.mkdir(parents=True,exist_ok=False)
    for i,run in enumerate(a.run):analyze(run,cfg,a.output/f'{i}_{run.name}')
if __name__=='__main__':main()
