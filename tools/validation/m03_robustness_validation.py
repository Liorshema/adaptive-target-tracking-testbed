"""Source-ordered replay of retained M03 observations; truth used only for scoring."""
import argparse
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.spatial.transform import Rotation
from robot_configuration.yaml_loader import load_yaml
from robot_estimation.configuration import load_estimation_parameters
from robot_estimation.factory.estimator_factory import create_robot_estimator
from robot_estimation.parameter_schema import POSE_POLICY_PARAMETERS
from robot_estimation.measurement import Measurement
from robot_estimation.robot.planar_pose import transform_observation
from m03_localization_metrics import aligned_pose
from tracking_loss_analysis import stats
from tracking_loss_geometry import interval_fraction
from joint_limit_recovery_analysis import control_continuity, stalled_windows, telemetry_summary
from robot_experiments.metrics.manipulation import analyze_phases


def replay(directory):
    resolved=json.loads((directory/'resolved_configuration.json').read_text())
    context=resolved['context']
    parameters=load_yaml(directory/'inputs/estimation.yaml')['estimation_node']['ros__parameters']
    read=lambda key:[json.loads(line) for line in (directory/'telemetry'/f'{key}.jsonl').read_text().splitlines()]
    diagnostics=[r['data']['parsed'] for r in read('diagnostics') if r['data'].get('parsed')]
    first=next(r['localization']['last_accepted_time'] for r in diagnostics
               if r.get('localization',{}).get('updates',0)>0)
    pose=context['robot']['pose'];world_from_odom=np.eye(4)
    world_from_odom[:3,:3]=Rotation.from_euler('xyz',pose[3:]).as_matrix()
    world_from_odom[:3,3]=pose[:3];odom_from_world=np.linalg.inv(world_from_odom)
    events=[];count=len(context['joint_names'])
    for source in ['odometry','joints','localization_measurement']:
        for record in read(source):
            t=record['source_time'];d=record['data']
            if source=='odometry':
                z=[d['linear_velocity'][0],d['angular_velocity'][2]]
                R=np.diag([parameters['wheel_linear_velocity_variance'],parameters['wheel_angular_velocity_variance']])
            elif source=='joints':
                indices=[d['names'].index(n) for n in context['joint_names']]
                z=np.r_[np.array(d['position'])[indices],np.array(d['velocity'])[indices]]
                R=np.diag([parameters['joint_position_variance']]*count+[parameters['joint_velocity_variance']]*count)
            else:
                if t<first-1e-9:continue
                yaw=Rotation.from_quat(d['orientation']).as_euler('xyz')[2]
                R=np.array(d['pose_covariance']).reshape(6,6)[np.ix_([0,1,5],[0,1,5])]
                z,R=transform_observation([*d['position'][:2],yaw],R,odom_from_world,parameters['localization_transform_tolerance'])
            events.append((t,source,Measurement(z,R,t)))
    events.sort(key=lambda item:item[:2])
    physical=context['description']['robot']['manipulator']['joints']
    configured=load_estimation_parameters(Path('src/robot_estimation/config/estimation.yaml'), ['robot_ekf','pose_aiding'])['estimation_node']['ros__parameters']
    truth=read('base_truth');output={}
    for mode in ['legacy','robust']:
        selected=dict(parameters)
        selected.update({key:configured[key] for key in POSE_POLICY_PARAMETERS})
        if mode=='legacy':selected['localization_consensus_window']=0
        estimator,_=create_robot_estimator(selected,physical)
        last={};rows=[];cost=[]
        for stamp,source,measurement in events:
            if stamp<=last.get(source,-1.):continue
            last[source]=stamp
            state=estimator.state
            if state.timestamp is not None and stamp>state.timestamp:
                estimator.predict(stamp-state.timestamp)
            if source=='odometry':estimator.update_wheel_odometry(measurement)
            elif source=='joints':estimator.update_joint_encoders(measurement)
            else:
                start=perf_counter();estimator.update_planar_pose(measurement,selected['localization_nis_limit'])
                cost.append((perf_counter()-start)*1e6)
                state=estimator.state;health=estimator.localization_health(stamp)
                rows.append({'t':stamp,'mean':state.mean[:3].tolist(),'covariance':state.covariance[:3,:3].tolist(),
                    'health':health,'diagnostics':dict(estimator.pose_diagnostics)})
        times=np.array([r['t'] for r in rows]);actual,valid=aligned_pose(truth,times)
        estimated=np.array([r['mean'] for r in rows]);estimated[:,:2]=estimated[:,:2]@world_from_odom[:2,:2].T+world_from_odom[:2,3]
        estimated[:,2]+=pose[5];error=estimated-actual
        error[:,2]=np.arctan2(np.sin(error[:,2]),np.cos(error[:,2]))
        for row,delta in zip(rows,error):row['xy_error']=float(np.linalg.norm(delta[:2]));row['yaw_error_deg']=float(abs(delta[2])*180/np.pi)
        accepted=[r for r in rows if r['diagnostics']['accepted']]
        output[mode]={'first_accepted':accepted[0] if accepted else None,
            'accepted':len(accepted),'final_diagnostics':estimator.pose_diagnostics,
            'pose_update_compute_us':{'mean':float(np.mean(cost)),'max':float(np.max(cost))},
            'rows':rows}
    return {'directory':str(directory),'first_available_pose':first,
        'policy':'Identical source-ordered recorded observations; first usable pose from original diagnostics. Offline interpolation only for scoring. Arrival timing/ROS ordering are not replayed.',
        'results':output}


def audit(directory):
    """Include physical holds and unpublished diagnostic poses in the audit."""
    resolved=json.loads((directory/'resolved_configuration.json').read_text())
    result=json.loads((directory/'result.json').read_text());context=resolved['context']
    if result.get('error'):return {'directory':str(directory),'failed_attempt':result}
    end=min(90.,result['window']['window_end'])
    streams={p.stem:[json.loads(line) for line in p.read_text().splitlines()]
             for p in (directory/'telemetry').glob('*.jsonl') if p.stem in [
                 'base_truth','target_truth','joints','control','actuation','diagnostics','robot_estimate']}
    settings=load_yaml(Path('src/robot_experiments/config/manipulation_analysis.yaml'))
    physical=analyze_phases(streams,context,[],0.,end,max_gap=settings['max_gap'],
        max_skew=settings['max_skew'],overhead_region=settings['overhead_region'])
    rows=[r for r in physical['series'] if r['distance_error_m'] is not None and r['pointing_error_rad'] is not None and 0<=r['time']<=end]
    compliance=interval_fraction([r['time'] for r in rows],
        [r['distance_error_m']<=.2 and r['pointing_error_rad']<=np.deg2rad(15.) for r in rows],0.,end,.15)
    control={}
    first=result['first_valid_control_time']
    for label,start in [('all',0.),('active',first)]:
        value=control_continuity(directory/'telemetry/control.jsonl',start,end)
        value['valid_fraction']=value['valid_samples']/value['samples'] if value['samples'] else None
        control[label]=value
    truth=streams['base_truth'];spawn=np.array(context['robot']['pose']);rotation=Rotation.from_euler('xyz',spawn[3:]).as_matrix()[:2,:2]
    def errors(times,means,covariance=None):
        means=np.asarray(means);actual,valid=aligned_pose(truth,times)
        world=means.copy();world[:,:2]=means[:,:2]@rotation.T+spawn[:2];world[:,2]+=spawn[5]
        delta=world-actual;delta[:,2]=np.arctan2(np.sin(delta[:,2]),np.cos(delta[:,2]))
        selection=valid&(times>=20.)&(times<=end) if end>=90 else valid
        selected=delta[selection]
        summary={'xy_m':stats(np.linalg.norm(selected[:,:2],axis=1)),
            'yaw_deg':stats(np.abs(selected[:,2])*180/np.pi),
            'first':{'time':float(times[0]),'xy_m':float(np.linalg.norm(delta[0,:2])),
                     'yaw_deg':float(abs(delta[0,2])*180/np.pi)}}
        if covariance is not None:
            covariance=np.array(covariance);J=np.eye(3);J[:2,:2]=rotation
            covariance=J@covariance@J.T
            nees=np.einsum('ni,nij,nj->n',selected,np.linalg.inv(covariance[selection]),selected)
            summary['nees']=stats(nees);summary['min_covariance_eigenvalue']=float(np.linalg.eigvalsh(covariance).min())
        return summary
    published=streams.get('robot_estimate',[])
    published_metrics=errors(np.array([r['source_time'] for r in published]),
        [r['data']['mean'][:3] for r in published],
        [np.array(r['data']['covariance']).reshape(len(r['data']['mean']),-1)[:3,:3] for r in published]) if published else None
    diagnostics=[r['data']['parsed'] for r in streams['diagnostics'] if r['data'].get('parsed')]
    classifications={}
    for record in streams['control']:
        if record['data'].get('valid') or record['source_time']>end:
            continue
        t=record['source_time'];reason=record['data'].get('reason','')
        prior=[d for d in diagnostics if 0<=t-d['time']<=.3]
        d=prior[-1] if prior else None
        if t<first:
            category='launch_readiness'
        elif 'prediction' in reason.lower():
            category='target_prediction_or_context'
        elif (d and not d.get('localization',{}).get('trusted',True)
                and d['filter_time'] is not None and t-d['filter_time']<.2
                and d.get('raw_joint_valid',False)):
            category='localization_hold_with_fresh_internal_filter'
        else:
            category='other_robot_freshness_or_unresolved'
        classifications[category]=classifications.get(category,0)+1
    localization=[d['localization']|{'time':d['time']} for d in diagnostics if 'localization' in d]
    internal=[d for d in localization if d.get('estimate_time') is not None and 'estimate_pose' in d]
    internal_metrics=errors(np.array([d['estimate_time'] for d in internal]),
        [d['estimate_pose'] for d in internal],[d['pose_covariance'] for d in internal]) if internal else None
    health=interval_fraction([d['time'] for d in localization],
        [d.get('trusted',False) for d in localization],0.,end,.3)
    after_outage=[r['source_time'] for r in published if r['source_time']>=7.]
    return {'directory':str(directory),'end':end,'result':result,
        'physical_all_control_states':physical['windows']['active'],
        'physical_equal_sample_distance_m':stats([r['distance_error_m'] for r in rows]),
        'physical_equal_sample_pointing_deg':stats([r['pointing_error_rad']*180/np.pi for r in rows]),
        'compliance':compliance,'control':control,'published_localization':published_metrics,
        'internal_localization_including_untrusted':internal_metrics,
        'health':health,'first_published_after_outage':min(after_outage,default=None),
        'last_localization_diagnostic':localization[-1] if localization else None,
        'fusion_errors':diagnostics[-1]['fusion_errors'] if diagnostics else None,
        'invalid_control_classification':classifications,
        'stall':stalled_windows(directory/'physics/actuator.csv'),
        'actuation':telemetry_summary(directory/'telemetry',first,end)}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--replay',type=Path);mode.add_argument('--audit',type=Path,nargs='+')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=replay(a.replay) if a.replay else {str(path):audit(path) for path in a.audit}
    with a.output.open('x') as stream:json.dump(result,stream,indent=2)
    if a.replay:
        for mode,value in result['results'].items():
            first=value['first_accepted'];print(mode,'first:',None if first is None else (first['t'],first['xy_error'],first['yaw_error_deg']),value['final_diagnostics'],flush=True)
    else:
        for path,value in result.items():
            print(path,value.get('internal_localization_including_untrusted'),flush=True)
