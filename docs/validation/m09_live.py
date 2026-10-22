"""Bounded real M09 marker/path/TF verification using the M08 recorder/lifecycle."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path as PathMessage
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from robot_configuration.yaml_loader import load_yaml
from robot_description.model_parameters import robot_model_parameters_from_description
from robot_bringup.factory.model_factory import create_robot_model
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_experiments.configuration import load_experiment, materialize_run
from robot_experiments.execution import DomainLease, OwnedLaunch, run_window
from robot_experiments.recording import Recorder
from robot_experiments.recording.normalization import message_data
from robot_experiments.metrics import evaluate
from robot_experiments.metrics.alignment import SourceIndex
from robot_experiments.reporting import create_study_directory, write_json
from scipy.spatial.transform import Rotation


def stamp(header):
    if isinstance(header,dict):
        return header['stamp']['sec']+header['stamp']['nanosec']*1e-9
    return header.stamp.sec+header.stamp.nanosec*1e-9


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--compute-only',action='store_true')
    parser.add_argument('--duration',type=float,default=None)
    parser.add_argument('--settling-time',type=float,default=3.)
    args=parser.parse_args()
    directory=create_study_directory(args.output,'gui' if args.gui else 'headless')
    share=lambda package:Path(get_package_share_directory(package))
    spec=load_experiment(share('robot_experiments')/'config/nominal.yaml',share)
    if args.compute_only:
        spec['runtime']['enable_actuation']=False
        variant=spec['variants'][0]
        variant['plan']=load_system_configuration(variant['plan']['source'],share,overrides={
            'enable_actuation':'false','publish_commands':'false',
            'publish_ground_truth':'true','gui':'false','visualize_tracking':'false'})
    prepared=materialize_run(spec['variants'][0],42,directory,share,spec['runtime'])
    cfg=load_yaml(share('robot_bringup')/'config/tracking_visualization.yaml')
    context=prepared['context'];process=recorder=observer=observer_executor=None;window=None;cleanup=None
    received={'markers':[],'paths':{},'tf':[]};handles=[]
    result={'schema_version':1,'directory':str(directory),'gui_requested':args.gui,'checks':{}}
    try:
        with DomainLease(log_dir=directory/'ros-log') as lease:
            old={k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','ROS_LOG_DIR']}
            try:
                os.environ.update({k:lease.env[k] for k in old})
                recorder=Recorder(context,directory/'telemetry')
                from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
                from rclpy.node import Node
                from rclpy.executors import SingleThreadedExecutor
                observer=Node('m09_display_probe',context=recorder._ros_context)
                observer_executor=SingleThreadedExecutor(context=recorder._ros_context)
                observer_executor.add_node(observer)
                observed=[0]
                qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.BEST_EFFORT)
                def capture(key,message):
                    # Separate executor prevents high-rate physical recording starving displays.
                    observed[0] += 1
                    raw=message_data(message)
                    if key.startswith('path:'):received['paths'].setdefault(key[5:],[]).append(raw)
                    else:received[key].append(raw)
                observer.create_subscription(MarkerArray,cfg['topic'],lambda msg:capture('markers',msg),qos)
                for key in ['target_truth','target_estimate','target_prediction','robot_truth','robot_estimate']:
                    observer.create_subscription(PathMessage,cfg['paths_topic']+'/'+key,lambda msg,k=key:capture('path:'+k,msg),qos)
                observer.create_subscription(TFMessage,'/tf',lambda msg:capture('tf',msg),
                    QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
                def drain_display():
                    for _ in range(40):
                        previous=observed[0]
                        observer_executor.spin_once(timeout_sec=0.)
                        if observed[0]==previous:break
                command=['ros2','launch','robot_bringup','visualization.launch.py',
                         f'system_config:={prepared["system_path"]}',
                         'gui:='+str(args.gui).lower(),'rviz:='+str(args.gui).lower(),
                         'enable_actuation:='+str(not args.compute_only).lower()]
                result['launch_command']=command
                process=OwnedLaunch(command,lease.env,directory/'launch.log').start()
                try:
                    window=run_window(process,recorder,required_streams=['target_truth','joints','base_truth','prediction','control'],
                        duration=args.duration or (8. if not args.gui else 12.),settling_time=args.settling_time,readiness_timeout=60.,stall_timeout=15.,wall_timeout=150.,tick=drain_display)
                    if args.gui:
                        screenshot=subprocess.run(['python3',str(Path(__file__).with_name('m09_capture_gui.py')),
                                                   '--output',str(directory)],env=lease.env,capture_output=True,text=True,timeout=15.)
                        result['capture']={'returncode':screenshot.returncode,'stdout':screenshot.stdout,'stderr':screenshot.stderr[-1500:]}
                finally:
                    cleanup=process.stop(8.)
                result.update(window=window,cleanup=cleanup)
                all_markers=[m for a in received['markers'] for m in a['markers'] if m['action']==0]
                result['marker_namespaces']=sorted({m['ns'] for m in all_markers})
                result['max_markers_in_message']=max([len(a['markers']) for a in received['markers']],default=0)
                checks=result['checks']
                checks['marker_activity']=len(received['markers'])>2
                checks['stable_unique_ids']=all(len({(m['ns'],m['id']) for m in a['markers'] if m['action']==0})==sum(m['action']==0 for m in a['markers']) for a in received['markers'])
                checks['replacement_deletes']=all(a['markers'][0]['action']==3 for a in received['markers'])
                checks['marker_frames']=bool(all_markers) and all(m['header']['frame_id']=='world' for m in all_markers)
                checks['layers_present']=set(['camera_optical','line_of_sight','tracking_radius','joint_limits']).issubset(result['marker_namespaces'])
                result['paths']={}
                for key in ['target_truth','target_estimate','target_prediction','robot_truth','robot_estimate']:
                    messages=received['paths'].get(key,[]);nonempty=[m for m in messages if m['poses']]
                    result['paths'][key]={'messages':len(messages),'nonempty':len(nonempty),'max_points':max([len(m['poses']) for m in messages],default=0),
                     'frame_ids':sorted({m['header']['frame_id'] for m in messages})}
                    checks[key+'_activity']=bool(nonempty) if cfg['layers'][key] else bool(messages) and not nonempty
                    checks[key+'_bounded']=all(len(m['poses'])<=cfg['max_points'] for m in messages)
                    checks[key+'_source_epochs']=all(all(p['header']['frame_id']=='world' for p in m['poses']) and all(stamp(a['header'])<=stamp(b['header']) for a,b in zip(m['poses'],m['poses'][1:])) for m in messages)
                    if key=='target_prediction':
                        checks['prediction_future']=all(all(stamp(p['header'])>=stamp(m['header'])-1e-8 for p in m['poses']) for m in messages)
                transforms=[t for m in received['tf'] for t in m['transforms']]
                checks['physical_display_base_tf']=any(t['child_frame_id']=='visual/'+context['robot']['differential_drive']['child_frame_id'] and t['header']['frame_id']=='world' for t in transforms)
                result['display_dynamic_frames']=sorted({t['child_frame_id'] for t in transforms if t['child_frame_id'].startswith('visual/')})
                checks['robot_model_joint_tf']=len(result['display_dynamic_frames'])>=3
                checks['current_target_markers']={'target_truth_current','target_estimate_current'}.issubset(result['marker_namespaces'])
                result['commands_recorded']=len(recorder.streams['command'])
                checks['command_policy']=(not recorder.streams['command'] if args.compute_only
                                          else bool(recorder.streams['command']))
                model=create_robot_model(robot_model_parameters_from_description(context['description']))
                joint_index=SourceIndex(sorted(recorder.streams['joints'],key=lambda r:r['source_time']))
                base_index=SourceIndex(sorted(recorder.streams['base_truth'],key=lambda r:r['source_time']),'world')
                errors=[];axis_errors=[];radius_errors=[]
                for message in received['markers']:
                    markers={m['ns']:m for m in message['markers'] if m['action']==0}
                    if not {'camera_origin','camera_optical'}.issubset(markers):continue
                    origin=markers['camera_origin'];t=stamp(origin['header'])
                    try:
                        joint=joint_index.nearest(t,.08)['data'];base=base_index.nearest(t,.08)['data']
                        transform=np.eye(4);transform[:3,:3]=Rotation.from_quat(base['orientation']).as_matrix();transform[:3,3]=base['position']
                        q=[joint['position'][joint['names'].index(n)] for n in context['joint_names']]
                        physical=WholeBodyKinematics.compute(model,transform,q)
                        actual=np.array([origin['pose']['position'][k] for k in ['x','y','z']])
                        errors.append(float(np.linalg.norm(actual-physical[:3,3])))
                        arrow=markers['camera_optical']['points'];direction=np.array([arrow[1][k]-arrow[0][k] for k in ['x','y','z']]);expected=physical[:3,:3]@np.array(context['control']['tracking_forward_axis']);direction/=np.linalg.norm(direction);expected/=np.linalg.norm(expected)
                        axis_errors.append(float(np.arccos(np.clip(direction@expected,-1,1))))
                        if 'tracking_radius' in markers:
                            shell=markers['tracking_radius'];target=SourceIndex(sorted(recorder.streams['target_truth'],key=lambda r:r['source_time']),'world').nearest(stamp(shell['header']),.08)['data']['position']
                            points=np.array([[v[k] for k in ['x','y','z']] for v in shell['points']])
                            radius_errors.append(float(np.max(np.abs(np.linalg.norm(points-target,axis=1)-context['control']['tracking_radius']))))
                    except (ValueError,KeyError):continue
                result['geometry']={'samples':len(errors),'max_origin_difference_m':max(errors,default=None),'max_optical_difference_rad':max(axis_errors,default=None),'max_radius_error_m':max(radius_errors,default=None),'alignment_skew_s':.08}
                checks['physical_fk_agreement']=bool(errors) and max(errors)<.08
                checks['optical_agreement']=bool(axis_errors) and max(axis_errors)<.12
                checks['configured_radius']=bool(radius_errors) and max(radius_errors)<.04
                result['tracking_metrics']=evaluate(recorder.streams,context,window['window_start'],window['window_end'],.08)
                result['tracking_metrics'].pop('time_series')
                checks['cleanup']=not cleanup['survivors']
                write_json(directory/'visualization_messages.json',received)
                write_json(directory/'availability.json',recorder.availability)
            finally:
                if process is not None and cleanup is None:cleanup=process.stop(8.)
                if observer_executor is not None:observer_executor.shutdown(timeout_sec=1.)
                if observer is not None:observer.destroy_node()
                if recorder is not None:recorder.close()
                for key,value in old.items():
                    if value is None:os.environ.pop(key,None)
                    else:os.environ[key]=value
    except Exception as error:
        result['error']={'type':type(error).__name__,'message':str(error)}
        raise
    finally:
        write_json(directory/'result.json',result)
        print(json.dumps({'directory':str(directory),'checks':result['checks'],'error':result.get('error')}))
    if not all(result['checks'].values()):raise RuntimeError('M09 visualization checks failed')


if __name__=='__main__':main()
