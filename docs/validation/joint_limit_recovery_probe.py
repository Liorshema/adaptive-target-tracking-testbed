"""Bounded real-native boundary probe using normal simulation launch/M08 isolation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import psutil
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64

from robot_configuration.yaml_loader import load_yaml
from robot_experiments.configuration import load_experiment, materialize_run
from robot_experiments.execution import DomainLease, OwnedLaunch
from robot_experiments.recording import Recorder
from robot_experiments.reporting import create_study_directory, write_json
from robot_simulation.gazebo.robot_builder import actuator_library
from robot_simulation.gazebo.physics_engine import physics_engine_library
from robot_experiments.metrics.actuator_trace import analyze_trace


def trial(variant, spec, directory, share, joint_key, side, backend, policy, engine=None):
    directory.mkdir()
    prepared=materialize_run(variant,42,directory,share,spec['runtime'],record_actuator_trace=True)
    context=prepared['context'];sim=load_yaml(prepared['simulation_path'])
    if engine is not None:
        sim['world']['physics']['engine'] = engine
    # Isolate arm physics from target motion/controller: retain the actual robot,
    # selected limits, native holding, world and physical timestep.
    defaults=load_yaml(share('robot_simulation')/'config/launch.yaml')
    sim['tracking']=defaults['tracking'];sim.pop('physical_context',None)
    sim['target']['config']=str(share('robot_simulation')/'config/scenarios/target.yaml')
    sim['simulation']['environment_config']=str(share('robot_simulation')/'config/scenarios/environment.yaml')
    holding_path=Path(sim['robot']['joint_position_hold']);holding=load_yaml(holding_path)
    holding['joint_position_hold']['backend']=backend
    holding_path.write_text(yaml.safe_dump(holding,sort_keys=False))
    prepared['simulation_path'].write_text(yaml.safe_dump(sim,sort_keys=False))
    physical=context['description']['robot']['manipulator']['joints']
    names=context['joint_names'];keys=list(physical);index=keys.index(joint_key)
    limit=physical[joint_key][side]
    inset=load_yaml(prepared['plan']['profile']['actuation_config'])['actuation']['position_recovery']['recovery_inset']
    inward=limit+(inset if side=='lower' else -inset)
    binary=Path(actuator_library())
    result={'variant':variant['name'],'joint_key':joint_key,'joint_name':names[index],
            'side':side,'lower':physical[joint_key]['lower'],'upper':physical[joint_key]['upper'],
            'engine':sim['world']['physics'].get('engine','dartsim'),'backend':backend,'inward_reference':inward,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
            'context':context,'policy':policy,'samples':[],'phases':{}}
    if result['engine'] == 'dartsim_recovery':
        library = Path(physics_engine_library(result['engine']))
        result['physics_engine'] = {'path':str(library),'sha256':hashlib.sha256(library.read_bytes()).hexdigest()}
    write_json(directory/'identity.json',{'prepared':prepared,'result':result})
    process=recorder=driver=None
    with DomainLease(log_dir=directory/'ros-log') as lease:
        saved={k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','ROS_LOG_DIR']}
        try:
            os.environ.update({k:lease.env[k] for k in saved})
            recorder=Recorder(context,directory/'telemetry')
            driver=Node('native_boundary_probe',context=recorder._ros_context)
            robot=context['robot']
            publishers=[driver.create_publisher(Float64,robot['joint_position_topic_template'].format(
                model=robot['entity_name'],joint=name),10) for name in names]
            targets=[holding['joint_position_hold']['positions'][key] for key in keys]
            command=['ros2','launch','robot_simulation','simulation.launch.py',
                     f'config:={prepared["simulation_path"]}','gui:=false','enable_actuation:=true',
                     'publish_ground_truth:=true']
            process=OwnedLaunch(command,lease.env,directory/'launch.log').start()
            result.update(command=command,partition=lease.partition,domain=lease.domain_id)
            wall_start=time.monotonic();last_publish=0.
            def pump():
                nonlocal last_publish
                if process.poll() is not None or time.monotonic()-wall_start>policy['max_wall_seconds']:
                    raise RuntimeError('native probe process/wall deadline')
                if time.monotonic()-last_publish>=policy['publish_period_seconds']:
                    for publisher,target in zip(publishers,targets):publisher.publish(Float64(data=float(target)))
                    last_publish=time.monotonic()
                recorder.spin_once(.01)
            while recorder.clock is None or not recorder.streams['joints']:pump()
            def phase(label,duration):
                start=recorder.clock
                while recorder.clock<start+duration:pump()
                data=recorder.streams['joints'][-1]['data']
                qi=data['names'].index(names[index]);q=data['position'][qi];v=data['velocity'][qi]
                result['phases'][label]={'start':start,'end':recorder.clock,'q':q,'v':v,'reference':targets[index]}
            phase('settle',policy['settle_seconds'])
            targets[index]=limit
            phase('seek',policy['seek_seconds'])
            phase('boundary_dwell',policy['boundary_dwell_seconds'])
            targets[index]=inward
            phase('inward',policy['release_seconds'])
            result['loaded_physics_libraries'] = {}
            for child in psutil.Process(process.process.pid).children(recursive=True):
                try:
                    paths = sorted({line.split()[-1] for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines()
                                    if ('dartsim' in line or 'libdart.so' in line)})
                    if paths: result['loaded_physics_libraries'][str(child.pid)] = paths
                except (OSError, psutil.Error): pass
            before=result['phases']['boundary_dwell']['q'];after=result['phases']['inward']['q']
            displacement=(after-before)*(1 if side=='lower' else -1)
            result['inward_displacement']=displacement
            release=load_yaml(prepared['plan']['profile']['actuation_config'])['actuation']['position_recovery']['release_margin']
            tolerance=load_yaml(prepared['plan']['profile']['actuation_config'])['actuation']['position_recovery']['numerical_tolerance']
            result['boundary_reached']=bool(abs(before-limit)<=tolerance)
            result['physical_release_demonstrated']=bool(result['boundary_reached'] and displacement>release)
        except Exception as error:result['error']=f'{type(error).__name__}: {error}'
        finally:
            if process:result['cleanup']=process.stop(5.)
            if driver:driver.destroy_node()
            if recorder:recorder.close()
            for k,v in saved.items():
                if v is None:os.environ.pop(k,None)
                else:os.environ[k]=v
    trace = directory/'physics/actuator.csv'
    if trace.exists():
        result['physical'] = analyze_trace(trace, context, policy['settle_seconds'])
    write_json(directory/'result.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--variants',nargs='+',default=['C'])
    parser.add_argument('--joints',nargs='+',default=['wrist'])
    parser.add_argument('--sides',nargs='+',choices=['lower','upper'],default=['lower','upper'])
    parser.add_argument('--backend',default='velocity_servo',choices=['velocity_servo','instrumented_pid'])
    parser.add_argument('--engine',choices=['dartsim','dartsim_recovery'],default=None)
    parser.add_argument('--starts',type=int,default=1)
    args=parser.parse_args()
    share=lambda name:Path(get_package_share_directory(name))
    spec=load_experiment(share('robot_experiments')/'config/reactive_manipulation.yaml',share)
    policy=load_yaml(Path(__file__).with_suffix('.yaml'))
    study=create_study_directory(args.output,'native_boundary')
    print(study,flush=True)
    results=[]
    selected_variants=[v for v in spec['variants'] if v['name'] in args.variants]
    for start in range(args.starts):
      for variant in selected_variants:
        for joint in args.joints:
            for side in args.sides:
                result=trial(variant,spec,study/f'{variant["name"]}_{joint}_{side}_start{start+1}',share,joint,side,args.backend,policy,args.engine)
                results.append(result);write_json(study/'results.json',results)
                print(variant['name'],joint,side,result.get('physical_release_demonstrated'),result.get('error'),flush=True)
    if any(not r.get('physical_release_demonstrated') or not r.get('physical',{}).get('accepted') for r in results):
        raise SystemExit(1)


if __name__=='__main__':main()
