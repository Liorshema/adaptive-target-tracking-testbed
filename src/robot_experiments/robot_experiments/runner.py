"""One owned deployment lifecycle per variant/seed through the existing M07 launch."""
import ast
import csv
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import psutil
import yaml

from .benchmarking import summarize
from .configuration import materialize_run
from .execution import DomainLease, OwnedLaunch, RunFailure, run_window
from .metrics import evaluate
from .reporting import create_study_directory, plain, write_json


def component_identities(log, expected):
    """Require adapter startup logs, not merely preflight's intended selections."""
    actual={}
    for line in log.splitlines():
        for token in ['Prediction node started: ','Control components: ']:
            if token in line:
                actual.update(ast.literal_eval(line.split(token,1)[1]))
        if 'Estimation node initialized: robot=' in line:
            tail=line.split('Estimation node initialized: robot=',1)[1]
            robot,target=tail.rstrip('.').split(', target=')
            actual.update(robot_estimator=robot,target_estimator=target)
        if 'Simulation components: dynamics=' in line:
            dynamics,integrator=line.split('Simulation components: dynamics=',1)[1].split(', integrator=')
            actual.update(dynamics=dynamics,integrator=integrator)
    problems=[]
    for name,wanted in expected.items():
        value=actual.get(name)
        if value not in (wanted,wanted.rsplit('.',1)[-1]):
            problems.append(f'{name}: expected {wanted}, observed {value}')
    return actual,problems


def _git():
    def git(*args):
        try:return subprocess.check_output(['git',*args],stderr=subprocess.DEVNULL,text=True).strip()
        except (OSError,subprocess.CalledProcessError):return None
    return {'revision':git('rev-parse','HEAD'),'branch':git('branch','--show-current'),
      'status':git('status','--short'),'tracked_diff_sha256':hashlib.sha256((git('diff','HEAD') or '').encode()).hexdigest()}


def _versions(share):
    packages={}
    for name in ['robot_experiments','robot_configuration','robot_description','robot_interfaces','robot_models',
                 'robot_estimation','robot_prediction','robot_control','robot_simulation','robot_bringup']:
        try:
            import xml.etree.ElementTree as ET
            path=share(name)/'package.xml';packages[name]=ET.parse(path).getroot().findtext('version')
        except (OSError,ValueError):packages[name]=None
    return packages


def _acceptance(metrics, enabled):
    safety=metrics['safety']['accepted']
    gates={'physical_joint_limits':safety}
    for key in ['invalid_commands','watchdog_events','rejected_commands','feedback_rejections']:
        item=metrics['metrics'][key]
        gates[key]=item.get('count',item.get('events'))==0 if item['available'] else None
    # These are availability gates, not invented tracking-score acceptance thresholds.
    for key in ['tracking_distance','pointing_error','prediction_rmse','target_position_error']:
        gates[key+'_available']=metrics['metrics'][key]['available']
    if not enabled:gates['physical_actuation_demonstrated']=None
    value=False if any(v is False for v in gates.values()) else (True if all(v is True for v in gates.values()) else None)
    return value,gates


def required_streams(plan, enabled):
    """Await only streams whose selected deployment actually publishes them."""
    required=['clock','target_truth','target_measurement','joints','base_truth',
              'robot_estimate','target_estimate','prediction','control']
    if plan['runtime']['publish_commands']:required.append('command')
    if enabled:required.append('actuation')
    return required


def run_one(spec,variant,seed,directory,share):
    from .recording import Recorder
    directory.mkdir()
    started=datetime.now(timezone.utc).isoformat()
    write_json(directory/'status.json',{'schema_version':1,'execution_status':'preparing','acceptance':None,'started_utc':started})
    process=recorder=None;window=None;cleanup=None;error=None;metrics=None;actual={};availability={};events=[]
    metadata={'schema_version':1,'variant':variant['name'],'seed':seed,'started_utc':started,
              'git':_git(),'package_versions':_versions(share),
              'time_contract':{'duration':'simulation seconds after settling','source_time':'ROS header epoch',
              'timeouts':'host monotonic seconds','wall_timestamps':'UTC ISO8601'},
              'environment':{k:os.environ.get(k) for k in ['ROS_DISTRO','RMW_IMPLEMENTATION','WSL_DISTRO_NAME','ROS_VERSION']},
              'platform':{'system':os.uname().sysname,'release':os.uname().release,'cpu_count':os.cpu_count()},
              'artifacts':{'metrics':'metrics.json','time_series':'time_series.json','telemetry':'telemetry','log':'launch.log'}}
    comparison_contract=None
    try:
        prepared=materialize_run(variant,seed,directory,share,spec['runtime'])
        context=prepared['context'];plan=prepared['plan']
        context['readiness_max_age']=spec['execution']['max_skew']
        comparison_contract={**prepared['comparison_contract'],'duration':spec['duration'],'settling_time':spec['settling_time'],
          'max_skew':spec['execution']['max_skew'],'physical_tolerance':context['physical_numerical_tolerance'],
          'enable_actuation':spec['runtime']['enable_actuation']}
        metadata.update(expected_component_types=plan['identities'],seed_consumers=prepared['snapshot']['seed_consumers'],
                        reproducibility=prepared['snapshot']['reproducibility'])
        write_json(directory/'resolved_configuration.json',{'experiment':{k:v for k,v in spec.items() if k!='variants'},
          'system':plan,'context':context,'owner_snapshot':prepared['snapshot'],'comparison_contract':comparison_contract})
        with DomainLease(log_dir=directory/'ros-log') as lease:
            metadata['isolation']={'ros_domain_id':lease.domain_id,'gazebo_partition':lease.partition}
            # rclpy owns a private Context, but its domain/log configuration comes from the environment.
            old={k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','ROS_LOG_DIR']}
            try:
                os.environ.update({k:lease.env[k] for k in old})
                recorder=Recorder(context,directory/'telemetry')
                # Refuse a domain already publishing a source stream before owning a launch.
                probe_deadline=time.monotonic()+.35
                while time.monotonic()<probe_deadline:recorder.spin_once(.02)
                if any(recorder.streams.values()):raise RunFailure('domain_already_active')
                command=['ros2','launch','robot_bringup','closed_loop.launch.py',f'system_config:={prepared["system_path"]}']
                metadata['launch_command']=command
                write_json(directory/'metadata.json',metadata)
                process=OwnedLaunch(command,lease.env,directory/'launch.log').start()
                required=required_streams(plan,spec['runtime']['enable_actuation'])
                performance=[];previous=[0.];cpu={}
                def tick():
                    now=time.monotonic()
                    if now-previous[0]<.5:return
                    previous[0]=now
                    children=psutil.Process(process.process.pid).children(recursive=True)
                    percent=0.;rss=0
                    for child in children:
                        try:
                            identity=(child.pid,child.create_time())
                            if identity not in cpu:cpu[identity]=child;child.cpu_percent()
                            else:percent+=cpu[identity].cpu_percent()
                            rss+=child.memory_info().rss
                            world=directory/'generated_world.sdf'
                            if not world.exists():
                                for argument in child.cmdline():
                                    candidate=Path(argument)
                                    if argument.startswith('/tmp/robot_simulation_') and candidate.name=='world.sdf' and candidate.is_file():
                                        shutil.copyfile(candidate,world)
                        except (psutil.Error,OSError):continue
                    performance.append({'clock':recorder.clock,'arrival_monotonic':now,'cpu_percent_sum':percent,'rss_bytes_sum':rss})
                try:
                    window=run_window(process,recorder,required_streams=required,duration=spec['duration'],settling_time=spec['settling_time'],
                      readiness_timeout=spec['execution']['readiness_timeout'],stall_timeout=spec['execution']['stall_timeout'],
                      wall_timeout=spec['execution']['wall_timeout'],tick=tick)
                finally:
                    cleanup=process.stop(spec['execution']['shutdown_timeout'])
                    write_json(directory/'host_performance.json',{'schema_version':1,'cpu_unit':'percent of one logical CPU; sum over owned descendants',
                      'memory_unit':'bytes; summed RSS includes shared pages','samples':performance})
                actual,problems=component_identities((directory/'launch.log').read_text(),plan['identities'])
                if problems:raise RunFailure('component_identity_mismatch',{'problems':problems})
                if cleanup['survivors']:raise RunFailure('cleanup_failed',cleanup)
                metrics=evaluate(recorder.streams,context,window['window_start'],window['window_end'],spec['execution']['max_skew'])
            finally:
                if process is not None and cleanup is None:cleanup=process.stop(spec['execution']['shutdown_timeout'])
                if recorder is not None:
                    availability=recorder.availability;events=recorder.events
                    recorder.close()
                for k,value in old.items():
                    if value is None:os.environ.pop(k,None)
                    else:os.environ[k]=value
    except (Exception,KeyboardInterrupt) as exc:
        error={'type':type(exc).__name__,'code':'interrupted' if isinstance(exc,KeyboardInterrupt) else getattr(exc,'code','exception'),'message':str(exc),'details':getattr(exc,'details',{})}
    accepted,gates=_acceptance(metrics,spec['runtime']['enable_actuation']) if metrics else (None,{})
    if metrics:
        time_series=metrics.pop('time_series')
        write_json(directory/'time_series.json',{'schema_version':1,'series':time_series})
        write_json(directory/'metrics.json',metrics)
    else:write_json(directory/'metrics.json',{'schema_version':1,'available':False,'reason':error})
    metadata.update(actual_component_types=actual,window=window,available_streams=availability)
    write_json(directory/'metadata.json',metadata)
    write_json(directory/'events.json',{'schema_version':1,'events':events,'failure':error})
    status={'schema_version':1,'execution_status':'failed' if error else 'completed','acceptance':accepted,
      'acceptance_gates':gates,'safety_status':metrics['safety'] if metrics else {'accepted':None,'reason':'evaluation unavailable'},
      'started_utc':started,'ended_utc':datetime.now(timezone.utc).isoformat(),'window':window,'error':error,'cleanup':cleanup}
    write_json(directory/'status.json',status)
    return {'variant':variant['name'],'seed':seed,'execution_status':status['execution_status'],'acceptance':accepted,
       'comparison_contract':hashlib.sha256(json.dumps(plain(comparison_contract),sort_keys=True).encode()).hexdigest(),
       'metrics':metrics or {},'artifacts':str(Path('runs')/directory.name),'status':status}


def run_experiment(spec,share):
    directory=create_study_directory(spec['output']['directory'],spec['name'])
    shutil.copyfile(spec['source'],directory/'experiment.yaml')
    write_json(directory/'resolved_experiment.json',spec)
    (directory/'runs').mkdir()
    runs=[]
    for seed in spec['seeds']:
        for variant in spec['variants']:
            run=run_one(spec,variant,seed,directory/'runs'/f'{variant["name"]}_seed{seed}',share)
            runs.append(run)
            summary=summarize(runs,[v['name'] for v in spec['variants']],spec['seeds'])
            summary['runs']=runs;write_json(directory/'summary.json',summary)
            interrupted=(run.get('status',{}).get('error') or {}).get('type')=='KeyboardInterrupt'
            if interrupted or (run['execution_status']=='failed' and spec['execution']['failure_policy']=='abort'):break
        else:continue
        break
    with (directory/'comparison.csv').open('w',newline='') as output:
        writer=csv.writer(output);writer.writerow(['variant','seed','execution_status','acceptance','metric','unit','mean','max','sample_count'])
        for run in runs:
            for name,metric in run['metrics'].get('metrics',{}).items():
                writer.writerow([run['variant'],run['seed'],run['execution_status'],run['acceptance'],name,metric.get('unit'),metric.get('mean'),metric.get('max'),metric.get('sample_count')])
    failed=any(r['execution_status']!='completed' for r in runs)
    acceptance=all(r['acceptance'] is True for r in runs)
    return directory,1 if failed else 0 if acceptance else 3
