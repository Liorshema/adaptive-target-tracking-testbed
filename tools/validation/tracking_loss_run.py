"""Matched diagnostic interventions using the existing M02/M08 trial unchanged."""

from robot_experiments.catalog import ExperimentCatalog
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.configuration import materialize_run
from robot_experiments.execution import OwnedLaunch
from robot_experiments.reporting import create_study_directory, write_json
from joint_limit_recovery_analysis import control_continuity, stalled_windows
import m02_resolution_run as runner

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--config',type=Path,default=Path(__file__).with_name('tracking_loss_experiments.yaml'));parser.add_argument('--modes',nargs='+');a=parser.parse_args()
    cfg=load_yaml(a.config);assert cfg['diagnostic_only'] is True
    share=lambda name:Path(get_package_share_directory(name));spec=ExperimentCatalog(share=share).prepare('EXP-MANIPULATION')
    variant=next(v for v in spec['variants'] if v['name']==cfg['variant']);study=create_study_directory(a.output,'tracking_loss');print(study,flush=True)
    results=[]
    for mode,selection in cfg['modes'].items():
        if a.modes and mode not in a.modes:continue
        directory=study/mode
        actual_command=[]
        def prepare(*args,**kwargs):
            prepared=materialize_run(*args,**kwargs);context=prepared['context'];original_context=deepcopy(context)
            if selection['robot_truth']:
                context['topics']['robot_estimate']=cfg['robot_output']
            if selection['target_truth']:
                context['topics']['prediction']=cfg['prediction_output']
            pred=load_yaml(prepared['plan']['prediction_path'])['prediction'];spawn=context['robot']['pose']
            diagnostic={**cfg,'mode':selection,'context':original_context,'spawn':spawn,'spawn_quaternion':Rotation.from_euler('xyz',spawn[3:]).as_quat().tolist(),'horizon':pred['horizon'],'dt':pred['dt'],'log':str((directory/'oracle.jsonl').resolve())}
            (directory/'inputs/diagnostic.yaml').write_text(yaml.safe_dump(diagnostic))
            return prepared
        def launch(command,env,log):
            if selection['robot_truth'] or selection['target_truth']:
                command=['ros2','launch',str(Path(__file__).with_name('tracking_loss.launch.py').resolve()),*command[4:],f'diagnostic_config:={(directory/"inputs/diagnostic.yaml").resolve()}']
                if selection['target_truth']:command.append(f'lookahead:={cfg["target_oracle_lookahead"]}')
            actual_command.extend(command)
            return OwnedLaunch(command,env,log)
        with patch.object(runner,'materialize_run',prepare),patch.object(runner,'OwnedLaunch',launch):
            result=runner.trial(spec,variant,cfg['seed'],'velocity_servo',directory,share,cfg['duration'],0.)
        result['actual_command']=actual_command
        result['intervention']={'name':mode,'selection':selection,'target_lookahead':cfg['target_oracle_lookahead'] if selection['target_truth'] else .25,'diagnostic_config_sha256':hashlib.sha256((directory/'inputs/diagnostic.yaml').read_bytes()).hexdigest()}
        if 'window' in result:
            w=result['window'];result['control_continuity']=control_continuity(directory/'telemetry/control.jsonl',w['window_start'],w['window_end'])
        if (directory/'physics/actuator.csv').exists():result['deadlocks']=stalled_windows(directory/'physics/actuator.csv')
        write_json(directory/'diagnostic_result.json',result);results.append(result);write_json(study/'results.json',results)
        print(mode,'error',result.get('error'),'physical',result.get('physical',{}).get('accepted'),'control',result.get('control_continuity'),flush=True)
    if any(r.get('error') or not r.get('physical',{}).get('accepted') or
           not r.get('control_continuity',{}).get('valid_samples') or
           r.get('control_continuity',{}).get('max_invalid_duration_s',float('inf'))>=1. or
           r.get('control_continuity',{}).get('max_sample_gap_s',float('inf'))>=1.
           for r in results):raise SystemExit(1)
if __name__=='__main__':main()
