"""Longer unchanged C/D/E runs using the existing M08/M02 trial lifecycle."""
import argparse
import hashlib
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from robot_experiments.configuration import load_experiment
from robot_simulation.gazebo.physics_engine import physics_engine_library
from robot_experiments.reporting import create_study_directory, write_json

from m02_resolution_run import trial
from joint_limit_recovery_analysis import control_continuity, stalled_windows, telemetry_summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--variants',nargs='+',default=['C','D','E'])
    parser.add_argument('--seeds',nargs='+',type=int,default=[42,917])
    parser.add_argument('--duration',type=float,default=90.)
    args=parser.parse_args()
    share=lambda name:Path(get_package_share_directory(name))
    spec=load_experiment(share('robot_experiments')/'config/reactive_manipulation.yaml',share)
    study=create_study_directory(args.output,'challenge_recovery');print(study,flush=True)
    results=[]
    for variant in spec['variants']:
        if variant['name'] not in args.variants:continue
        for seed in args.seeds:
            directory=study/f'{variant["name"]}_seed{seed}'
            result=trial(spec,variant,seed,'velocity_servo',directory,share,args.duration,0.)
            binary=Path(physics_engine_library('dartsim_recovery'))
            result['physics_engine']={'path':str(binary),'sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
            trace=directory/'physics/actuator.csv'
            if trace.exists():result['deadlock_analysis']=stalled_windows(trace)
            if (directory/'telemetry/actuation.jsonl').exists():
                window=result.get('window',{})
                result['recovery_analysis']=telemetry_summary(directory/'telemetry',
                    window.get('window_start'),window.get('window_end'))
            if 'window' in result:
                window=result['window']
                result['control_continuity']=control_continuity(directory/'telemetry/control.jsonl',
                    window['window_start'],window['window_end'])
            write_json(directory/'recovery_result.json',result)
            results.append(result);write_json(study/'results.json',results)
            print(directory.name,'error',result.get('error'),'physical',result.get('physical',{}).get('accepted'),
                  'max_stall',result.get('deadlock_analysis',{}).get('max_contiguous_stall_s'),flush=True)
    # Completion requires inspectable physical/mobility evidence, not process exit alone.
    if any(r.get('error') or not r.get('physical',{}).get('accepted') or
           r.get('deadlock_analysis',{}).get('windows_over_one_second') or
           not r.get('control_continuity',{}).get('valid_samples') or
           r.get('control_continuity',{}).get('max_invalid_duration_s',float('inf'))>=1. or
           r.get('control_continuity',{}).get('max_sample_gap_s',float('inf'))>=1.
           for r in results):
        raise SystemExit(1)


if __name__=='__main__':main()
