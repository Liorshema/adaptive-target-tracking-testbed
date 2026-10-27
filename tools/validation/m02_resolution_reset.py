"""Bounded real-world-reset probe; record both native actuator epochs intact."""

from robot_experiments.catalog import ExperimentCatalog
import argparse
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.configuration import materialize_run
from robot_experiments.execution import DomainLease, OwnedLaunch
from robot_experiments.metrics.actuator_trace import analyze_trace
from robot_experiments.recording import Recorder
from robot_experiments.reporting import create_study_directory, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--system-config',type=Path,default=None,
                        help='Optional installed bringup manifest; default nominal is preserved')
    args = parser.parse_args()
    directory = create_study_directory(args.output,'reset')
    share = lambda p: Path(get_package_share_directory(p))
    spec = ExperimentCatalog(share=share).prepare('EXP-NOMINAL')
    if args.system_config is not None:
        from robot_bringup.factory.system_configuration import load_system_configuration
        system = args.system_config
        if not system.is_absolute():
            system = share('robot_bringup')/system
        from robot_experiments.configuration import resolve_system_resource
        system = resolve_system_resource(system, share('robot_bringup'), share)
        spec['variants'][0]['plan'] = load_system_configuration(system,share,overrides={
            'enable_actuation':'true','publish_ground_truth':'true','gui':'false',
            'visualize_tracking':'false'})
    prepared = materialize_run(spec['variants'][0],42,directory,share,spec['runtime'])
    context = prepared['context']
    simpath = directory/'inputs/simulation.yaml'
    sim = load_yaml(simpath)
    hold = load_yaml(sim['robot']['joint_position_hold'])
    hold['joint_position_hold']['trace_directory'] = str(directory/'physics')
    holdpath = directory/'inputs/holding.yaml'
    holdpath.write_text(yaml.safe_dump(hold))
    sim['robot']['joint_position_hold'] = str(holdpath)
    simpath.write_text(yaml.safe_dump(sim))
    world = ET.parse(sim['world']['source']).getroot().find('world').get('name')
    result = {'directory':str(directory),'checks':{}}
    with DomainLease(log_dir=directory/'ros-log') as lease:
        old = {k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','ROS_LOG_DIR']}
        process = recorder = None
        try:
            os.environ.update({k:lease.env[k] for k in old})
            recorder = Recorder(context,directory/'telemetry')
            process = OwnedLaunch(['ros2','launch','robot_bringup','closed_loop.launch.py',
                f'system_config:={prepared["system_path"]}'],lease.env,directory/'launch.log').start()
            def until(condition):
                deadline = time.monotonic()+45.
                while not condition():
                    if time.monotonic()>deadline or process.poll() is not None:
                        raise RuntimeError('Reset probe deadline or process failure')
                    recorder.spin_once(.02)
            until(lambda:recorder.clock is not None and recorder.clock>=4.)
            result['pre_reset_clock'] = recorder.clock
            service = subprocess.run(['gz','service','-s',f'/world/{world}/control',
                '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
                '--timeout','3000','--req','reset: { all: true }'],env=lease.env,
                text=True,capture_output=True,timeout=6.)
            result['reset_service'] = {'returncode':service.returncode,'stdout':service.stdout,
                                       'stderr':service.stderr}
            until(lambda:recorder.clock_reset)
            reset_control_index = len(recorder.streams['control'])
            until(lambda:recorder.clock is not None and recorder.clock>=3.)
            result['post_reset_clock'] = recorder.clock
            control = recorder.streams['control']
            result['checks']['control_recovered'] = any(
                r['source_time']>1. and r['source_time']<3. and r['data'].get('valid')
                for r in control[reset_control_index:] if r.get('source_time') is not None)
            result['checks']['clock_reset_observed'] = recorder.clock_reset
        finally:
            if process:result['cleanup'] = process.stop(5.)
            if recorder:recorder.close()
            for k,value in old.items():
                if value is None:os.environ.pop(k,None)
                else:os.environ[k] = value
    result['epochs'] = [analyze_trace(p,context,1.5)
                        for p in sorted((directory/'physics').glob('actuator*.csv'))]
    result['checks']['both_epochs_complete_and_safe'] = (
        len(result['epochs'])>=2 and all(r['accepted'] for r in result['epochs']))
    result['checks']['cleanup'] = not result['cleanup']['survivors']
    write_json(directory/'result.json',result)
    print(directory, result['checks'],flush=True)


if __name__ == '__main__':main()
