"""Bounded matched actuator trials using M08 lifecycle and immutable input snapshots."""

from robot_experiments.catalog import ExperimentCatalog
import argparse
import hashlib
import os
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from robot_configuration.yaml_loader import load_yaml
from robot_experiments.configuration import materialize_run
from robot_experiments.execution import DomainLease, OwnedLaunch, run_window
from robot_experiments.metrics import evaluate
from robot_experiments.metrics.actuator_trace import analyze_trace
from robot_experiments.recording import Recorder
from robot_experiments.reporting import create_study_directory, write_json
from robot_simulation.gazebo.robot_builder import actuator_library


def trial(spec, variant, seed, backend, directory, share, duration, settling):
    directory.mkdir()
    prepared = materialize_run(variant, seed, directory, share, spec['runtime'])
    context = prepared['context']
    simulation_path = directory/'inputs/simulation.yaml'
    simulation = load_yaml(simulation_path)
    hold = load_yaml(simulation['robot']['joint_position_hold'])
    hold['joint_position_hold'].update(backend=backend, trace_directory=str(directory/'physics'))
    holding_path = directory/'inputs/holding.yaml'
    holding_path.write_text(yaml.safe_dump(hold, sort_keys=False))
    simulation['robot']['joint_position_hold'] = str(holding_path)
    simulation_path.write_text(yaml.safe_dump(simulation, sort_keys=False))
    binary = Path(actuator_library())
    write_json(directory/'resolved_configuration.json', {'context': context,
        'owner_snapshot': prepared['snapshot'], 'holding': hold, 'backend': backend, 'seed': seed,
        'plugin': {'path': str(binary), 'sha256': hashlib.sha256(binary.read_bytes()).hexdigest()},
        'inputs': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in (directory/'inputs').glob('*') if p.is_file()}})
    result = {'backend': backend, 'predictor': variant['name'], 'seed': seed,
              'directory': str(directory)}
    process = recorder = None
    with DomainLease(log_dir=directory/'ros-log') as lease:
        old = {k: os.environ.get(k) for k in ['ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','ROS_LOG_DIR']}
        try:
            os.environ.update({k: lease.env[k] for k in old})
            recorder = Recorder(context, directory/'telemetry')
            command = ['ros2','launch','robot_bringup','closed_loop.launch.py',
                       f'system_config:={prepared["system_path"]}']
            result.update(command=command, ros_domain_id=lease.domain_id, partition=lease.partition)
            process = OwnedLaunch(command, lease.env, directory/'launch.log').start()
            result['window'] = run_window(process, recorder,
                required_streams=['clock','joints','base_truth','prediction','control','actuation'],
                duration=duration, settling_time=settling, readiness_timeout=60.,
                stall_timeout=15., wall_timeout=180.)
        except Exception as error:
            result['error'] = f'{type(error).__name__}: {error}'
        finally:
            if process:
                result['cleanup'] = process.stop(5.)
            if recorder:
                if 'window' in result:
                    w = result['window']
                    result['tracking'] = evaluate(recorder.streams, context,
                        w['window_start'], w['window_end'], .08)
                    result['tracking'].pop('time_series', None)
                    control = recorder.streams['control']
                    active = [r['source_time'] for r in control if r['data'].get('valid')]
                    result['first_valid_control_time'] = min(active) if active else w['window_start']
                write_json(directory/'availability.json', recorder.availability)
                recorder.close()
            for key,value in old.items():
                if value is None:
                    os.environ.pop(key,None)
                else:
                    os.environ[key] = value
    trace = directory/'physics/actuator.csv'
    if trace.exists():
        result['physical'] = analyze_trace(trace, context, result.get('first_valid_control_time', 0.))
    write_json(directory/'result.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--backends', nargs='+', default=['instrumented_pid','velocity_servo'])
    parser.add_argument('--predictors', nargs='+', default=['cv','imm'])
    parser.add_argument('--seeds', nargs='+', type=int, default=[42,917])
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--settling', type=float, default=2.)
    args = parser.parse_args()
    share = lambda package: Path(get_package_share_directory(package))
    spec = ExperimentCatalog(share=share).prepare('EXP-PREDICTION')
    study = create_study_directory(args.output, 'm02_actuator')
    results = []
    for backend in args.backends:
        for predictor in args.predictors:
            variant = next(v for v in spec['variants'] if v['name'] == predictor)
            for seed in args.seeds:
                directory = study/f'{backend}_{predictor}_{seed}'
                result = trial(spec, variant, seed, backend, directory, share,
                               args.duration, args.settling)
                results.append(result)
                print(directory.name, 'error=', result.get('error'),
                      'physical=', result.get('physical',{}).get('accepted'), flush=True)
                write_json(study/'results.json',results)
    print(study, flush=True)


if __name__ == '__main__':
    main()
