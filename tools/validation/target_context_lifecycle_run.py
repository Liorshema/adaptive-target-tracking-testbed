"""Inspect real launch ownership; qualify repeated root/group SIGINT shutdowns."""

from robot_experiments.catalog import ExperimentCatalog
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import psutil
from ament_index_python.packages import get_package_share_directory
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_experiments.configuration import materialize_run, resolve_system_resource
from robot_experiments.execution import DomainLease, OwnedLaunch, run_window
from robot_experiments.recording import Recorder
from robot_experiments.reporting import create_study_directory, write_json


def identities(process):
    result = []
    for p in process._snapshot(scan_marker=True, scan_session=True):
        try:
            result.append({'pid': p.pid, 'created': p.create_time(), 'ppid': p.ppid(),
                'session': os.getsid(p.pid), 'command': p.cmdline(),
                'marker': p.environ().get('_ROBOT_EXPERIMENTS_OWNER')})
        except psutil.Error:
            pass
    return result


def transport():
    result = {}
    for name, command in [('services',['gz','service','-l']), ('clock',['gz','topic','-i','-t','/clock'])]:
        try:
            r=subprocess.run(command,text=True,capture_output=True,timeout=4.)
            result[name]={'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr}
        except subprocess.TimeoutExpired:
            result[name]={'unavailable':'discovery timeout; not proof of absence'}
    return result


def graph(recorder):
    node = recorder._node
    return {'clock': [{'node': p.node_name, 'namespace': p.node_namespace,
        'gid': list(p.endpoint_gid)} for p in node.get_publishers_info_by_topic('/clock')],
        'services': node.get_service_names_and_types()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--duration', type=float, default=6.)
    parser.add_argument('--failure', choices=['none', 'server', 'bridge', 'bad_config', 'context_delay', 'context_gap'], default='none')
    args = parser.parse_args()
    share = lambda p: Path(get_package_share_directory(p))
    spec = ExperimentCatalog(share=share).prepare('EXP-MANIPULATION')
    variant = next(v for v in spec['variants'] if v['name'] == 'D')
    system = resolve_system_resource(
        {'package': 'robot_experiments', 'path': 'config/systems/mobile_manipulator_pose_tracking.yaml'},
        share('robot_experiments'), share)
    variant['plan'] = load_system_configuration(system, share,
        overrides={'enable_actuation': 'true', 'publish_ground_truth': 'true', 'gui': 'false', 'visualize_tracking': 'false'})
    study = create_study_directory(args.output, 'lifecycle')
    results = []
    # Reuse domain AND Gazebo partition: isolation between our own cycles must
    # come from actual shutdown, rather than obtaining another empty namespace.
    with DomainLease(log_dir=study/'ros-log') as lease:
        old = {k: os.environ.get(k) for k in ['ROS_DOMAIN_ID', 'GZ_PARTITION', 'GZ_IP', 'ROS_LOG_DIR']}
        os.environ.update({k: lease.env[k] for k in old})
        unrelated = subprocess.Popen(['sleep', '600'])
        try:
            for index in range(args.cycles):
                directory = study/f'cycle_{index+1}'; directory.mkdir()
                prepared = materialize_run(variant, 42, directory, share, spec['runtime'])
                recorder = Recorder(prepared['context'], directory/'telemetry')
                command = ['ros2', 'launch', 'robot_bringup', 'closed_loop.launch.py',
                           f'system_config:={prepared["system_path"]}']
                if args.failure == 'bad_config': command[-1] = 'system_config:=/nonexistent/test.yaml'
                result = {'cycle': index+1, 'domain': lease.domain_id, 'partition': lease.partition,
                    'start_wall': datetime.now(timezone.utc).isoformat(), 'before': graph(recorder), 'command': command}
                process = OwnedLaunch(command, lease.env, directory/'launch.log').start()
                result['pid'] = process.process.pid
                try:
                    result['window'] = run_window(process, recorder,
                        required_streams=['clock','joints','base_truth','prediction','control','actuation'],
                        duration=args.duration, settling_time=0., readiness_timeout=45., stall_timeout=10., wall_timeout=70.)
                    result['during'] = graph(recorder)
                    if args.failure != 'none': result['gazebo_during'] = transport()
                    result['owned'] = identities(process)
                    result['first_clock'] = recorder.streams['clock'][0]['source_time']
                    result['last_clock'] = recorder.clock
                    result['valid_controls'] = sum(r['data']['valid'] for r in recorder.streams['control'])
                    if args.failure in ['context_delay','context_gap']:
                        victims=[p for p in result['owned'] if any('/simulation_node' in s for s in p['command'])]
                        if len(victims)!=1: raise RuntimeError('Target process not uniquely identified')
                        victim=psutil.Process(victims[0]['pid'])
                        assert victim.create_time()==victims[0]['created']
                        result['injected_failure']=victims[0]
                        result['context_outage_start']=recorder.clock
                        from robot_configuration.yaml_loader import load_yaml
                        deployment=load_yaml(directory/'inputs/simulation.yaml')
                        integration=load_yaml(deployment['simulation']['config'])['simulation']['integration']
                        interval=(.12 if args.failure=='context_delay' else
                            integration['parameters']['dt']*integration['max_steps']
                            +2*deployment['physical_context']['max_age'])
                        victim.send_signal(signal.SIGSTOP)
                        try:
                            deadline=time.monotonic()+45.
                            while recorder.clock < result['context_outage_start']+interval:
                                if time.monotonic()>deadline: raise RuntimeError('Outage clock deadline')
                                recorder.spin_once(.02)
                        finally:
                            victim.send_signal(signal.SIGCONT)
                        result['context_outage_end']=recorder.clock
                        deadline=time.monotonic()+40.
                        recovered=False
                        while time.monotonic()<deadline and process.poll() is None:
                            recorder.spin_once(.02)
                            target=recorder.streams['target_truth']
                            control=recorder.streams['control']
                            recovered=bool(target and target[-1]['source_time']>result['context_outage_end']
                                and control and control[-1]['data']['valid'])
                            if args.failure=='context_delay' and recovered: break
                        result['context_recovered']=recovered
                        result['post_outage_clock']=recorder.clock
                        result['terminal_fault_observed']='Irrecoverable target context/time gap' in (directory/'launch.log').read_text()
                        if args.failure=='context_delay' and not recovered: raise RuntimeError('Recoverable context did not resume')
                        if args.failure=='context_gap' and not result['terminal_fault_observed']: raise RuntimeError('Irrecoverable gap not surfaced')
                    if args.failure in ['server','bridge']:
                        victims = [p for p in result['owned'] if (
                            (any(text in ' '.join(p['command']) for text in ['gz sim server','gz sim -s '])) if args.failure == 'server' else
                            any('/parameter_bridge' in s for s in p['command']))]
                        if len(victims) != 1: raise RuntimeError(f'Cannot uniquely identify failure victim: {victims}')
                        result['injected_failure'] = victims[0]
                        victim = psutil.Process(victims[0]['pid'])
                        if victim.create_time() != victims[0]['created']: raise RuntimeError('PID changed')
                        victim.send_signal(signal.SIGTERM)
                        result['shutdown_method'] = 'owned essential child SIGTERM'
                    else:
                        # Alternate actual foreground-group Ctrl+C semantics and
                        # a graceful request delivered solely to launch itself.
                        grouped = index % 2 == 1
                        result['shutdown_method'] = 'foreground process-group SIGINT' if grouped else 'launch-root SIGINT'
                        if grouped:
                            assert os.getsid(process.process.pid) == process.process.pid
                            os.killpg(process.process.pid, signal.SIGINT)
                        elif process.poll() is None: process.process.send_signal(signal.SIGINT)
                    started = time.monotonic()
                    while time.monotonic()-started < 12. and process._snapshot():
                        recorder.spin_once(.03)
                    result['launch_shutdown_elapsed'] = time.monotonic()-started
                    result['before_runner_fallback'] = identities(process)
                    result['launch_returncode'] = process.poll()
                except Exception as error:
                    result['error'] = str(error)
                finally:
                    result['cleanup'] = process.stop(3.)
                    deadline = time.monotonic()+2.
                    while time.monotonic()<deadline: recorder.spin_once(.05)
                    result['after'] = graph(recorder)
                    if args.failure != 'none': result['gazebo_after'] = transport()
                    result['remaining_owned'] = identities(process)
                    result['unrelated_alive'] = unrelated.poll() is None
                    result['stop_wall'] = datetime.now(timezone.utc).isoformat()
                    result['availability'] = recorder.availability
                    result['accepted'] = (not result.get('error') and result.get('valid_controls',0)>0
                        and not result.get('before_runner_fallback') and not result['remaining_owned']
                        and not result['after']['clock'] and result['unrelated_alive']
                        and len(result.get('during',{}).get('clock',[])) == 1)
                    if args.failure == 'bad_config': result['expected_failure'] = result.get('error') is not None
                    recorder.close()
                    write_json(directory/'result.json', result)
                    results.append(result); write_json(study/'results.json', results)
                    print(directory, 'accepted=',result['accepted'], 'fallback=',result['cleanup']['signals'], flush=True)
        finally:
            unrelated.terminate(); unrelated.wait(timeout=3.)
            for k,v in old.items():
                if v is None: os.environ.pop(k,None)
                else: os.environ[k]=v
    print(study, flush=True)


if __name__ == '__main__': main()
