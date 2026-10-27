"""Real adapter boundary recovery fixture; no direct motor or feedback writer.

Run in a sourced workspace. This launches normal simulation, then publishes only
SafeVelocityCommand through the real VelocityActuationNode. Adapter internals are
read solely for validation observations. Native physics stays in actuator.csv.
"""

from robot_experiments.catalog import ExperimentCatalog

import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import yaml

from functools import partial
from robot_experiments.composition import load_resource
from robot_experiments.configuration import resolve_system_resource
load_yaml = partial(load_resource, share=lambda package: Path(__file__).resolve().parents[2] / 'src' / package)
from robot_experiments.configuration import materialize_run
from robot_experiments.execution import DomainLease, OwnedLaunch
from robot_experiments.recording import Recorder
from robot_experiments.metrics.actuator_trace import analyze_trace
from robot_experiments.reporting import create_study_directory, write_json
from robot_interfaces.msg import SafeVelocityCommand
from robot_simulation.gazebo.physics_engine import physics_engine_library
from robot_simulation.gazebo.robot_builder import actuator_library
from robot_simulation.nodes.velocity_actuation import VelocityActuationNode


def at_boundary(q, lower, upper, side, tolerance):
    """Use the configured guard numerical band, rather than a widened limit."""
    return q <= lower + tolerance if side == 'lower' else q >= upper - tolerance


def beyond_release(q, lower, upper, side, margin):
    """Require absolute physical inward clearance from the authoritative stop."""
    return q >= lower + margin if side == 'lower' else q <= upper - margin


def adapter_arguments(prepared, simulation):
    """Use the same parameter owners as normal closed-loop launch."""
    context = prepared['context']
    parameters = {
        'use_sim_time': True,
        'enabled': True,
        'configuration_file': str(prepared['plan']['profile']['actuation_config']),
        'description_file': str(Path(simulation['robot']['config']).parent / 'robot_description.yaml'),
        'robot_configuration_file': str(simulation['robot']['config']),
        'holding_configuration_file': str(simulation['robot']['joint_position_hold']),
        'base_velocity_lower': context['control']['base_velocity_lower'],
        'base_velocity_upper': context['control']['base_velocity_upper'],
    }
    arguments = ['--ros-args']
    for name, value in parameters.items():
        # JSON is valid YAML for these scalar/array ROS parameter values.
        arguments.extend(['-p', f'{name}:={json.dumps(value)}'])
    return arguments


def prepare_trial(variant, spec, directory, share, seed, engine):
    """Reuse M08 materialization and the native probe's isolated normal world."""
    prepared = materialize_run(
        variant, seed, directory, share, spec['runtime'], record_actuator_trace=True)
    simulation = load_yaml(prepared['simulation_path'])
    defaults = load_resource({'package': 'robot_experiments', 'path': 'config/experiments.yaml',
                              'group': 'resources.deployment_standalone'}, share=share)
    simulation['tracking'] = defaults['tracking']
    simulation.pop('physical_context', None)
    simulation['target']['config'] = str(resolve_system_resource(
        {'package': 'robot_experiments', 'path': 'config/experiments.yaml',
         'group': 'resources.target_independent'}, directory, share))
    simulation['simulation']['environment_config'] = str(resolve_system_resource(
        {'package': 'robot_experiments', 'path': 'config/experiments.yaml',
         'group': 'resources.environment_reactive'}, directory, share))
    simulation['world'].setdefault('physics', {})['engine'] = engine
    prepared['simulation_path'].write_text(yaml.safe_dump(simulation, sort_keys=False))
    return prepared, simulation


def trial(variant, spec, directory, share, joint_key, side, seed, engine, policy):
    """Run bounded acquisition, recovery and resumed normal physical motion."""
    directory.mkdir()
    prepared, simulation = prepare_trial(variant, spec, directory, share, seed, engine)
    context = prepared['context']
    physical = context['description']['robot']['manipulator']['joints']
    index = list(physical).index(joint_key)
    names = context['joint_names']
    joint = physical[joint_key]
    lower, upper = joint['lower'], joint['upper']
    sign = 1. if side == 'lower' else -1.
    actuation = load_yaml(prepared['plan']['profile']['actuation_config'])['actuation']
    recovery = actuation['position_recovery']
    physics_binary = physics_engine_library(engine)
    motor_binary = Path(actuator_library())
    result = {
        'variant': variant['name'], 'seed': seed, 'joint_key': joint_key,
        'joint_name': names[index], 'side': side, 'lower': lower, 'upper': upper,
        'engine': engine, 'physics_library': physics_binary,
        'physics_sha256': (hashlib.sha256(Path(physics_binary).read_bytes()).hexdigest()
                           if Path(physics_binary).is_file() else None),
        'motor_sha256': hashlib.sha256(motor_binary.read_bytes()).hexdigest(),
        'policy': policy, 'recovery_policy': recovery, 'phases': {},
        'samples_file': str(directory / 'authority_samples.jsonl'),
        'physical_release_demonstrated': False, 'normal_motion_resumed': False,
        'serious_violation_physical_fixture': 'not injected; existing unit safety tests only',
    }
    write_json(directory / 'identity.json', {'prepared': prepared, 'result': result})
    process = recorder = driver = adapter = executor = None
    initialized = False
    environment_keys = ['ROS_DOMAIN_ID', 'GZ_PARTITION', 'GZ_IP', 'ROS_LOG_DIR']
    with DomainLease(log_dir=directory / 'ros-log') as lease:
        saved = {key: os.environ.get(key) for key in environment_keys}
        try:
            os.environ.update({key: lease.env[key] for key in environment_keys})
            # Recorder and publisher retain their own private ROS context. Only
            # the unchanged real adapter uses the default ROS context.
            recorder = Recorder(context, directory / 'telemetry')
            driver = Node('authority_boundary_driver', context=recorder._ros_context)
            publisher = driver.create_publisher(
                SafeVelocityCommand, actuation['command_topic'], 10)
            if rclpy.ok():
                raise RuntimeError('fixture requires an unused default ROS context')
            rclpy.init(args=adapter_arguments(prepared, simulation))
            initialized = True
            adapter = VelocityActuationNode()
            executor = SingleThreadedExecutor()
            executor.add_node(adapter)
            command = ['ros2', 'launch', 'robot_simulation', 'simulation.launch.py',
                       f'config:={prepared["simulation_path"]}', 'gui:=false',
                       'enable_actuation:=true', 'publish_ground_truth:=true']
            process = OwnedLaunch(command, lease.env, directory / 'launch.log').start()
            result.update(command=command, domain=lease.domain_id, partition=lease.partition)
            wall_start = time.monotonic()
            last_publish = 0.
            sequence = 0
            session = str(uuid.uuid4())
            velocities = np.zeros(2 + len(names))
            command_active = False
            stage = 'startup'
            last_feedback_stamp = None
            sample_count = 0
            seen_recovery = False
            fresh_recovery_samples = 0

            with (directory / 'authority_samples.jsonl').open('x', buffering=1) as sample_file:
                def snapshot():
                    measured = adapter._joint
                    return {
                        'stage': stage, 'source_time': measured[0] if measured else None,
                        'adapter_time': adapter._now(), 'joint_name': names[index],
                        'lower': lower, 'upper': upper,
                        'measured_position': float(measured[2][index]) if measured else None,
                        'measured_velocity': (float(adapter._joint_velocity[index])
                                              if adapter._joint_velocity is not None else None),
                        'position_reference': (float(adapter._reference.position[index])
                                               if adapter._reference.position is not None else None),
                        'guard_mode': adapter._position_guard.mode,
                        'recovery_authority_fresh': bool(adapter._fresh(
                            adapter._recovery_heartbeat, actuation['command_max_age'])),
                        'feedback_fresh': bool(adapter._feedback_ready()),
                        'command_active': adapter._command is not None,
                        'published_command_active': command_active,
                        'fault_reason': adapter._position_guard.fault or adapter._reason or '',
                        'accepted_sequence': (adapter._accepted_identity[1]
                                              if adapter._accepted_identity is not None else None),
                        'timeout_events': adapter._timeout_events,
                        'rejected_commands': adapter._rejected_commands,
                    }

                def pump():
                    nonlocal sequence, last_publish, last_feedback_stamp
                    nonlocal sample_count, seen_recovery, fresh_recovery_samples
                    if process.poll() is not None:
                        raise RuntimeError(f'simulation exited: {process.poll()}')
                    if time.monotonic() - wall_start > policy['max_wall_seconds']:
                        raise TimeoutError('total wall deadline')
                    # Drain adapter subscriptions/timers as well as the recorder;
                    # DDS callbacks are never replaced by direct callback calls.
                    for _ in range(24):
                        executor.spin_once(timeout_sec=0.)
                    recorder.spin_once(.002)
                    if recorder.clock_reset:
                        raise RuntimeError('unexpected simulation clock reset')
                    now_wall = time.monotonic()
                    if (stage != 'native_settle' and recorder.clock is not None
                            and now_wall - last_publish >= policy['publish_period_seconds']):
                        message = SafeVelocityCommand()
                        message.header.stamp = adapter.get_clock().now().to_msg()
                        message.header.frame_id = context['robot']['differential_drive']['child_frame_id']
                        message.session_id = session
                        sequence += 1
                        message.sequence = sequence
                        message.joint_names = names
                        message.active = command_active
                        message.velocity = velocities.tolist() if command_active else [0.] * len(velocities)
                        publisher.publish(message)
                        last_publish = now_wall
                    observation_key = (adapter._joint_source_stamp, adapter._position_guard.mode)
                    if adapter._joint is not None and observation_key != last_feedback_stamp:
                        observation = snapshot()
                        sample_file.write(json.dumps(observation, allow_nan=False) + '\n')
                        sample_count += 1
                        last_feedback_stamp = observation_key
                        if observation['guard_mode'] in ('numerical_recovery', 'recovery'):
                            seen_recovery = True
                            fresh_recovery_samples += int(observation['recovery_authority_fresh']
                                                          and observation['feedback_fresh'])
                    if adapter._position_guard.mode == 'fault':
                        raise RuntimeError(adapter._position_guard.fault)

                def wait_stage(label, seconds, condition=None):
                    nonlocal stage
                    stage = label
                    started = recorder.clock
                    started_wall = time.monotonic()
                    while True:
                        pump()
                        if condition is not None and condition():
                            break
                        if started is None:
                            started = recorder.clock
                        if started is not None and recorder.clock is not None and recorder.clock >= started + seconds:
                            if condition is not None:
                                raise TimeoutError(f'{label}: simulation deadline without required observation')
                            break
                        if time.monotonic() - started_wall > policy['stage_wall_seconds']:
                            raise TimeoutError(f'{label}: wall deadline')
                    result['phases'][label] = {'start': started, 'end': recorder.clock, **snapshot()}

                # Let configured native holding finish startup before taking over
                # its position topics, matching normal controller readiness.
                wait_stage('native_settle', policy['settle_seconds'])
                command_active = True
                wait_stage('startup', policy['seek_seconds'],
                           lambda: adapter._feedback_ready() and adapter._command is not None)
                wait_stage('settle', policy['settle_seconds'])
                velocities[2 + index] = -sign * joint['velocity'] * policy['seek_speed_fraction']
                wait_stage('seek', policy['seek_seconds'], lambda: (
                    adapter._position_guard.mode != 'normal' or
                    (adapter._joint is not None and at_boundary(
                        adapter._joint[2][index], lower, upper, side,
                        recovery['numerical_tolerance']))))
                result['boundary_position'] = float(adapter._joint[2][index])
                # An inactive validated heartbeat handles an exactly legal stop
                # without inventing feedback excursions or changing guard state.
                command_active = False
                velocities[:] = 0.
                wait_stage('recovery_entry', policy['recovery_seconds'],
                           lambda: seen_recovery)
                command_active = True
                velocities[2 + index] = sign * joint['velocity'] * policy['resume_speed_fraction']
                wait_stage('recovery_release', policy['recovery_seconds'], lambda: (
                    seen_recovery and adapter._position_guard.mode == 'normal'
                    and adapter._joint is not None and beyond_release(
                        adapter._joint[2][index], lower, upper, side, recovery['release_margin'])))
                result['physical_release_demonstrated'] = True
                before_q = float(adapter._joint[2][index])
                before_base = np.asarray(recorder.streams['base_truth'][-1]['data']['position'])
                base_upper = context['control']['base_velocity_upper'][0]
                base_lower = context['control']['base_velocity_lower'][0]
                velocities[0] = policy['resume_speed_fraction'] * (base_upper if base_upper > 0 else base_lower)
                wait_stage('normal_resume', policy['resume_seconds'])
                after_base = np.asarray(recorder.streams['base_truth'][-1]['data']['position'])
                result['resumed_joint_displacement'] = sign * (float(adapter._joint[2][index]) - before_q)
                result['resumed_base_displacement'] = float(np.linalg.norm(after_base - before_base))
                result['normal_motion_resumed'] = bool(
                    adapter._position_guard.mode == 'normal' and adapter._command is not None
                    and result['resumed_joint_displacement'] > policy['minimum_motion_radians']
                    and result['resumed_base_displacement'] > policy['minimum_base_motion_metres'])
                result.update(recovery_seen=seen_recovery,
                              fresh_recovery_samples=fresh_recovery_samples,
                              samples=sample_count,
                              accepted_commands=adapter._accepted_commands,
                              rejected_commands=adapter._rejected_commands,
                              timeout_events=adapter._timeout_events)
                result['verdict'] = ('PASS' if result['normal_motion_resumed']
                                     and fresh_recovery_samples > 0 else 'FAIL')
        except Exception as error:
            result['error'] = f'{type(error).__name__}: {error}'
            result['verdict'] = 'FAIL'
            if adapter is not None:
                result['final_guard_mode'] = adapter._position_guard.mode
                result['final_reason'] = adapter._reason
        finally:
            # Revoke fixture command authority before stopping its physical world.
            if adapter is not None:
                adapter._halt('Validation fixture shutdown')
            if process is not None:
                result['cleanup'] = process.stop(5.)
            if executor is not None:
                executor.shutdown(timeout_sec=1.)
            if adapter is not None:
                adapter.destroy_node()
            if initialized:
                rclpy.try_shutdown()
            if driver is not None:
                driver.destroy_node()
            if recorder is not None:
                recorder.close()
            for key, value in saved.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
    trace = directory / 'physics/actuator.csv'
    if trace.exists():
        result['physical'] = analyze_trace(trace, context, policy['settle_seconds'])
        if not result['physical']['accepted']:
            result['verdict'] = 'FAIL'
    if result.get('cleanup', {}).get('survivors'):
        result['verdict'] = 'FAIL'
    write_json(directory / 'result.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variants', nargs='+', default=['C'])
    parser.add_argument('--joints', nargs='+', default=['wrist'])
    parser.add_argument('--sides', nargs='+', choices=['lower', 'upper'], default=['lower'])
    parser.add_argument('--seeds', type=int, nargs='+', default=[42])
    parser.add_argument('--engine', choices=['dartsim', 'dartsim_recovery'], default='dartsim_recovery')
    args = parser.parse_args()
    share = lambda name: Path(get_package_share_directory(name))
    spec = ExperimentCatalog(share=share).prepare('EXP-MANIPULATION')
    policy = load_yaml(Path(__file__).with_suffix('.yaml'))
    variants = [variant for variant in spec['variants'] if variant['name'] in args.variants]
    if {variant['name'] for variant in variants} != set(args.variants):
        parser.error('unknown variant requested')
    study = create_study_directory(args.output, 'authority_boundary')
    print(study, flush=True)
    results = []
    for variant in variants:
        for seed in args.seeds:
            for joint in args.joints:
                for side in args.sides:
                    directory = study / f'{variant["name"]}_seed{seed}_{joint}_{side}'
                    result = trial(variant, spec, directory, share, joint, side, seed, args.engine, policy)
                    results.append(result)
                    write_json(study / 'results.json', results)
                    print(variant['name'], seed, joint, side, result['verdict'], result.get('error'), flush=True)
    return 0 if all(result['verdict'] == 'PASS' for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
