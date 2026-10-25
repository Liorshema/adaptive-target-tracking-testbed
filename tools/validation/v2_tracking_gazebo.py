"""Matched production runs: hold, legacy DLS and distance/pointing optimization."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import (ActuationTelemetry, ControlTelemetry, SafeVelocityCommand,
                                  StateEstimate, PredictedTrajectory)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from uuid import uuid4
import yaml

from robot_bringup.factory.model_factory import create_robot_model
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_configuration.yaml_loader import load_yaml

ROOT = Path(__file__).resolve().parents[2]


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=[
        'hold', 'v1', 'v2', 'no_feedforward', 'no_lookahead', 'reactive'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', default='config/optimal_tracking.yaml')
    parser.add_argument('--strategy-config', default='config/whole_body.yaml')
    parser.add_argument('--system-config', default='',
                        help='Exercise the existing bringup unified-manifest entry point')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--without-markers', action='store_true',
                        help='Retain physical ground truth without the optional RViz marker node')
    parser.add_argument('--profile-control', action='store_true',
                        help='Profile startup solves and record pipeline CPU versus wall time')
    parser.add_argument('--physical-ground-truth', action='store_true',
                        help='Evaluate using actual Gazebo base pose instead of wheel odometry')
    parser.add_argument('--shoulder-posture', action='store_true',
                        help='Capture native first-link poses and raw target/live TF experiments')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--settling-time', type=float)
    parser.add_argument('--wall-timeout', type=float)
    parser.add_argument('--target-trace', action='store_true',
                        help='Observe exact target state/acceleration without altering integration')
    parser.add_argument('--estimator-trace', action='store_true',
                        help='Observe actual estimator callback ordering')
    parser.add_argument('--before-m03-estimation', action='store_true',
                        help='Load archived pre-M03 estimator only in the validation wrapper')
    parser.add_argument('--drive-fixture', type=Path,
                        help='Drive only through the normal safe-command actuator interface')
    parser.add_argument('--actuation-trace', action='store_true',
                        help='Observe timestamps of actual emitted PID position references')
    parser.add_argument('--before-m02-actuation', action='store_true',
                        help='Use archived pre-M02 actuation for a controlled failure reproduction')
    args = parser.parse_args()
    if args.before_m03_estimation:
        args.estimator_trace = True
        os.environ['M03_BASELINE_ESTIMATION'] = str(
            (ROOT / 'results/validation/m03_before_source').resolve())
    if args.before_m02_actuation:
        args.actuation_trace = True
        os.environ['M02_BASELINE_ACTUATION'] = str(
            (ROOT / 'results/validation/m02_before_source').resolve())
    thresholds = yaml.safe_load(Path(__file__).with_name('v2_thresholds.yaml').read_text())
    if args.duration is not None:
        thresholds['duration'] = args.duration
    if args.settling_time is not None:
        thresholds['settling_time'] = args.settling_time
    if args.wall_timeout is not None:
        thresholds['wall_timeout'] = args.wall_timeout
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '115')
    os.environ.update(GZ_PARTITION=f'v2-evaluation-{os.getpid()}', GZ_IP='127.0.0.1',
                      ROS_LOG_DIR=str(args.output / 'ros-log'))
    robot = yaml.safe_load((ROOT / 'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
    description = yaml.safe_load((ROOT / 'src/robot_description/config/robot/robot.yaml').read_text())['robot']
    control = yaml.safe_load((ROOT / 'src/robot_control/config/control.yaml').read_text())['control_node']['ros__parameters']
    actuator = yaml.safe_load((ROOT / 'src/robot_simulation/config/actuation.yaml').read_text())['actuation']
    names = [robot['joint_name_template'].format(key=key) for key in description['manipulator']['joints']]
    model = create_robot_model()
    initial = robot['pose']
    world_odom_rotation = Rotation.from_euler('xyz', initial[3:]).as_matrix()
    enabled = args.case != 'hold'
    command = ['ros2', 'launch', 'robot_bringup', 'closed_loop.launch.py',
               f'gui:={str(args.gui).lower()}',
               f'config:={args.config}',
               f'enable_actuation:={str(enabled).lower()}']
    if args.system_config:
        command = ['ros2', 'launch', 'robot_bringup', 'closed_loop.launch.py',
                   f'system_config:={args.system_config}', f'gui:={str(args.gui).lower()}',
                   f'enable_actuation:={str(enabled).lower()}']
    elif args.case not in ['hold', 'v1']:
        command.append(f'strategy_config:={args.strategy_config}')
    if args.case == 'no_feedforward':
        command.append('feedforward:=false')
    if args.case == 'no_lookahead':
        command.append('lookahead:=0.0')
    if args.physical_ground_truth:
        command.extend(['publish_ground_truth:=true', 'visualize_tracking:=false']
                       if args.without_markers else ['visualize_tracking:=true'])
    if args.drive_fixture:
        command.append('publish_commands:=false')
    master = load_yaml(ROOT / 'src/robot_bringup' / args.config)
    if args.system_config:
        from robot_bringup.factory.system_configuration import load_system_configuration
        manifest = Path(args.system_config)
        if not manifest.is_absolute():
            manifest = ROOT / 'src/robot_bringup' / manifest
        plan = load_system_configuration(manifest, lambda package: ROOT / 'src' / package)
        master = {key: str(value) if isinstance(value, Path) else value
                  for key, value in plan['profile'].items()}
    snapshot = {'bringup': master, 'robot': robot,
                'simulation': load_yaml(ROOT / 'src/robot_simulation' / master['simulation_config']),
                'description': description, 'actuation': actuator, 'control': control,
                'strategy': load_yaml(ROOT / 'src/robot_control' / args.strategy_config),
                'estimation_overrides': load_yaml(
                    ROOT / 'src/robot_bringup' / master['estimation_overrides']),
                'launch': command}
    if args.system_config:
        snapshot['system_manifest'] = args.system_config
        snapshot['selected_components'] = plan['identities']
        snapshot['prediction_configuration'] = load_yaml(plan['prediction_path'])
    simulation_resource = ROOT / 'src/robot_simulation'
    sim_cfg = snapshot['simulation']
    snapshot['target'] = load_yaml(simulation_resource / sim_cfg['target']['config'])
    snapshot['holding'] = load_yaml(
        simulation_resource / sim_cfg['robot']['joint_position_hold'])['joint_position_hold']
    snapshot['environment'] = load_yaml(simulation_resource / sim_cfg['simulation']['environment_config'])
    snapshot['joint_order'] = names
    snapshot['estimator'] = yaml.safe_load((ROOT / 'src/robot_estimation' /
        master['robot_estimator_config']).read_text())['estimation_node']['ros__parameters']
    snapshot['before_m03_estimation'] = args.before_m03_estimation
    if args.drive_fixture:
        snapshot['drive_fixture'] = load_yaml(args.drive_fixture)
    if args.target_trace or args.estimator_trace or args.actuation_trace or args.profile_control:
        code = (ROOT / 'src/robot_bringup/launch/closed_loop.launch.py').read_text()
        code = code.replace('from pathlib import Path', 'from pathlib import Path\nimport sys')
        if args.target_trace:
            os.environ['TARGET_DYNAMICS_TRACE'] = str((args.output / 'target_trace.json').resolve())
            sim_code = (ROOT / 'src/robot_simulation/launch/simulation.launch.py').read_text()
            sim_code = sim_code.replace('from pathlib import Path', 'from pathlib import Path\nimport sys')
            anchor = 'executable="simulation_node",'
            assert anchor in sim_code
            sim_code = sim_code.replace(anchor, anchor +
                f" prefix=[sys.executable, ' ', '{ROOT / 'tools/validation/target_dynamics_trace.py'}'],")
            observed_sim = args.output / 'observed_simulation.launch.py'
            observed_sim.write_text(sim_code)
            anchor = "str(share('robot_simulation') /\n                                              'launch/simulation.launch.py')"
            assert anchor in code
            code = code.replace(anchor, repr(str(observed_sim.resolve())))
        if args.estimator_trace:
            os.environ['ESTIMATION_TIMING_LOG'] = str((args.output / 'processing.json').resolve())
            anchor = "Node(package='robot_estimation', executable='estimation_node', output='screen',"
            assert anchor in code
            code = code.replace(anchor, anchor +
                f" prefix=[sys.executable, ' ', '{ROOT / 'tools/validation/estimator_timing_probe.py'}'],")
        if args.actuation_trace:
            os.environ['ACTUATION_DYNAMICS_TRACE'] = str((args.output / 'actuation_trace.json').resolve())
            anchor = "Node(package='robot_simulation', executable='velocity_actuation', output='screen',"
            assert anchor in code
            code = code.replace(anchor, anchor +
                f" prefix=[sys.executable, ' ', '{ROOT / 'tools/validation/actuation_dynamics_trace.py'}'],")
        if args.profile_control:
            os.environ['CONTROL_PROFILE_OUTPUT'] = str(args.output.resolve())
            anchor = "Node(package='robot_control', executable='control_node', output='screen',"
            assert anchor in code
            code = code.replace(anchor, anchor +
                f" prefix=[sys.executable, ' ', '{ROOT / 'tools/validation/control_compute_profile.py'}'],")
        launch = args.output / 'observed.launch.py'
        launch.write_text(code)
        command = ['ros2', 'launch', str(launch.resolve()), *command[4:]]
        snapshot['launch'] = command
    (args.output / 'configuration_snapshot.yaml').write_text(yaml.safe_dump(snapshot, sort_keys=False))
    log = (args.output / 'runtime.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    scene_process, scene_log = None, None
    if args.shoulder_posture:
        scene_log = (args.output / 'gazebo_poses.jsonl').open('w')
        scene_process = subprocess.Popen([
            'gz', 'topic', '-e', '-t', '/world/tracking_testbed/pose/info', '--json-output'],
            stdout=scene_log, stderr=log, start_new_session=True)
    streams = {'joints': [], 'targets': [], 'tracking_tf': [], 'commands': [], 'control': [],
               'robot_estimates': [],
               'wheel_joints': [],
               'estimation_diagnostics': [],
               'position_commands': [], 'odometry': [], 'physical_base': [], 'actuation': []}
    latest, rows, blocked, rtf = {}, [], Counter(), []
    received, cpu, cpu_processes = Counter(), [], {}
    node = None
    try:
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = Node('v2_tracking_observer')
        if args.drive_fixture:
            publisher = node.create_publisher(SafeVelocityCommand, actuator['command_topic'], 10)
            session, sequence = str(uuid4()), 0
            segments = snapshot['drive_fixture']['segments']
            ramp = snapshot['drive_fixture']['ramp_seconds']
            base_scale = np.maximum(np.abs(control['base_velocity_lower']),
                                    np.abs(control['base_velocity_upper']))
            def drive_command():
                nonlocal sequence
                now = node.get_clock().now()
                elapsed = -1. if origin is None else now.nanoseconds * 1e-9 - origin
                desired, previous = np.zeros(2), np.zeros(2)
                for segment in segments:
                    if 0 <= elapsed < segment['seconds']:
                        blend = min(elapsed / ramp, 1.)
                        desired = previous + blend * (
                            np.asarray(segment['velocity_fraction']) - previous)
                        break
                    elapsed -= segment['seconds']
                    previous = np.asarray(segment['velocity_fraction'])
                message = SafeVelocityCommand()
                message.header.stamp = now.to_msg()
                message.header.frame_id = robot['differential_drive']['child_frame_id']
                message.joint_names = names
                message.session_id, message.sequence = session, sequence
                message.active = origin is not None
                message.velocity = np.r_[base_scale * desired, np.zeros(len(names))].tolist()
                publisher.publish(message)
                sequence += 1
            node.create_timer(1 / actuator['rate_hz'], drive_command)
        def record(key, message):
            received[key] += 1
            latest[key] = message
            if args.shoulder_posture and key == 'joint':
                streams['joints'].append([stamp(message.header),
                    [message.position[message.name.index(name)] for name in names],
                    [message.velocity[message.name.index(name)] for name in names]])
                wheel_names = (robot['differential_drive']['left_joints'] +
                               robot['differential_drive']['right_joints'])
                streams['wheel_joints'].append([stamp(message.header),
                    [message.position[message.name.index(name)] for name in wheel_names],
                    [message.velocity[message.name.index(name)] for name in wheel_names]])
            elif args.shoulder_posture and key == 'target':
                streams['targets'].append([stamp(message.header),
                    [message.point.x, message.point.y, message.point.z]])
            elif args.shoulder_posture and key == 'robot_estimate':
                streams['robot_estimates'].append([stamp(message.header), list(message.mean),
                                                   list(message.covariance)])
            elif args.shoulder_posture and key in ['odom', 'physical']:
                p, q = message.pose.pose.position, message.pose.pose.orientation
                streams['odometry' if key == 'odom' else 'physical_base'].append(
                    [stamp(message.header), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]])
            elif args.shoulder_posture and key == 'actuation':
                streams['actuation'].append([stamp(message.header), list(message.commanded_velocity),
                                             list(message.position_reference), message.active,
                                             message.reason])
        if args.shoulder_posture:
            node.create_subscription(String, snapshot['estimator']['diagnostic_topic'],
                lambda m: streams['estimation_diagnostics'].append(json.loads(m.data)), 100)
            def tracking_tf(message):
                for transform in message.transforms:
                    if transform.child_frame_id == 'tracking_frame':
                        point = transform.transform.translation
                        streams['tracking_tf'].append([stamp(transform.header), [point.x, point.y, point.z]])
            node.create_subscription(TFMessage, '/tf', tracking_tf, 100)
            node.create_subscription(SafeVelocityCommand, actuator['command_topic'],
                lambda m: streams['commands'].append(
                    [stamp(m.header), list(m.velocity), m.active, m.session_id, m.sequence]), 100)
            for index, name in enumerate(names):
                topic = robot['joint_position_topic_template'].format(
                    model=robot['entity_name'], joint=name)
                node.create_subscription(Float64, topic,
                    lambda m, i=index: streams['position_commands'].append(
                        [node.get_clock().now().nanoseconds * 1e-9, i, m.data]), 100)
        node.create_subscription(JointState, robot['joint_states']['topic'], lambda m: record('joint', m), 10)
        node.create_subscription(Odometry, robot['differential_drive']['odom_topic'], lambda m: record('odom', m), 10)
        if args.physical_ground_truth:
            node.create_subscription(Odometry, robot['ground_truth_odometry']['topic'],
                                     lambda m: record('physical', m), 10)
        node.create_subscription(PointStamped, '/simulation/target_position', lambda m: record('target', m), 10)
        node.create_subscription(ActuationTelemetry, actuator['diagnostic_topic'], lambda m: record('actuation', m), 10)
        node.create_subscription(Twist, robot['differential_drive']['topic'], lambda m: record('base_command', m), 10)
        for topic, message_type, key in [
            ('/estimation/robot_state_full', StateEstimate, 'robot_estimate'),
            ('/estimation/target_state_full', StateEstimate, 'target_estimate'),
            ('/prediction/trajectory', PredictedTrajectory, 'prediction'),
        ]:
            node.create_subscription(message_type, topic, lambda m, k=key: record(k, m), 10)
        def telemetry(message):
            received['control_telemetry'] += 1
            if args.shoulder_posture:
                streams['control'].append([stamp(message.header), message.valid, message.reason])
            if not message.valid:
                blocked[message.reason] += 1
                return
            if not all(key in latest for key in ['joint', 'odom', 'target']):
                blocked['Observer incomplete feedback'] += 1
                return
            joint, odom, target = [latest[key] for key in ['joint', 'odom', 'target']]
            if args.physical_ground_truth:
                if 'physical' not in latest:
                    blocked['Observer missing physical pose'] += 1
                    return
                odom = latest['physical']
            timestamp = stamp(message.header)
            if max(abs(timestamp - stamp(m.header)) for m in [joint, odom, target]) > thresholds['max_feedback_skew']:
                blocked['Observer feedback skew'] += 1
                return
            indices = [joint.name.index(name) for name in names]
            q = np.asarray(joint.position)[indices]
            dq = np.asarray(joint.velocity)[indices]
            pose = odom.pose.pose
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                                  pose.orientation.z, pose.orientation.w]).as_matrix()
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            if args.physical_ground_truth:
                world_odom = np.eye(4)
                world_odom[:3, :3] = world_odom_rotation
                world_odom[:3, 3] = initial[:3]
                transform = np.linalg.inv(world_odom) @ transform
            actual = WholeBodyKinematics.compute(robot_model=model, transform_world_base=transform, joint_positions=q)
            target_position = world_odom_rotation.T @ (np.array([target.point.x, target.point.y, target.point.z]) - initial[:3])
            relative = target_position - actual[:3, 3]
            distance = np.linalg.norm(relative)
            optical = actual[:3, :3] @ np.asarray(control['tracking_forward_axis'])
            pointing = np.degrees(np.arccos(np.clip(optical @ relative / (np.linalg.norm(optical) * distance), -1., 1.)))
            diagnostic = latest.get('actuation')
            reference = list(diagnostic.position_reference) if diagnostic else []
            base_command = latest.get('base_command')
            counters = {key: getattr(diagnostic, key) for key in ['accepted_commands', 'rejected_commands',
                         'timeout_events', 'reference_limit_events', 'inactive_commands', 'feedback_rejections']} if diagnostic else {}
            rows.append(dict(time=timestamp, target=target_position.tolist(),
                             predicted_target=[message.target_position.x, message.target_position.y, message.target_position.z],
                             actual_position=actual[:3, 3].tolist(), actual_rotation=actual[:3, :3].tolist(),
                             distance_error=distance - control['tracking_radius'], pointing_degrees=float(pointing),
                             base=[pose.position.x, pose.position.y, Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')[2]],
                             base_velocity=[odom.twist.twist.linear.x, odom.twist.twist.angular.z],
                             base_command=[base_command.linear.x, base_command.angular.z] if base_command else [],
                             joints=q.tolist(), joint_velocity=dq.tolist(), position_reference=reference,
                             actuator_input=list(diagnostic.commanded_velocity) if diagnostic else [],
                             actuation_active=diagnostic.active if diagnostic else False, counters=counters,
                             raw=list(message.raw_velocity), safe=list(message.safe_velocity),
                             task_error=list(message.task_error), singular_values=list(message.jacobian_singular_values),
                             compute_time=message.computation_time, active_bounds=message.active_bounds,
                             weighted_motion_cost=message.weighted_motion_cost, task_residual=message.task_residual))
        node.create_subscription(ControlTelemetry, control['telemetry_topic'], telemetry, 100)
        start_wall = time.monotonic()
        origin = None
        previous_wall, previous_sim = start_wall, 0.
        while time.monotonic() - start_wall < thresholds['wall_timeout']:
            rclpy.spin_once(node, timeout_sec=.01)
            wall, simulated = time.monotonic(), node.get_clock().now().nanoseconds * 1e-9
            if origin is None and 'target' in latest:
                match = re.search(r'Simulation initialized at t=([\d.]+)s', (args.output / 'runtime.log').read_text())
                if match:
                    origin = float(match.group(1))
                    previous_sim, previous_wall = simulated, wall
            if origin is not None and wall - previous_wall >= 1.:
                rtf.append([simulated - origin, (simulated - previous_sim) / (wall - previous_wall)])
                measurements = {}
                for child in psutil.Process(process.pid).children(recursive=True):
                    try:
                        if child.pid not in cpu_processes:
                            cpu_processes[child.pid] = child
                            child.cpu_percent()
                            continue
                        child = cpu_processes[child.pid]
                        arguments = child.cmdline()
                        # Preserve the actual generated limits/PID/physics, not just source YAML.
                        if not (args.output / 'generated_world.sdf').exists():
                            match = re.search(r'(/tmp/robot_simulation_[^ /]+/world\.sdf)',
                                              ' '.join(arguments))
                            if match and Path(match.group(1)).is_file():
                                (args.output / 'generated_world.sdf').write_text(
                                    Path(match.group(1)).read_text())
                        label = next((Path(arg).name for arg in arguments
                                      if '/lib/robot_' in arg), child.name())
                        measurements[label] = measurements.get(label, 0.) + child.cpu_percent()
                    except psutil.Error:
                        continue
                cpu.append([simulated - origin, measurements])
                previous_sim, previous_wall = simulated, wall
            if origin is not None and simulated >= origin + thresholds['duration']:
                break
            if process.poll() is not None:
                raise RuntimeError('Production launch exited unexpectedly')
        else:
            raise RuntimeError('Simulation timed out')
        selected = [row for row in rows if thresholds['settling_time'] <= row['time'] - origin <= thresholds['duration']]
        if not selected:
            raise RuntimeError('No measured evaluation samples')
        errors = np.abs([row['distance_error'] for row in selected])
        angles = np.array([row['pointing_degrees'] for row in selected])
        base = np.array([row['base'] for row in selected])
        joints = np.array([row['joints'] for row in selected])
        safe = np.array([row['safe'] for row in selected])
        limits = description['manipulator']['joints'].values()
        velocity_scales = np.r_[np.maximum(np.abs(control['base_velocity_lower']), np.abs(control['base_velocity_upper'])), [j['velocity'] for j in limits]]
        weights = snapshot['strategy']['controller']['parameters']['motion_weights']
        dt = np.diff([row['time'] for row in selected])
        cost = np.sum(weights * (safe / velocity_scales)**2, axis=1)
        timeout_delta = selected[-1]['counters'].get('timeout_events', 0) - selected[0]['counters'].get('timeout_events', 0)
        result = dict(case=args.case, launch=command, target_initialization_time=origin, samples=len(selected),
                      pose_source=('Gazebo physical base pose + measured joints' if args.physical_ground_truth
                                   else 'wheel odometry + measured joints'),
                      mean_distance_error=float(np.mean(errors)), max_distance_error=float(np.max(errors)),
                      mean_pointing_degrees=float(angles.mean()), max_pointing_degrees=float(angles.max()),
                      base_excursion=float(np.max(np.linalg.norm(base[:, :2] - base[0, :2], axis=1))),
                      base_yaw_excursion=float(np.ptp(np.unwrap(base[:, 2]))), joint_excursions=np.ptp(joints, axis=0).tolist(),
                      joint_velocity_peaks=np.abs([row['joint_velocity'] for row in selected]).max(axis=0).tolist(),
                      weighted_motion_integral=float(np.sum(cost[:-1] * dt)) if enabled else 0.,
                      mean_command_variation=float(np.mean(np.linalg.norm(np.diff(safe / velocity_scales, axis=0), axis=1))) if enabled else 0.,
                      mean_rtf=float(np.mean([value for phase, value in rtf if phase >= thresholds['settling_time']])),
                      mean_compute_ms=float(np.mean([row['compute_time'] for row in selected]) * 1000),
                      p95_compute_ms=float(np.quantile([row['compute_time'] for row in selected], .95) * 1000),
                      saturation_cycles=sum(row['active_bounds'] > 0 for row in selected),
                      watchdog_timeouts_after_settling=timeout_delta, blocked_cycles=dict(blocked),
                      counters=selected[-1]['counters'], actuation_active_fraction=float(np.mean([row['actuation_active'] for row in selected])),
                      thresholds=thresholds)
        result['target_excursion'] = float(np.max(np.linalg.norm(
            np.asarray([row['target'] for row in selected]) - selected[0]['target'], axis=1)))
        joint_limits = list(description['manipulator']['joints'].values())
        low, high = np.array([j['lower'] for j in joint_limits]), np.array([j['upper'] for j in joint_limits])
        result['minimum_joint_limit_margin'] = float(np.min(np.minimum(joints - low, high - joints)))
        result['checks'] = dict(physical_limits=bool(np.all(joints >= low - thresholds['physical_numerical_tolerance']) and
                                                    np.all(joints <= high + thresholds['physical_numerical_tolerance'])),
                               watchdog=timeout_delta <= thresholds['max_watchdog_timeouts_after_settling'])
        tolerance = thresholds['physical_numerical_tolerance']
        result['checks']['measured_joint_velocity_limits'] = bool(np.all(
            np.asarray(result['joint_velocity_peaks']) <= velocity_scales[2:] + tolerance))
        result['checks']['safe_velocity_limits'] = bool(np.all(
            np.abs(safe) <= velocity_scales + tolerance))
        steady_cpu = [values for phase, values in cpu if phase >= thresholds['settling_time']]
        result['mean_process_cpu_percent'] = {
            label: float(np.mean([values.get(label, 0.) for values in steady_cpu]))
            for label in sorted({label for values in steady_cpu for label in values})}
        result['observed_topic_rates_sim_hz'] = {
            label: count / thresholds['duration'] for label, count in received.items()}
        if args.shoulder_posture:
            from shoulder_experiment_metrics import experiment_metrics
            (args.output / 'experiment_streams.json').write_text(json.dumps(streams))
            result['experiment'] = experiment_metrics(
                args.output, streams, description, robot, origin, thresholds, control=control,
                center=np.asarray(snapshot['environment']['environment']['acceleration_fields'][0]['parameters']['center']))
            measurement = result['experiment']
            if 'raw_joint_speed_peaks' in measurement:
                result['telemetry_sampled_joint_velocity_peaks'] = result['joint_velocity_peaks']
                result['joint_velocity_peaks'] = measurement['raw_joint_speed_peaks']
                result['joint_speed_source'] = 'all received joint-state measurements after settling'
                result['checks']['measured_joint_velocity_limits'] = bool(np.all(
                    np.asarray(result['joint_velocity_peaks']) <= velocity_scales[2:] + tolerance))
            shoulder = list(description['manipulator']['joints']).index('shoulder')
            result['checks'].update(
                shoulder_all_feedback=bool(
                    measurement['minimum_shoulder'] >= low[shoulder] - tolerance),
                first_link_horizontal=measurement['minimum_link_elevation'] >= -tolerance,
                first_link_base_clearance=measurement['minimum_first_link_box_base_clearance'] >= -tolerance)
            if snapshot['simulation']['tracking']['source'] == 'tf':
                result['checks']['live_tracking_tf'] = (
                    measurement['tracking_tf_samples'] > 1 and
                    measurement['tracking_tf_excursion'] > 0.)
        if args.case == 'v2':
            result['checks'].update(distance=result['mean_distance_error'] <= thresholds['max_mean_distance_error'],
                                    max_distance=result['max_distance_error'] <= thresholds['max_distance_error'],
                                    pointing=result['mean_pointing_degrees'] <= thresholds['max_mean_pointing_degrees'],
                                    max_pointing=result['max_pointing_degrees'] <= thresholds['max_pointing_degrees'],
                                    base_motion=result['base_excursion'] >= thresholds['min_base_excursion'],
                                    arm_motion=float(np.linalg.norm(result['joint_excursions'])) >= thresholds['min_arm_motion'],
                                    performance=result['mean_rtf'] >= thresholds['min_mean_rtf'])
        if args.case == 'hold':
            result['checks']['no_actuation'] = not any(row['base_command'] for row in rows)
        (args.output / 'samples.jsonl').write_text('\n'.join(json.dumps(row) for row in rows) + '\n')
        (args.output / 'rtf.json').write_text(json.dumps(rtf))
        (args.output / 'cpu.json').write_text(json.dumps(cpu))
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        if not all(result['checks'].values()):
            raise RuntimeError('Configured experiment checks failed')
    except Exception as error:
        (args.output / 'failure.json').write_text(json.dumps({
            'case': args.case, 'launch': command, 'error': str(error),
            'samples': len(rows), 'blocked_cycles': dict(blocked)}, indent=2) + '\n')
        if rows and not (args.output / 'samples.jsonl').exists():
            (args.output / 'samples.jsonl').write_text(
                '\n'.join(json.dumps(row) for row in rows) + '\n')
        raise
    finally:
        # A diagnosed safety stop is evidence even if it leaves no valid control samples.
        if args.shoulder_posture:
            (args.output / 'experiment_streams.json').write_text(json.dumps(streams))
            (args.output / 'rtf.json').write_text(json.dumps(rtf))
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if scene_process is not None:
            if scene_process.poll() is None:
                os.killpg(scene_process.pid, signal.SIGINT)
                try:
                    scene_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(scene_process.pid, signal.SIGTERM)
                    scene_process.wait(timeout=5)
            scene_log.close()
        if process.poll() is None:
            # Let launch propagate one interrupt, avoiding duplicate child SIGINTs
            # while observational wrappers flush their final records.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        log.close()


if __name__ == '__main__':
    main()
