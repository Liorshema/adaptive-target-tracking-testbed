"""Matched production runs: hold, legacy DLS and distance/pointing optimization."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import ActuationTelemetry, ControlTelemetry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
import yaml

from robot_bringup.model_factory import create_robot_model
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_simulation.factory.config_loader import load_yaml

ROOT = Path(__file__).resolve().parents[2]


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=[
        'hold', 'v1', 'v2', 'no_feedforward', 'no_lookahead', 'reactive'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', default='config/optimal_tracking.yaml')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--settling-time', type=float)
    args = parser.parse_args()
    thresholds = yaml.safe_load(Path(__file__).with_name('v2_thresholds.yaml').read_text())
    if args.duration is not None:
        thresholds['duration'] = args.duration
    if args.settling_time is not None:
        thresholds['settling_time'] = args.settling_time
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
    command = ['ros2', 'launch', 'robot_bringup', 'closed_loop.launch.py', 'gui:=false',
               f'config:={args.config}',
               f'enable_actuation:={str(enabled).lower()}']
    if args.case not in ['hold', 'v1']:
        command.append('strategy_config:=config/whole_body.yaml')
    if args.case == 'no_feedforward':
        command.append('feedforward:=false')
    if args.case == 'no_lookahead':
        command.append('lookahead:=0.0')
    master = load_yaml(ROOT / 'src/robot_bringup' / args.config)
    snapshot = {'bringup': master,
                'simulation': load_yaml(ROOT / 'src/robot_simulation' / master['simulation_config']),
                'description': description, 'actuation': actuator, 'control': control,
                'strategy': load_yaml(ROOT / 'src/robot_control/config/whole_body.yaml'),
                'estimation_overrides': load_yaml(
                    ROOT / 'src/robot_bringup' / master['estimation_overrides']),
                'launch': command}
    (args.output / 'configuration_snapshot.yaml').write_text(yaml.safe_dump(snapshot))
    log = (args.output / 'runtime.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    latest, rows, blocked, rtf = {}, [], Counter(), []
    received, cpu, cpu_processes = Counter(), [], {}
    node = None
    try:
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = Node('v2_tracking_observer')
        def record(key, message):
            received[key] += 1
            latest[key] = message
        node.create_subscription(JointState, robot['joint_states']['topic'], lambda m: record('joint', m), 10)
        node.create_subscription(Odometry, robot['differential_drive']['odom_topic'], lambda m: record('odom', m), 10)
        node.create_subscription(PointStamped, '/simulation/target_position', lambda m: record('target', m), 10)
        node.create_subscription(ActuationTelemetry, actuator['diagnostic_topic'], lambda m: record('actuation', m), 10)
        node.create_subscription(Twist, robot['differential_drive']['topic'], lambda m: record('base_command', m), 10)
        def telemetry(message):
            received['control_telemetry'] += 1
            if not message.valid:
                blocked[message.reason] += 1
                return
            if not all(key in latest for key in ['joint', 'odom', 'target']):
                blocked['Observer incomplete feedback'] += 1
                return
            joint, odom, target = [latest[key] for key in ['joint', 'odom', 'target']]
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
                        label = next((Path(arg).name for arg in arguments[:2]
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
        weights = yaml.safe_load((ROOT / 'src/robot_control/config/whole_body.yaml').read_text())['controller']['parameters']['motion_weights']
        dt = np.diff([row['time'] for row in selected])
        cost = np.sum(weights * (safe / velocity_scales)**2, axis=1)
        timeout_delta = selected[-1]['counters'].get('timeout_events', 0) - selected[0]['counters'].get('timeout_events', 0)
        result = dict(case=args.case, launch=command, target_initialization_time=origin, samples=len(selected),
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
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
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
