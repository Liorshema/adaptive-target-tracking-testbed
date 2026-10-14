"""Observe the production Bringup launch, with reproducible isolated runs.

No validation-only model or alternative controller is constructed. The FK
model includes its FixedMountModel once. References come from real Control.
"""

import argparse
from collections import Counter
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import ControlTelemetry, SafeVelocityCommand
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
import yaml

from robot_bringup.model_factory import create_robot_model
from robot_models.whole_body.kinematics import WholeBodyKinematics

ROOT = Path(__file__).resolve().parents[2]


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['active', 'baseline', 'disabled'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--thresholds', type=Path,
                        default=Path(__file__).with_name('closed_loop_thresholds.yaml'))
    args = parser.parse_args()
    thresholds = yaml.safe_load(args.thresholds.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '89')
    os.environ['GZ_PARTITION'] = f'closed-loop-{args.case}-{os.getpid()}'
    os.environ['GZ_IP'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros-log')
    config = yaml.safe_load((ROOT / 'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
    physical = yaml.safe_load((ROOT / 'src/robot_description/config/robot/robot.yaml').read_text())['robot']
    control = yaml.safe_load((ROOT / 'src/robot_control/config/control.yaml').read_text())['control_node']['ros__parameters']
    joint_names = [config['joint_name_template'].format(key=key)
                   for key in physical['manipulator']['joints']]
    model = create_robot_model()
    enabled = args.case != 'baseline'
    command = ['ros2', 'launch', 'robot_bringup', 'closed_loop.launch.py',
               'gui:=false', f'enable_actuation:={str(enabled).lower()}']
    if args.case == 'disabled':
        command.append('publish_commands:=false')
    log = (args.output / 'runtime.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    node = None
    data = []
    latest = {}
    blocked = Counter()
    commands = []
    outputs = []
    ground_truth = []
    try:
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = Node('closed_loop_observer')
        def received(key, message):
            latest[key] = message
        node.create_subscription(Odometry, config['differential_drive']['odom_topic'],
                                 lambda m: received('odom', m), 10)
        node.create_subscription(JointState, config['joint_states']['topic'],
                                 lambda m: received('joints', m), 10)
        node.create_subscription(SafeVelocityCommand, control['command_topic'],
                                 commands.append, 10)
        node.create_subscription(Twist, config['differential_drive']['topic'],
                                 lambda m: outputs.append([node.get_clock().now().nanoseconds * 1e-9,
                                                           m.linear.x, m.angular.z]), 10)
        node.create_subscription(PointStamped, '/simulation/target_position',
                                 lambda m: ground_truth.append([stamp(m.header), m.point.x,
                                                                m.point.y, m.point.z]), 10)

        def telemetry(message):
            if not message.valid:
                blocked[message.reason] += 1
                return
            if not all(key in latest for key in ['odom', 'joints']):
                blocked['Observer missing feedback'] += 1
                return
            odom, joints = latest['odom'], latest['joints']
            timestamp = stamp(message.header)
            if max(abs(timestamp - stamp(m.header)) for m in [odom, joints]) > thresholds['max_feedback_skew']:
                blocked['Observer feedback skew'] += 1
                return
            indices = [list(joints.name).index(name) for name in joint_names]
            q = np.array(joints.position)[indices]
            dq = np.array(joints.velocity)[indices]
            transform = np.eye(4)
            pose = odom.pose.pose
            transform[:3, :3] = Rotation.from_quat([
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            actual = WholeBodyKinematics.compute(robot_model=model,
                                                transform_world_base=transform, joint_positions=q)
            desired = message.desired_pose.position
            reference = np.array([desired.x, desired.y, desired.z])
            target = message.target_position
            data.append(dict(time=timestamp, target=[target.x, target.y, target.z],
                             desired=reference.tolist(), actual=actual[:3, 3].tolist(),
                             actual_rotation=actual[:3, :3].tolist(),
                             error=float(np.linalg.norm(reference - actual[:3, 3])),
                             raw=list(message.raw_velocity), safe=list(message.safe_velocity),
                             base=[pose.position.x, pose.position.y,
                                   Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')[2]],
                             base_velocity=[odom.twist.twist.linear.x, odom.twist.twist.angular.z],
                             joints=q.tolist(), joint_velocity=dq.tolist()))

        node.create_subscription(ControlTelemetry, control['telemetry_topic'], telemetry, 100)
        wall_start = time.monotonic()
        # Align the comparison to target initialization, not DDS discovery.
        simulation_start = None
        while time.monotonic() - wall_start < thresholds['wall_timeout']:
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('Bringup exited unexpectedly; inspect runtime.log')
            if ground_truth and simulation_start is None:
                match = re.search(r'Simulation initialized at t=([\d.]+)s',
                                  (args.output / 'runtime.log').read_text())
                if match:
                    simulation_start = float(match.group(1))
            if simulation_start is not None and node.get_clock().now().nanoseconds * 1e-9 >= simulation_start + thresholds['duration']:
                break
        else:
            raise RuntimeError('Simulation did not reach configured duration')
        samples = [row for row in data if simulation_start + thresholds['settling_time'] <= row['time'] <= simulation_start + thresholds['duration']]
        if not samples:
            raise RuntimeError('No measured-FK evaluation samples')
        errors = np.array([row['error'] for row in samples])
        base = np.array([row['base'] for row in samples])
        joints = np.array([row['joints'] for row in samples])
        target = np.array(ground_truth)[:, 1:]
        result = dict(case=args.case, launch_command=command, thresholds=thresholds,
                      simulation_start=simulation_start, samples=len(samples),
                      mean_position_error=float(errors.mean()),
                      rmse_position_error=float(np.sqrt(np.mean(errors**2))),
                      p95_position_error=float(np.quantile(errors, .95)),
                      base_excursion=float(np.max(np.linalg.norm(base[:, :2] - base[0, :2], axis=1))),
                      joint_excursions=np.ptp(joints, axis=0).tolist(),
                      target_excursion=float(np.max(np.linalg.norm(target - target[0], axis=1))),
                      blocked_cycles=dict(blocked), active_commands=sum(m.active for m in commands),
                      actuator_outputs=len(outputs),
                      orientation_task_enabled=any(axis >= 3 for axis in control['controller_task_axes']))
        result['checks'] = {'target_moves': result['target_excursion'] >= thresholds['min_target_excursion']}
        if args.case == 'active':
            result['checks'].update(
                mean_error=result['mean_position_error'] <= thresholds['max_mean_position_error'],
                rmse_error=result['rmse_position_error'] <= thresholds['max_rmse_position_error'],
                base_moves=result['base_excursion'] >= thresholds['min_base_excursion'])
        else:
            result['checks']['no_active_commands'] = result['active_commands'] == 0
        with (args.output / 'samples.jsonl').open('w') as stream:
            for row in data:
                stream.write(json.dumps(row) + '\n')
        (args.output / 'target.json').write_text(json.dumps(ground_truth))
        (args.output / 'base_commands.json').write_text(json.dumps(outputs))
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        if not all(result['checks'].values()):
            raise RuntimeError('Configured experiment thresholds failed')
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
