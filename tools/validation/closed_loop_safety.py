"""Physical actuator smoke and failure/recovery checks on production launches."""
from robot_control.configuration import load_control_parameters

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import SafeVelocityCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['smoke', 'failures'], required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', default='')
    parser.add_argument('--strategy-config', default='')
    parser.add_argument('--visualization', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '91')
    os.environ['GZ_PARTITION'] = f'closed-loop-safety-{os.getpid()}'
    os.environ['GZ_IP'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros-log')
    limits = yaml.safe_load((ROOT / 'src/robot_description/config/robot/robot.yaml').read_text())['robot']['manipulator']['joints']
    robot = yaml.safe_load((ROOT / 'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
    control = load_control_parameters(ROOT / 'src/robot_control/config/control.yaml', 'parameters')['control_node']['ros__parameters']
    thresholds = yaml.safe_load(Path(__file__).with_name('closed_loop_thresholds.yaml').read_text())
    names = [robot['joint_name_template'].format(key=key) for key in limits]
    launch = ['ros2', 'launch', 'robot_experiments', 'tracking.launch.py', 'enable_actuation:=true']
    if args.config:
        launch.append(f'config:={args.config}')
    if args.strategy_config:
        launch.append(f'strategy_config:={args.strategy_config}')
    if args.visualization:
        launch.append('visualize_tracking:=true')
    if args.case == 'smoke':
        launch.append('publish_commands:=false')
    log = (args.output / 'runtime.log').open('w')
    process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    paused = []
    restarts = []
    node = None
    latest = {}
    outputs = []
    commands = []
    samples = []
    references = {}
    results = {}
    marker_samples = []
    try:
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = Node('closed_loop_safety_observer')
        def now():
            return node.get_clock().now().nanoseconds * 1e-9
        def received(key, msg):
            latest[key] = msg
        def joints(msg):
            try:
                q = np.array(msg.position)[[msg.name.index(name) for name in names]]
            except ValueError:
                return
            samples.append((now(), q))
            latest['joints'] = q
        node.create_subscription(JointState, robot['joint_states']['topic'], joints, 10)
        node.create_subscription(Odometry, robot['differential_drive']['odom_topic'],
                                 lambda m: received('odom', m), 10)
        node.create_subscription(SafeVelocityCommand, control['command_topic'], commands.append, 100)
        node.create_subscription(Twist, robot['differential_drive']['topic'],
                                 lambda m: outputs.append((now(), m.linear.x, m.angular.z)), 100)
        if args.visualization:
            node.create_subscription(MarkerArray, '/tracking/markers', marker_samples.append, 10)
        for name in names:
            topic = robot['joint_position_topic_template'].format(model=robot['entity_name'], joint=name)
            node.create_subscription(Float64, topic,
                                     lambda m, key=name: references.update({key: m.data}), 10)

        def wait(predicate, label, wall_limit=90):
            deadline = time.monotonic() + wall_limit
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
                if predicate():
                    print(label, flush=True)
                    return
                if process.poll() is not None:
                    raise RuntimeError('Launch exited')
            raise RuntimeError('Timed out: ' + label)

        def spin(duration):
            end = now() + duration
            wait(lambda: now() >= end, f'advanced {duration}s')

        def stopped():
            return bool(outputs) and max(abs(value) for value in outputs[-1][1:]) < 1e-12

        def flowing():
            return bool(commands) and commands[-1].active and not stopped()

        wait(lambda: 'joints' in latest and 'odom' in latest and now() >= 3., 'Complete feedback')
        if args.case == 'smoke':
            results['publication_disabled'] = not commands and stopped()
            assert results['publication_disabled']
            start_q = latest['joints'].copy()
            start_pose = latest['odom'].pose.pose
            start_xy = np.array([start_pose.position.x, start_pose.position.y])
            start_rotation = start_pose.orientation.z
            publisher = node.create_publisher(SafeVelocityCommand, control['command_topic'], 10)
            sequence = 0
            def publish():
                nonlocal sequence
                sequence += 1
                message = SafeVelocityCommand()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = robot['differential_drive']['child_frame_id']
                message.session_id = f'smoke-{os.getpid()}'
                message.sequence = sequence
                message.joint_names = names
                # Exercise every axis well within the configured operating limits.
                message.velocity = [.25 * control['base_velocity_upper'][0],
                                    .25 * control['base_velocity_upper'][1]] + [
                                        .15 * joint['velocity'] for joint in limits.values()]
                message.active = True
                publisher.publish(message)
            timer = node.create_timer(.05, publish)
            spin(3.)
            node.destroy_timer(timer)
            end_pose = latest['odom'].pose.pose
            results.update(joint_response=np.abs(latest['joints'] - start_q).tolist(),
                           base_translation=float(np.linalg.norm(np.array([
                               end_pose.position.x, end_pose.position.y]) - start_xy)),
                           base_quaternion_z_change=abs(end_pose.orientation.z - start_rotation))
            assert min(results['joint_response']) > thresholds['min_joint_smoke_response']
            assert results['base_translation'] > thresholds['min_base_excursion']
            assert results['base_quaternion_z_change'] > .01
            spin(.6)
            held = dict(references)
            spin(.3)
            results['watchdog_stopped'] = stopped()
            results['arm_reference_held'] = held == references
            results['physical_base_stopped'] = abs(latest['odom'].twist.twist.linear.x) < thresholds['max_stopped_base_speed']
            assert all(results[key] for key in ['watchdog_stopped', 'arm_reference_held', 'physical_base_stopped'])
        else:
            wait(flowing, 'Tracking active')
            def child(executable):
                matches = [child for child in psutil.Process(process.pid).children(recursive=True)
                           if child.cmdline() and any(arg.endswith('/' + executable)
                                                     for arg in child.cmdline()[:2])]
                if len(matches) != 1:
                    raise RuntimeError(f'Expected exactly one owned {executable}: {matches}')
                return matches[0]
            for executable, reason, timeout in [('control_node', 'command_timeout', .6),
                                                ('prediction_node', 'stale_prediction', .8),
                                                ('estimation_node', 'stale_robot_feedback', .6)]:
                component = child(executable)
                component.send_signal(signal.SIGSTOP)
                paused.append(component)
                spin(timeout)
                wait(stopped, reason + ' base stop')
                held = dict(references)
                spin(.3)
                results[reason] = dict(base_command_zero=stopped(), arm_reference_held=held == references,
                                       measured_base_speed=latest['odom'].twist.twist.linear.x)
                assert results[reason]['base_command_zero'] and results[reason]['arm_reference_held']
                assert abs(results[reason]['measured_base_speed']) < thresholds['max_stopped_base_speed']
                if executable != 'control_node':
                    assert commands and not commands[-1].active
                component.send_signal(signal.SIGCONT)
                paused.remove(component)
                wait(flowing, reason + ' recovered')
                results[reason]['recovered'] = True
            session_before = commands[-1].session_id
            time_before = now()
            world = ET.parse(ROOT / 'src/robot_simulation/worlds/tracking_testbed.sdf.xacro').getroot().find('world').get('name')
            reset = subprocess.run(['gz', 'service', '-s', f'/world/{world}/control',
                                    '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                                    '--timeout', '5000', '--req', 'reset: {all: true}'],
                                   capture_output=True, text=True, timeout=10)
            assert reset.returncode == 0 and 'true' in reset.stdout
            wait(lambda: now() < time_before, 'Clock reset observed')
            wait(lambda: commands and commands[-1].active and
                 commands[-1].session_id != session_before and flowing(), 'Reset recovered')
            if args.visualization:
                wait(lambda: marker_samples and len(marker_samples[-1].markers) == 12 and
                     marker_samples[-1].markers[0].header.stamp.sec < time_before,
                     'Physical and inferred tracking markers recovered after reset')
                results['visualization_reset_recovered'] = True
            results['clock_reset'] = dict(recovered=True, new_session=commands[-1].session_id != session_before)
            # Replay an old session as the sole publisher, with a fresh stamp:
            # this exercises epoch identity rather than just freshness/arbitration.
            component = child('control_node')
            restart_args = component.cmdline()
            component.send_signal(signal.SIGINT)
            wait(lambda: not node.get_publishers_info_by_topic(control['command_topic']),
                 'Controller shut down gracefully')
            wait(stopped, 'Graceful controller shutdown stopped base')
            replay = node.create_publisher(SafeVelocityCommand, control['command_topic'], 10)
            replay_sequence = 1_000_000
            def publish_replay():
                nonlocal replay_sequence
                replay_sequence += 1
                message = SafeVelocityCommand()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = robot['differential_drive']['child_frame_id']
                message.session_id = session_before
                message.sequence = replay_sequence
                message.joint_names = names
                message.velocity = [.1 * value for value in control['base_velocity_upper']] + [
                    .1 * joint['velocity'] for joint in limits.values()]
                message.active = True
                replay.publish(message)
            replay_timer = node.create_timer(.05, publish_replay)
            spin(.6)
            results['retired_session_replay_blocked'] = stopped()
            assert results['retired_session_replay_blocked']
            node.destroy_timer(replay_timer)
            node.destroy_publisher(replay)
            wait(lambda: not node.get_publishers_info_by_topic(control['command_topic']),
                 'Replay publisher removed')
            restarts.append(subprocess.Popen(restart_args, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
            wait(flowing, 'Fresh controller session recovered')
            results['controller_restart_recovered'] = True
            # After startup/holding, both output and reference paths exist once each.
            results['single_actuator_owner'] = all(
                len(node.get_publishers_info_by_topic(topic)) == 1 for topic in
                [robot['differential_drive']['topic']] + [
                    robot['joint_position_topic_template'].format(model=robot['entity_name'], joint=name)
                    for name in names])
            assert results['single_actuator_owner']
        lower = np.array([joint['lower'] for joint in limits.values()])
        upper = np.array([joint['upper'] for joint in limits.values()])
        measured = np.array([row[1] for row in samples])
        results['physical_joint_positions_within_limits'] = bool(
            np.all(measured >= lower) and np.all(measured <= upper))
        assert results['physical_joint_positions_within_limits']
        results['launch_command'] = launch
        if args.visualization:
            valid = [message for message in marker_samples if len(message.markers) == 12]
            results['visualization_samples'] = len(valid)
            assert valid
            (args.output / 'marker_sample.json').write_text(json.dumps([
                {'name': marker.ns, 'frame': marker.header.frame_id,
                 'origin': [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
                 'points': [[point.x, point.y, point.z] for point in marker.points]}
                for marker in valid[-1].markers], indent=2) + '\n')
        (args.output / 'result.json').write_text(json.dumps(results, indent=2) + '\n')
        (args.output / 'base_commands.json').write_text(json.dumps(outputs))
        print(json.dumps(results, indent=2), flush=True)
    finally:
        for component in paused:
            component.send_signal(signal.SIGCONT)
        for restarted in restarts:
            if restarted.poll() is None:
                os.killpg(restarted.pid, signal.SIGINT)
                try:
                    restarted.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(restarted.pid, signal.SIGTERM)
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
