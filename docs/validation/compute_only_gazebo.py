"""Manual live verification; no actuator output from robot_control.

Run after sourcing ROS Jazzy and the workspace. This verification fixture
uses an isolated ROS domain/Gazebo partition and cleans up its own processes.
The robot, bridges and effort holding come from the normal permanent
simulation launch. This script only observes and tests that launch.
"""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_interfaces.msg import StateEstimate, PredictedTrajectory
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import yaml

from robot_control.ros.control_node import ControlNode
from robot_estimation.estimation_node import EstimationNode
from robot_prediction.prediction_node import PredictionNode
from robot_bringup.model_factory import create_robot_model
from robot_models.whole_body.kinematics import WholeBodyKinematics

ROOT = Path(__file__).resolve().parents[2]
OUT = Path('/tmp/control-runtime')
os.environ['ROS_DOMAIN_ID'] = '79'
os.environ['GZ_PARTITION'] = 'control-compute-verification'
os.environ['ROS_LOG_DIR'] = str(OUT / 'ros-log')
os.environ['GZ_IP'] = '127.0.0.1'
processes = []
files = []
nodes = []
executor = None
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--gui', action='store_true', help='Verify the default GUI launch as well')
options = parser.parse_args()


def start(name, args):
    log = (OUT / (name + '.log')).open('w')
    files.append(log)
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)
    return process


def run(args, timeout=15):
    result = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
    (OUT / 'commands.log').open('a').write(str(args) + '\n' + result.stdout + result.stderr + '\n')
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result


def spin_for(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


def wait_until(predicate, label, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            print(label, flush=True)
            return
    raise RuntimeError('Timed out: ' + label)


try:
    OUT.mkdir(parents=True, exist_ok=True)
    robot = yaml.safe_load((ROOT / 'src/robot_description/config/robot/robot.yaml').read_text())['robot']
    deployment = yaml.safe_load((ROOT / 'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
    hold = yaml.safe_load((ROOT / 'src/robot_simulation/config/gazebo/joint_position_hold.yaml').read_text())['joint_position_hold']
    joint_names = [deployment['joint_name_template'].format(key=key) for key in robot['manipulator']['joints']]
    targets = np.array([hold['positions'][key] for key in robot['manipulator']['joints']])
    model = create_robot_model()
    arm_samples = []
    def sample_arm(message):
        index = [list(message.name).index(name) for name in joint_names]
        positions = np.array(message.position)[index]
        velocities = np.array(message.velocity)[index]
        stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        pose = WholeBodyKinematics.compute(robot_model=model, transform_world_base=np.eye(4),
                                           joint_positions=positions)
        arm_samples.append({'time': stamp, 'positions': positions.tolist(),
                            'velocities': velocities.tolist(), 'tracking_height_above_base': pose[2, 3]})
    launch_command = ['ros2', 'launch', 'robot_simulation', 'simulation.launch.py']
    if not options.gui:
        launch_command.append('gui:=false')
    start('simulation', launch_command)
    start('static-tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                      '--z', str(deployment['pose'][2]), '--frame-id', 'world', '--child-frame-id', 'odom',
                      '--ros-args', '-p', 'use_sim_time:=true'])
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true',
                    '--params-file', str(ROOT / 'src/robot_estimation/config/robot_estimator.yaml'),
                    '--params-file', str(ROOT / 'src/robot_estimation/config/target_estimator.yaml'),
                    '--params-file', str(ROOT / 'src/robot_control/config/control.yaml')])
    estimation = EstimationNode()
    prediction = PredictionNode()
    control = ControlNode()
    observer = Node('control_verification_observer')
    nodes = [estimation, prediction, control, observer]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    seen = {key: 0 for key in ['wheel', 'joints', 'target', 'robot', 'prediction']}
    latest = {}
    def received(key, message):
        seen[key] += 1
        latest[key] = message
        if key == 'joints':
            sample_arm(message)
    for msg_type, topic, key in [
        (Odometry, '/wheel/odometry', 'wheel'), (JointState, '/joint_states', 'joints'),
        (PointStamped, '/perception/target_position', 'target'),
        (StateEstimate, '/estimation/robot_state_full', 'robot'),
        (PredictedTrajectory, '/prediction/trajectory', 'prediction'),
    ]:
        observer.create_subscription(msg_type, topic, lambda message, key=key: received(key, message), 10)
    results = []
    original_compute = control._pipeline.compute
    lower = np.array([-0.5, -1., -1., -1., -1., -1.5])
    upper = -lower
    joint_lower = np.array([v['lower'] for v in robot['manipulator']['joints'].values()])
    joint_upper = np.array([v['upper'] for v in robot['manipulator']['joints'].values()])
    def checked_compute(**kwargs):
        result = original_compute(**kwargs)
        assert result.raw_output.command.shape == (6,)
        assert result.safe_output.command.shape == (6,)
        assert np.all(np.isfinite(result.raw_output.command))
        assert np.all(np.isfinite(result.safe_output.command))
        safe = result.safe_output.command
        assert np.all(safe >= lower - 1e-12) and np.all(safe <= upper + 1e-12)
        next_joints = kwargs['robot_state'].configuration[3:] + kwargs['dt'] * safe[2:]
        assert np.all(next_joints >= joint_lower - 1e-12)
        assert np.all(next_joints <= joint_upper + 1e-12)
        results.append({'time': kwargs['now'], 'raw': result.raw_output.command.tolist(), 'safe': safe.tolist()})
        return result
    control._pipeline.compute = checked_compute
    wait_until(lambda: observer.get_clock().now().nanoseconds > 0,
               'Gazebo clock received', timeout=60)
    wait_until(lambda: len(results) >= 100, '100 safety-checked live control cycles')
    print('Live topics: ' + json.dumps(seen), flush=True)
    # Measure a sustained interval after startup settling, with real gravity
    # and effort-limited PID controllers, not static/kinematic links.
    wait_until(lambda: arm_samples and arm_samples[-1]['time'] >= 35.0,
               '35 simulated seconds of permanent arm holding', timeout=180)
    settled = [sample for sample in arm_samples if 5.0 <= sample['time'] <= 35.0]
    errors = np.abs(np.array([sample['positions'] for sample in settled]) - targets)
    speeds = np.abs(np.array([sample['velocities'] for sample in settled]))
    heights = np.array([sample['tracking_height_above_base'] for sample in settled])
    stability = {'samples': len(settled), 'start_time': settled[0]['time'],
                 'end_time': settled[-1]['time'], 'max_position_error_rad': np.max(errors),
                 'max_joint_speed_rad_s': np.max(speeds),
                 'min_tracking_height_above_base_m': np.min(heights), 'targets_rad': targets.tolist()}
    stability['max_position_errors_rad'] = np.max(errors, axis=0).tolist()
    stability['max_joint_speeds_rad_s'] = np.max(speeds, axis=0).tolist()
    (OUT / 'arm-stability.json').write_text(json.dumps(stability, indent=2))
    print('Arm stability: ' + json.dumps(stability), flush=True)
    assert np.max(errors) < 0.06, f'Arm failed to hold configured pose: {np.max(errors)}'
    assert np.max(speeds) < 0.15, f'Arm remained unstable: {np.max(speeds)}'
    assert np.min(heights) > robot['body']['height'] / 2.0
    for subscription in list(estimation.subscriptions):
        if subscription.topic_name == '/joint_states':
            estimation.destroy_subscription(subscription)
    spin_for(0.8)
    count = len(results)
    spin_for(1.0)
    assert len(results) == count, 'Stale joint data still computed commands'
    print('Stale joint input blocked computation while wheel feedback continued', flush=True)
    estimation.create_subscription(JointState, '/joint_states', estimation._joint_state_callback, 10)
    wait_until(lambda: len(results) >= count + 40, 'Control recovered after robot feedback resumed')
    count = len(results)
    old_time = results[-1]['time']
    run(['gz', 'service', '-s', '/world/tracking_testbed/control',
         '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', 'reset: {all: true}'])
    spin_for(0.5)
    scene = run(['gz', 'service', '-s', '/world/tracking_testbed/scene/info',
                 '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
                 '--timeout', '5000', '--req', ''])
    assert 'name: "' + deployment['entity_name'] + '"' in scene.stdout, 'Permanent robot lost on reset'
    wait_until(lambda: len(results) >= count + 60 and any(r['time'] < old_time for r in results[count:]),
               'Control recovered after actual Gazebo reset')
    wait_until(lambda: arm_samples and 5.0 <= arm_samples[-1]['time'] < old_time,
               'Raised arm settled after world reset', timeout=60)
    post_reset_arm = arm_samples[-1]
    assert np.max(np.abs(np.array(post_reset_arm['positions']) - targets)) < 0.06
    assert np.max(np.abs(post_reset_arm['velocities'])) < 0.15
    assert post_reset_arm['tracking_height_above_base'] > robot['body']['height'] / 2.0
    report = {'status': 'passed', 'topics': seen, 'cycles': len(results),
              'permanent_launch': True, 'gui': options.gui, 'launch_command': launch_command,
              'arm_stability': stability,
              'post_reset_arm': post_reset_arm,
              'first': results[0], 'last': results[-1],
              'robot_state_dimension': len(latest['robot'].mean),
              'joint_names': list(latest['joints'].name),
              'publish_commands': control.get_parameter('publish_commands').value,
              'control_publishers': [publisher.topic_name for publisher in control.publishers]}
    (OUT / 'result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
finally:
    if executor is not None:
        executor.shutdown()
    for node in reversed(nodes):
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=8)
    for log in files:
        log.close()
