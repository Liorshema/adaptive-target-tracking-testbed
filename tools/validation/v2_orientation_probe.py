"""Compare configured optical FK to Gazebo link poses and actual EKF callback order."""
from robot_control.configuration import load_control_parameters

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from robot_bringup.factory.model_factory import create_robot_model
from robot_interfaces.msg import StateEstimate
from robot_models.whole_body.kinematics import WholeBodyKinematics
from robot_configuration.yaml_loader import load_yaml
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[2]


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def matrix(position, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()
    result[:3, 3] = [position.x, position.y, position.z]
    return result


def angle(first, second):
    return float(np.degrees(np.arccos(np.clip(
        first @ second / (np.linalg.norm(first) * np.linalg.norm(second)), -1., 1.))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--visualization', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.update(GZ_PARTITION=f'orientation-{os.getpid()}', GZ_IP='127.0.0.1',
                      ROS_LOG_DIR=str(args.output / 'ros-log'),
                      ESTIMATION_TIMING_LOG=str(args.output.resolve() / 'processing.json'))
    os.environ.setdefault('ROS_DOMAIN_ID', '132')
    robot = load_yaml(ROOT / 'src/robot_simulation/config/gazebo/robot.yaml')['robot']
    description = load_yaml(ROOT / 'src/robot_description/config/robot/robot.yaml')['robot']
    control = load_control_parameters(ROOT / 'src/robot_control/config/control.yaml', 'parameters')['control_node']['ros__parameters']
    names = [robot['joint_name_template'].format(key=key) for key in description['manipulator']['joints']]
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler('xyz', description['tracking_frame']['rotation_rpy']).as_matrix()
    mount[:3, 3] = description['tracking_frame']['translation']
    spawn = np.eye(4)
    spawn[:3, :3] = Rotation.from_euler('xyz', robot['pose'][3:]).as_matrix()
    spawn[:3, 3] = robot['pose'][:3]
    model = create_robot_model()
    code = (ROOT / 'src/robot_bringup/launch/closed_loop.launch.py').read_text()
    wrapper = ROOT / 'tools/validation/estimator_timing_probe.py'
    code = code.replace('from pathlib import Path', 'from pathlib import Path\nimport sys')
    code = code.replace("Node(package='robot_estimation', executable='estimation_node', output='screen',",
                        "Node(package='robot_estimation', executable='estimation_node', output='screen', "
                        f"prefix=[sys.executable, ' ', '{wrapper}'],")
    launch = args.output / 'observed.launch.py'
    launch.write_text(code)
    command = ['ros2', 'launch', str(launch.resolve()), 'gui:=false',
               'case:=optimal_tracking', 'controller:=whole_body',
               'enable_actuation:=true']
    if args.visualization:
        command.append('visualize_tracking:=true')
    log = (args.output / 'runtime.log').open('w')
    processes = [subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)]
    scene_topic = '/world/tracking_testbed/pose/info'
    scene_log = (args.output / 'gazebo_poses.jsonl').open('w')
    processes.append(subprocess.Popen([
        'gz', 'topic', '-e', '-t', scene_topic, '--json-output'],
        stdout=scene_log, stderr=log, start_new_session=True))
    latest, rows, streams = {}, [], {'odom': [], 'joint': [], 'estimate': [], 'target': []}
    node = None
    try:
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = Node('orientation_observer')
        def receive(key, message):
            latest[key] = message
            if key in streams:
                item = dict(time=stamp(message), received=node.get_clock().now().nanoseconds * 1e-9)
                if key == 'estimate':
                    item['mean'] = list(message.mean)
                elif key == 'joint':
                    indices = [message.name.index(name) for name in names]
                    item['q'] = np.asarray(message.position)[indices].tolist()
                elif key == 'target':
                    item['position'] = [message.point.x, message.point.y, message.point.z]
                else:
                    pose = message.pose.pose
                    item['base'] = [pose.position.x, pose.position.y,
                                    Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                                       pose.orientation.z, pose.orientation.w]).as_euler('xyz')[2]]
                streams[key].append(item)
        for key, kind, topic in [('odom', Odometry, robot['differential_drive']['odom_topic']),
                                 ('joint', JointState, robot['joint_states']['topic']),
                                 ('estimate', StateEstimate, '/estimation/robot_state_full'),
                                 ('target', PointStamped, '/simulation/target_position'),
                                 ('measurement', PointStamped, '/perception/target_position')]:
            node.create_subscription(kind, topic, lambda message, key=key: receive(key, message), 100)
        def scene(message):
            if not all(key in latest for key in ['odom', 'joint', 'target']):
                return
            poses = {transform.child_frame_id: transform for transform in message.transforms}
            if not all(key in poses for key in [robot['entity_name'], description['tracking_frame']['parent']]):
                return
            parent = poses[description['tracking_frame']['parent']]
            simulated = stamp(parent)
            if rows and simulated - rows[-1]['time'] < .05:
                return
            if max(abs(simulated - stamp(latest[key])) for key in ['odom', 'joint', 'target']) > .08:
                return
            entity = poses[robot['entity_name']]
            physical = matrix(entity.transform.translation, entity.transform.rotation) @ matrix(
                parent.transform.translation, parent.transform.rotation) @ mount
            odom = latest['odom'].pose.pose
            joints = latest['joint']
            indices = [joints.name.index(name) for name in names]
            q = np.asarray(joints.position)[indices]
            modeled = spawn @ WholeBodyKinematics.compute(
                model, matrix(odom.position, odom.orientation), q)
            target = latest['target'].point
            target = np.array([target.x, target.y, target.z])
            direction = target - physical[:3, 3]
            optical = physical[:3, :3] @ np.asarray(control['tracking_forward_axis'])
            bounds = list(description['manipulator']['joints'].values())
            margins = np.minimum(q - [j['lower'] for j in bounds], [j['upper'] for j in bounds] - q)
            rows.append(dict(time=simulated, physical_pose=physical.tolist(), modeled_pose=modeled.tolist(),
                             optical_axis=optical.tolist(), arm_axis=physical[:3, 0].tolist(),
                             target=target.tolist(), target_direction=(direction / np.linalg.norm(direction)).tolist(),
                             pointing=angle(optical, direction), shaft_pointing=angle(physical[:3, 0], direction),
                             distance_error=float(np.linalg.norm(direction) - control['tracking_radius']),
                             model_position_error=float(np.linalg.norm(modeled[:3, 3] - physical[:3, 3])),
                             model_rotation_error=float(np.degrees(Rotation.from_matrix(
                                 modeled[:3, :3].T @ physical[:3, :3]).magnitude())),
                             q=q.tolist(), joint_margins=margins.tolist()))
        start = time.monotonic()
        while time.monotonic() - start < 180:
            rclpy.spin_once(node, timeout_sec=.005)
            if node.get_clock().now().nanoseconds * 1e-9 >= 35:
                break
            if processes[0].poll() is not None:
                raise RuntimeError('Production launch exited')
        else:
            raise RuntimeError('No complete Gazebo pose capture before timeout')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        log.close()
        scene_log.close()
        (args.output / 'streams.json').write_text(json.dumps(streams))
    # SceneBroadcaster link poses are model-relative; model poses are world-relative.
    # The generic Pose_V->TF bridge discards name/stamp here, so keep native JSON.
    from types import SimpleNamespace as S
    def vector(data):
        return S(**{key: float(data.get(key, 0.)) for key in ['x', 'y', 'z']})
    times = {key: np.array([entry['time'] for entry in entries])
             for key, entries in streams.items()}
    for line in (args.output / 'gazebo_poses.jsonl').read_text().splitlines():
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            continue
        sec = data['header']['stamp']
        timestamp = float(sec.get('sec', 0)) + float(sec.get('nsec', 0)) * 1e-9
        if timestamp < 1 or timestamp > 35:
            continue
        header = S(stamp=S(sec=int(timestamp), nanosec=round((timestamp-int(timestamp))*1e9)))
        for key in ['joint', 'odom', 'target']:
            index = int(np.argmin(np.abs(times[key] - timestamp)))
            entry = streams[key][index]
            local_header = S(stamp=S(sec=int(entry['time']), nanosec=round((entry['time']-int(entry['time']))*1e9)))
            if key == 'joint':
                latest[key] = S(header=local_header, name=names, position=entry['q'])
            elif key == 'target':
                latest[key] = S(header=local_header, point=vector(dict(zip(['x','y','z'],entry['position']))))
            else:
                x,y,yaw = entry['base']
                quaternion = Rotation.from_euler('z',yaw).as_quat()
                latest[key] = S(header=local_header, pose=S(pose=S(
                    position=S(x=x,y=y,z=0.), orientation=S(**dict(zip(['x','y','z','w'],quaternion))))))
        transforms = []
        for pose in data['pose']:
            rotation = pose.get('orientation', {})
            transforms.append(S(header=header, child_frame_id=pose['name'], transform=S(
                translation=vector(pose.get('position',{})),
                rotation=S(**{k:float(rotation.get(k,1. if k=='w' else 0.)) for k in ['x','y','z','w']}))))
        scene(S(transforms=transforms))
    (args.output / 'poses.json').write_text(json.dumps(rows))
    selected = [row for row in rows if row['time'] >= 10]
    summary = {key: {'mean': float(np.mean([row[key] for row in selected])),
                     'max': float(np.max([row[key] for row in selected]))}
               for key in ['pointing', 'shaft_pointing', 'model_position_error', 'model_rotation_error']}
    summary['mean_distance_error'] = float(np.mean([abs(row['distance_error']) for row in selected]))
    summary['snapshot'] = min(rows, key=lambda row: abs(row['time'] - 21.164))
    summary['samples'] = len(selected)
    summary['launch'] = command
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
