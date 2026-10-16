"""Short isolated performance observations of the production v1 launch.

Only temporary deployment configurations change between experiments. Optional
actuator counters wrap existing callbacks without changing their decisions.
"""

import argparse
from collections import Counter, defaultdict
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from robot_configuration.yaml_loader import load_yaml
from robot_interfaces.msg import (ControlTelemetry, SafeVelocityCommand,
                                  StateEstimate, PredictedTrajectory)
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', required=True, type=int)
    parser.add_argument('--threads', choices=['default', 'one'], default='default')
    parser.add_argument('--step', type=float, default=.001)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--warmup', type=float, default=7.)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--hold', action='store_true')
    parser.add_argument('--instrument', action='store_true')
    parser.add_argument('--profile-estimation', action='store_true')
    parser.add_argument('--relay', action='store_true',
                        help='Temporary measurement: 200 Hz clock / 100 Hz joints in sim time')
    parser.add_argument('--joint-relay', action='store_true',
                        help='Temporary measurement: only 100 Hz joint states')
    parser.add_argument('--estimator-cadence', action='store_true',
                        help='Temporary measurement: admit joint EKF updates at 100 Hz')
    parser.add_argument('--references', action='store_true',
                        help='Also observe four position-command topics (additional observer load)')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), GZ_PARTITION=f'v2-performance-{os.getpid()}',
               GZ_IP='127.0.0.1', ROS_LOG_DIR=str(args.output / 'ros-log'))
    if args.threads == 'one':
        env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    os.environ.update(env)
    launch = ['ros2', 'launch', 'robot_bringup', 'closed_loop.launch.py',
              'enable_actuation:=true', 'gui:=false']
    if args.hold:
        launch.append('publish_commands:=false')
    # Explicit override survives scenario inheritance and template changes.
    simulation_source = ROOT / 'src/robot_simulation/config/scenarios/moving_target.yaml'
    simulation = load_yaml(simulation_source)
    simulation['world'].setdefault('physics', {})['max_step_size'] = args.step
    simulation['paths']['base'] = 'package'
    simfile = args.output / 'simulation.yaml'
    simfile.write_text(yaml.safe_dump(simulation))
    bringup = load_yaml(ROOT / 'src/robot_bringup/config/closed_loop.yaml')
    bringup['simulation_config'] = str(simfile)
    config = args.output / 'bringup.yaml'
    config.write_text(yaml.safe_dump(bringup))
    launch.append(f'config:={config}')
    if args.instrument or args.profile_estimation or args.relay or args.joint_relay or args.estimator_cadence:
        code = (ROOT / 'src/robot_bringup/launch/closed_loop.launch.py').read_text()
        if args.relay or args.joint_relay:
            simulation_launch = (ROOT / 'src/robot_simulation/launch/simulation.launch.py').read_text()
            remaps = [("/joint_states", "/v2/raw_joints")]
            if args.relay:
                remaps.append(("/clock", "/v2/raw_clock"))
            simulation_launch = simulation_launch.replace('executable="parameter_bridge",',
                f'executable="parameter_bridge", remappings={remaps!r},')
            simlaunch = args.output / 'observed_simulation.launch.py'
            simlaunch.write_text(simulation_launch)
            code = code.replace("PythonLaunchDescriptionSource(str(share('robot_simulation') /\n                                              'launch/simulation.launch.py'))",
                                f"PythonLaunchDescriptionSource('{simlaunch}')")
        if args.instrument:
            wrapper = args.output / 'actuation_counters.py'
            wrapper.write_text('''import atexit, collections, json, runpy, sys
from pathlib import Path
from robot_simulation.nodes.velocity_actuation import VelocityActuationNode as C
counts=collections.Counter()
command=C._command_callback
halt=C._halt
def observe_command(self,msg):
    counts['incoming']+=1
    command(self,msg)
    counts['accepted' if self._command is not None else ('inactive' if not msg.active else 'rejected')]+=1
def observe_halt(self,reason):
    counts['halt:'+reason]+=1
    halt(self,reason)
C._command_callback=observe_command
C._halt=observe_halt
atexit.register(lambda destination=Path(__file__).with_name('actuation_counts.json'),counter=counts,dumps=json.dumps:destination.write_text(dumps(dict(counter),indent=2)))
sys.argv=sys.argv[1:]
runpy.run_path(sys.argv[0],run_name='__main__')
''')
            code = code.replace("Node(package='robot_simulation', executable='velocity_actuation', output='screen',",
                                f"Node(package='robot_simulation', executable='velocity_actuation', output='screen', prefix='python3 {wrapper}',")
        if args.profile_estimation:
            code = code.replace("Node(package='robot_estimation', executable='estimation_node', output='screen',",
                                f"Node(package='robot_estimation', executable='estimation_node', output='screen', prefix='python3 -m cProfile -o {args.output / 'estimation.prof'}',")
        if args.estimator_cadence:
            wrapper = args.output / 'estimation_cadence.py'
            wrapper.write_text('''import runpy, sys
from robot_estimation.nodes.estimation_node import EstimationNode as C
original=C._joint_state_callback
def gated(self,msg):
    timestamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    previous=getattr(self,'_benchmark_joint_time',None)
    if previous is not None and timestamp>=previous and timestamp-previous<.01-1e-9:
        return
    original(self,msg)
    if self._last_joint_time==timestamp:
        self._benchmark_joint_time=timestamp
C._joint_state_callback=gated
sys.argv=sys.argv[1:]
runpy.run_path(sys.argv[0],run_name='__main__')
''')
            code = code.replace("Node(package='robot_estimation', executable='estimation_node', output='screen',",
                                f"Node(package='robot_estimation', executable='estimation_node', output='screen', prefix='python3 {wrapper}',")
        launchfile = args.output / 'observed.launch.py'
        launchfile.write_text(code)
        launch[2:4] = [str(launchfile)]
    logfile = (args.output / 'runtime.log').open('w')
    process = subprocess.Popen(launch, env=env, stdout=logfile, stderr=subprocess.STDOUT,
                               start_new_session=True)
    owned = [process]
    counts, blocked = Counter(), Counter()
    arrivals, stamps = defaultdict(list), defaultdict(list)
    samples, cpu, latest = [], [], {}
    node = None
    begin = time.monotonic()
    cpu_processes = {}
    try:
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = Node('v2_performance_observer')
        if args.relay or args.joint_relay:
            relay_clock = node.create_publisher(Clock, '/clock', 100)
            relay_joints = node.create_publisher(JointState, '/joint_states', 100)
            last_forwarded = {'clock': -1., 'joints': -1.}
            def relay(key, message):
                timestamp = message.clock if key == 'clock' else message.header.stamp
                stamp = timestamp.sec + timestamp.nanosec * 1e-9
                period = .005 if key == 'clock' else .01
                previous = last_forwarded[key]
                if stamp < previous or stamp - previous >= period - 1e-9:
                    (relay_clock if key == 'clock' else relay_joints).publish(message)
                    last_forwarded[key] = stamp
            if args.relay:
                node.create_subscription(Clock, '/v2/raw_clock', lambda m: relay('clock', m), 100)
            node.create_subscription(JointState, '/v2/raw_joints', lambda m: relay('joints', m), 100)
        def receive(key, msg):
            wall = time.monotonic() - begin
            latest[key] = msg
            if wall < args.warmup:
                return
            counts[key] += 1
            arrivals[key].append(wall)
            if hasattr(msg, 'header'):
                stamps[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            if key == 'control' and not msg.valid:
                blocked[msg.reason] += 1
        subscriptions = [
            (Clock, '/clock', 'clock'), (JointState, '/joint_states', 'joints'),
            (Odometry, '/wheel/odometry', 'odometry'),
            (StateEstimate, '/estimation/robot_state_full', 'robot_estimate'),
            (StateEstimate, '/estimation/target_state_full', 'target_estimate'),
            (PredictedTrajectory, '/prediction/trajectory', 'prediction'),
            (SafeVelocityCommand, '/control/safe_velocity', 'command'),
            (ControlTelemetry, '/control/telemetry', 'control'),
            (Twist, '/simulation/robot/cmd_vel', 'base_command'),
            (PointStamped, '/simulation/target_position', 'target'),
        ]
        for msgtype, topic, key in subscriptions:
            node.create_subscription(msgtype, topic, lambda msg, k=key: receive(k, msg), 100)
        if args.references:
            robot = yaml.safe_load((ROOT / 'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
            physical = yaml.safe_load((ROOT / 'src/robot_description/config/robot/robot.yaml').read_text())['robot']['manipulator']['joints']
            for joint_key in physical:
                name = robot['joint_name_template'].format(key=joint_key)
                topic = robot['joint_position_topic_template'].format(model=robot['entity_name'], joint=name)
                node.create_subscription(Float64, topic, lambda m, k=name: receive('reference:'+k, m), 100)
        next_cpu, gui_started = 0., False
        while time.monotonic() - begin < args.warmup + args.seconds:
            rclpy.spin_once(node, timeout_sec=.001)
            wall = time.monotonic() - begin
            if process.poll() is not None:
                raise RuntimeError('Launch exited; see runtime.log')
            if args.gui and not gui_started and wall > args.warmup:
                owned.append(subprocess.Popen(['gz', 'sim', '-g', '--force-version', '8'], env=env,
                                              stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True))
                gui_started = True
            if wall >= next_cpu:
                next_cpu = wall + 1.
                descendants = psutil.Process(process.pid).children(recursive=True)
                for owner in owned[1:]:
                    if owner.poll() is None:
                        descendants += [psutil.Process(owner.pid)] + psutil.Process(owner.pid).children(recursive=True)
                for child in descendants:
                    try:
                        if child.pid not in cpu_processes:
                            cpu_processes[child.pid] = child
                            child.cpu_percent()
                        cmd = child.cmdline()
                        cpu.append(dict(wall=wall, pid=child.pid, command=' '.join(cmd)[:400],
                                        percent=cpu_processes[child.pid].cpu_percent(), threads=child.num_threads(),
                                        rss=child.memory_info().rss))
                    except psutil.Error:
                        pass
                now = node.get_clock().now().nanoseconds * 1e-9
                row = dict(wall=wall, sim=now, counts=dict(counts), load=os.getloadavg())
                joints = latest.get('joints')
                if joints is not None:
                    row.update(joint_names=list(joints.name), joint_position=list(joints.position),
                               joint_velocity=list(joints.velocity))
                command = latest.get('command')
                if command is not None:
                    row.update(active=command.active, safe_velocity=list(command.velocity))
                telemetry = latest.get('control')
                if telemetry is not None:
                    row.update(control_valid=telemetry.valid, control_reason=telemetry.reason,
                               raw_velocity=list(telemetry.raw_velocity),
                               control_safe_velocity=list(telemetry.safe_velocity))
                if args.references:
                    row['position_references'] = {key.partition(':')[2]:m.data for key,m in latest.items() if key.startswith('reference:')}
                base = latest.get('base_command')
                if base is not None:
                    row['base_command'] = [base.linear.x, base.angular.z]
                odom = latest.get('odometry')
                if odom is not None:
                    row['base_position'] = [odom.pose.pose.position.x, odom.pose.pose.position.y]
                samples.append(row)
        stable = [row for row in samples if row['wall'] >= args.warmup]
        elapsed = stable[-1]['wall'] - stable[0]['wall']
        simulated = stable[-1]['sim'] - stable[0]['sim']
        rates = {}
        for key, times in arrivals.items():
            gaps = np.diff(times)
            rates[key] = dict(count=counts[key], wall_hz=len(times) / args.seconds,
                              sim_hz=len(times) / simulated if simulated > 0 else 0.,
                              wall_gap_max=float(gaps.max()) if len(gaps) else None,
                              wall_gap_p95=float(np.quantile(gaps, .95)) if len(gaps) else None)
        cpu_summary = {}
        for pid in cpu_processes:
            rows = [row for row in cpu if row['pid'] == pid and row['wall'] >= args.warmup]
            if rows:
                cpu_summary[str(pid)] = dict(command=rows[-1]['command'],
                    mean_cpu=float(np.mean([r['percent'] for r in rows])),
                    peak_cpu=max(r['percent'] for r in rows), max_threads=max(r['threads'] for r in rows))
        result = dict(command=launch, domain=args.domain, partition=env['GZ_PARTITION'],
                      threads=args.threads, step=args.step, gui=args.gui, hold=args.hold,
                      instrument=args.instrument, profile_estimation=args.profile_estimation,
                      relay=args.relay,
                      joint_relay=args.joint_relay,
                      estimator_cadence=args.estimator_cadence,
                      references=args.references,
                      warmup_wall=args.warmup, duration_wall=elapsed, simulated_duration=simulated,
                      rtf=simulated / elapsed, rates=rates, blocked=dict(blocked), cpu=cpu_summary,
                      environment={k:env.get(k) for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','DISPLAY','WAYLAND_DISPLAY']})
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        (args.output / 'samples.json').write_text(json.dumps(samples, indent=2) + '\n')
        (args.output / 'cpu.json').write_text(json.dumps(cpu, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for owner in reversed(owned):
            for sig, timeout in [(signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)]:
                try:
                    os.killpg(owner.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    owner.wait(timeout=timeout)
                    if sig != signal.SIGKILL:
                        try:
                            os.killpg(owner.pid, signal.SIGTERM)
                        except ProcessLookupError:
                            pass
                    break
                except subprocess.TimeoutExpired:
                    pass
        logfile.close()


if __name__ == '__main__':
    main()
