"""Isolated observational estimate/odometry capture; changes no node settings."""
from robot_estimation.configuration import load_estimation_parameters
import json, os, signal, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from robot_interfaces.msg import StateEstimate
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import yaml
ROOT=Path('/home/user/ros2_ws')
OUT=ROOT/'results/validation/v2_estimation_audit'
OUT.mkdir(parents=True,exist_ok=True)
os.environ.update(ROS_DOMAIN_ID='121',GZ_PARTITION=f'v2-estimation-audit-{os.getpid()}',GZ_IP='127.0.0.1',ROS_LOG_DIR=str(OUT/'ros-log'))
robot=yaml.safe_load((ROOT/'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
estcfg=load_estimation_parameters(ROOT/'src/robot_estimation/config/estimation.yaml', 'robot_ekf')['estimation_node']['ros__parameters']
desc=yaml.safe_load((ROOT/'src/robot_description/config/robot/robot.yaml').read_text())['robot']
names=[robot['joint_name_template'].format(key=key) for key in desc['manipulator']['joints']]
streams={'estimate':[],'odom':[],'joint':[]}
def stamp(message):return message.header.stamp.sec+message.header.stamp.nanosec*1e-9
log=(OUT/'runtime.log').open('w')
command=['ros2','launch','robot_experiments','tracking.launch.py','gui:=false','enable_actuation:=true','controller:=whole_body']
proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
node=None
try:
 rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);node=Node('v2_estimation_observer')
 def estimate(msg):
  n=len(msg.mean); p=np.array(msg.covariance).reshape(n,n)
  streams['estimate'].append({'t':stamp(msg),'received':time.monotonic(),'sim_received':node.get_clock().now().nanoseconds*1e-9,'frame':msg.header.frame_id,'mean':list(msg.mean),'pdiag':np.diag(p).tolist()})
 def odom(msg):
  p=msg.pose.pose;q=p.orientation;yaw=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
  streams['odom'].append({'t':stamp(msg),'received':time.monotonic(),'sim_received':node.get_clock().now().nanoseconds*1e-9,'frame':msg.header.frame_id,'base':[p.position.x,p.position.y,float(yaw)],'velocity':[msg.twist.twist.linear.x,msg.twist.twist.angular.z]})
 def joint(msg):
  indices=[msg.name.index(name) for name in names]
  streams['joint'].append({'t':stamp(msg),'received':time.monotonic(),'sim_received':node.get_clock().now().nanoseconds*1e-9,'q':[msg.position[i] for i in indices],'dq':[msg.velocity[i] for i in indices]})
 node.create_subscription(StateEstimate,estcfg['robot_state_full_topic'],estimate,1000)
 node.create_subscription(Odometry,robot['differential_drive']['odom_topic'],odom,1000)
 node.create_subscription(JointState,robot['joint_states']['topic'],joint,1000)
 start=time.monotonic()
 while time.monotonic()-start<35:
  rclpy.spin_once(node,timeout_sec=.005)
  if proc.poll() is not None: raise RuntimeError('launch exited')
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(proc.pid,signal.SIGTERM)
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
 if node is not None:node.destroy_node();rclpy.shutdown()
 log.close()
 (OUT/'streams.json').write_text(json.dumps(streams))
print(json.dumps({'output':str(OUT),'counts':{k:len(v) for k,v in streams.items()},'last_times':{k:v[-1]['t'] if v else None for k,v in streams.items()}}),flush=True)
