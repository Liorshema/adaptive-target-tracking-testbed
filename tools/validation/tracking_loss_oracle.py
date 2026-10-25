"""Explicit test-only current-state relay; no actuator publisher or future truth."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from robot_interfaces.msg import StateEstimate, PredictedTrajectory
from robot_configuration.yaml_loader import load_yaml
from tracking_loss_geometry import History, pose, oracle_mean, wrap

class Oracle(Node):
    def __init__(self, path):
        super().__init__('tracking_loss_diagnostic_oracle',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.cfg=load_yaml(path);assert self.cfg['diagnostic_only'] is True
        self.base=[];self.previous_target=None;self.last_emit=-np.inf;self.last_robot=-np.inf
        self.log=Path(self.cfg['log']).open('w');self.topics=self.cfg['context']['topics'];self.mode=self.cfg['mode']
        self.spawn=pose(self.cfg['spawn'][:3],self.cfg['spawn_quaternion'])
        self.create_timer(1.,self.audit)
        if self.mode['robot_truth']:
            assert self.cfg['robot_output']!=self.topics['robot_estimate']
            self.robot_pub=self.create_publisher(StateEstimate,self.cfg['robot_output'],10)
            self.create_subscription(Odometry,self.topics['base_truth'],self.base_callback,qos_profile_sensor_data)
            self.create_subscription(StateEstimate,self.topics['robot_estimate'],self.robot_callback,qos_profile_sensor_data)
        if self.mode['target_truth']:
            assert self.cfg['prediction_output']!=self.topics['prediction']
            self.target_pub=self.create_publisher(PredictedTrajectory,self.cfg['prediction_output'],10)
            self.create_subscription(PointStamped,self.topics['target_truth'],self.target_callback,qos_profile_sensor_data)
    def audit(self):
        topics=[self.cfg['robot_output']] if self.mode['robot_truth'] else []
        if self.mode['target_truth']:topics.append(self.cfg['prediction_output'])
        self.log.write(json.dumps({'kind':'interface_audit','topics':{
            topic:{'publishers':[x.node_name for x in self.get_publishers_info_by_topic(topic)],
                   'subscribers':[x.node_name for x in self.get_subscriptions_info_by_topic(topic)]}
            for topic in topics}})+'\n')
    def stamp(self,m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def base_callback(self,m):
        t=self.stamp(m)
        if m.header.frame_id!=self.cfg['context']['world_frame']:return
        if self.base and t<self.stamp(self.base[-1]):self.base=[]
        self.base.append(m);self.base=[a for a in self.base if t-self.stamp(a)<=2.]
    def robot_callback(self,m):
        t=self.stamp(m)
        if t<self.last_robot:self.base=[]
        self.last_robot=t
        available=[a for a in self.base if self.stamp(a)<=t]
        if not available or t-self.stamp(available[-1])>self.cfg['max_truth_age']:return
        base=available[-1];p=base.pose.pose.position;q=base.pose.pose.orientation
        transform=pose([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);linear=base.twist.twist.linear.x;angular=base.twist.twist.angular.z
        out=deepcopy(m);out.mean=oracle_mean(m.mean,transform,self.spawn,linear,angular).tolist()
        if m.header.frame_id!=self.cfg['context']['estimator_frame']:return
        self.robot_pub.publish(out)
        self.log.write(json.dumps({'kind':'robot','t':t,'truth_t':self.stamp(base),'original_mean':list(m.mean),'oracle_mean':list(out.mean)})+'\n')
    def target_callback(self,m):
        t=self.stamp(m)
        if m.header.frame_id!=self.cfg['context']['world_frame']:return
        p=np.array([m.point.x,m.point.y,m.point.z])
        if self.previous_target is not None and t<=self.previous_target[0]:
            self.previous_target=None;self.last_emit=-np.inf
        v=np.zeros(3)
        if self.previous_target is not None:
            old_t,old_p=self.previous_target
            if t-old_t<=self.cfg['max_truth_age']:v=(p-old_p)/(t-old_t)
            else:self.previous_target=(t,p);return
        else:self.previous_target=(t,p);return
        self.previous_target=(t,p)
        if t-self.last_emit<self.cfg['target_publish_period']-1e-9:return
        out=PredictedTrajectory();out.header=m.header
        future=np.arange(1,self.cfg['horizon']+1)*self.cfg['dt']
        out.horizon=len(future);out.state_dimension=6;out.timestamps=(t+future).tolist()
        out.states=np.column_stack([p+future[:,None]*v,np.tile(v,(len(future),1))]).ravel().tolist()
        self.target_pub.publish(out);self.last_emit=t
        self.log.write(json.dumps({'kind':'target','t':t,'position':p.tolist(),'causal_velocity':v.tolist(),'forecast':'CV from current/past truth; zero controller lookahead'})+'\n')
    def destroy_node(self):
        self.log.close();super().destroy_node()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True);a,ros=parser.parse_known_args()
    rclpy.init(args=ros);node=Oracle(a.config)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
