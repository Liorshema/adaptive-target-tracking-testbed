"""Publish the tracking frame from measured base/joint feedback for reactive targets."""

from pathlib import Path

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import load_robot_model_parameters
from robot_models.whole_body.kinematics import WholeBodyKinematics
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def tracking_pose(model, odometry, joints, joint_names):
    """Compute tracking FK from actual feedback with the mount already in the model."""
    names = list(joints.name)
    if len(set(names)) != len(names):
        raise ValueError('Duplicate measured joint names')
    positions = np.asarray([joints.position[names.index(name)] for name in joint_names])
    pose = odometry.pose.pose
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w]).as_matrix()
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.all(np.isfinite(np.r_[transform.ravel(), positions])):
        raise ValueError('Nonfinite physical feedback')
    return WholeBodyKinematics.compute(
        robot_model=model, transform_world_base=transform, joint_positions=positions)


class TrackingFeedbackTF(Node):
    """Own the optional odom-to-tracking transform with bounded sensor time skew."""

    def __init__(self):
        """Receive configured measured feedback without introducing actuator owners."""
        super().__init__('tracking_feedback_tf')
        for key, kind in [
                ('joint_names', Parameter.Type.STRING_ARRAY),
                ('joint_topic', Parameter.Type.STRING), ('odom_topic', Parameter.Type.STRING),
                ('parent_frame', Parameter.Type.STRING), ('child_frame', Parameter.Type.STRING),
                ('max_age', Parameter.Type.DOUBLE), ('max_skew', Parameter.Type.DOUBLE),
                ('future_tolerance', Parameter.Type.DOUBLE), ('rate_hz', Parameter.Type.DOUBLE)]:
            self.declare_parameter(key, kind)
        self.declare_parameter('description_file', '')
        self._names = list(self.get_parameter('joint_names').value)
        self._parent = self.get_parameter('parent_frame').value
        self._child = self.get_parameter('child_frame').value
        description = self.get_parameter('description_file').value
        parameters = load_robot_model_parameters(Path(description)) if description else None
        self._model = create_robot_model(parameters)
        if (len(self._names) != self._model.manipulator.dof or
                len(set(self._names)) != len(self._names) or
                any(not name for name in self._names)):
            raise ValueError('TF joint names must match the configured robot')
        if not self._parent or not self._child or self._parent == self._child:
            raise ValueError('TF requires distinct nonempty parent and child frames')
        self._max_age = float(self.get_parameter('max_age').value)
        self._max_skew = float(self.get_parameter('max_skew').value)
        self._future_tolerance = float(self.get_parameter('future_tolerance').value)
        if (not np.all(np.isfinite([self._max_age, self._max_skew, self._future_tolerance])) or
                self._max_age <= 0 or min(self._max_skew, self._future_tolerance) < 0):
            raise ValueError('TF freshness tolerances must be finite and valid')
        self._latest = {}
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, self.get_parameter('joint_topic').value,
                                 lambda m: self._receive('joints', m), 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 lambda m: self._receive('odom', m), 10)
        self._jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True),
            post_callback=lambda _: self._latest.clear())
        rate = float(self.get_parameter('rate_hz').value)
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError('TF rate must be finite and positive')
        self._timer = self.create_timer(1 / rate, self._publish)

    def _receive(self, key, message):
        previous = self._latest.get(key)
        stamp = message.header.stamp
        now = self.get_clock().now().nanoseconds * 1e-9
        if stamp.sec + stamp.nanosec * 1e-9 > now + self._future_tolerance:
            return
        if previous is not None:
            old = previous.header.stamp
            if (stamp.sec, stamp.nanosec) < (old.sec, old.nanosec):
                return
        self._latest[key] = message

    def _publish(self):
        if not all(key in self._latest for key in ['joints', 'odom']):
            return
        joints, odom = self._latest['joints'], self._latest['odom']
        times = [message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                 for message in [joints, odom]]
        now = self.get_clock().now().nanoseconds * 1e-9
        if (odom.header.frame_id != self._parent or
                now - min(times) > self._max_age or
                max(times) > now + self._future_tolerance or
                abs(times[0] - times[1]) > self._max_skew):
            return
        try:
            transform = tracking_pose(self._model, odom, joints, self._names)
        except (ValueError, IndexError):
            return
        message = TransformStamped()
        message.header.stamp = Time(seconds=max(times)).to_msg()
        message.header.frame_id = self._parent
        message.child_frame_id = self._child
        translation = message.transform.translation
        translation.x, translation.y, translation.z = transform[:3, 3].tolist()
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat().tolist()
        rotation = message.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = quaternion
        self._broadcaster.sendTransform(message)


def main(args=None):
    """Run the optional measured tracking-frame broadcaster."""
    rclpy.init(args=args)
    node = None
    try:
        node = TrackingFeedbackTF()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
