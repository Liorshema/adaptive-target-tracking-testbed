"""Display measured tracking FK, frame axes and optical direction using ROS markers."""

from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_bringup.factory.model_factory import create_robot_model
from robot_bringup.nodes.tracking_feedback_tf import tracking_pose
from robot_description.model_parameters import load_robot_model_parameters
from robot_configuration.yaml_loader import load_yaml
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def transform_matrix(transform):
    """Convert a TF transform to a homogeneous matrix."""
    position, rotation = transform.translation, transform.rotation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
    result[:3, 3] = [position.x, position.y, position.z]
    return result


def build_markers(pose, target, forward_axis, configuration, frame):
    """Draw the configured optical axis independently of the wrist's local +X."""
    pose, target, axis = np.asarray(pose), np.asarray(target), np.asarray(forward_axis)
    if (pose.shape != (4, 4) or target.shape != (3,) or axis.shape != (3,) or
            not np.all(np.isfinite(np.r_[pose.ravel(), target, axis])) or
            np.linalg.norm(axis) <= np.finfo(float).eps):
        raise ValueError('Invalid tracking visualization geometry')
    axis = axis / np.linalg.norm(axis)
    origin = pose[:3, 3]
    endpoints = {key: origin + configuration['axis_length'] * pose[:3, index]
                 for index, key in enumerate(['x', 'y', 'z'])}
    endpoints['optical'] = origin + configuration['forward_length'] * pose[:3, :3] @ axis
    endpoints['target'] = target
    result = MarkerArray()
    for name in ['origin', *endpoints]:
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = name
        marker.id = 0
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            configuration['colors'][name])
        marker.lifetime = Duration(seconds=configuration['lifetime']).to_msg()
        if name == 'origin':
            marker.type = Marker.SPHERE
            position = marker.pose.position
            position.x, position.y, position.z = origin.tolist()
            marker.scale.x = marker.scale.y = marker.scale.z = configuration['origin_size']
        else:
            marker.type = Marker.ARROW
            marker.scale.x = configuration['width']
            marker.scale.y = marker.scale.z = 2 * configuration['width']
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                             for p in [origin, endpoints[name]]]
        result.markers.append(marker)
    return result


class TrackingVisualization(Node):
    """Observe measured feedback without changing commands, model geometry or optical axes."""

    def __init__(self):
        """Load visual styling and receive deployment geometry from Bringup."""
        super().__init__('tracking_visualization')
        for name in ['configuration_file', 'description_file', 'joint_topic', 'odom_topic',
                     'reference_frame', 'physical_pose_topic']:
            self.declare_parameter(name, '')
        self.declare_parameter('joint_names', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('forward_axis', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self._configuration = load_yaml(self.get_parameter('configuration_file').value)
        cfg = self._configuration
        for name in ['axis_length', 'forward_length', 'width', 'origin_size', 'lifetime',
                     'rate_hz', 'max_age', 'max_skew']:
            if not np.isfinite(cfg[name]) or cfg[name] <= 0:
                raise ValueError(f'Visualization {name} must be finite and positive')
        self._model = create_robot_model(load_robot_model_parameters(
            self.get_parameter('description_file').value))
        self._names = self.get_parameter('joint_names').value
        self._axis = self.get_parameter('forward_axis').value
        self._frame = self.get_parameter('reference_frame').value
        self._latest = {}
        self._buffer = Buffer(node=self)
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(
            MarkerArray, cfg['topic'],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for name, kind, topic in [
                ('joint', JointState, self.get_parameter('joint_topic').value),
                ('odom', Odometry, self.get_parameter('odom_topic').value),
                ('physical', Odometry, self.get_parameter('physical_pose_topic').value),
                ('target', PointStamped, cfg['target_topic'])]:
            self.create_subscription(
                kind, topic, lambda msg, key=name: self._receive(key, msg), 10)
        self._jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True), post_callback=lambda _: self._latest.clear())
        self._timer = self.create_timer(1 / cfg['rate_hz'], self._publish)

    def _receive(self, key, message):
        timestamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if timestamp > now + self._configuration['future_tolerance']:
            return
        old = self._latest.get(key)
        if old is not None and timestamp < old.header.stamp.sec + old.header.stamp.nanosec * 1e-9:
            return
        self._latest[key] = message

    def _publish(self):
        cfg = self._configuration
        try:
            joint, odom, target, physical = [
                self._latest[key] for key in ['joint', 'odom', 'target', 'physical']]
            times = [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                     for msg in [joint, odom, target, physical]]
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - min(times) > cfg['max_age'] or max(times) - min(times) > cfg['max_skew']:
                raise ValueError('Visualization feedback stale or skewed')
            measured = tracking_pose(self._model, odom, joint, self._names)
            base_transform = self._buffer.lookup_transform(
                self._frame, odom.header.frame_id, Time(seconds=times[1]))
            measured = transform_matrix(base_transform.transform) @ measured
            point = np.array([target.point.x, target.point.y, target.point.z])
            if target.header.frame_id != self._frame:
                target_transform = self._buffer.lookup_transform(
                    self._frame, target.header.frame_id, Time(seconds=times[2]))
                point = (transform_matrix(target_transform.transform) @ np.r_[point, 1.])[:3]
            if physical.header.frame_id != self._frame:
                raise ValueError('Physical pose must use the configured visualization frame')
            actual = tracking_pose(self._model, physical, joint, self._names)
            markers = build_markers(actual, point, self._axis, cfg, self._frame)
            inferred = build_markers(measured, point, self._axis, cfg, self._frame)
            for marker in inferred.markers:
                marker.ns = 'odometry_' + marker.ns
                marker.color.a *= .3
            markers.markers.extend(inferred.markers)
            for marker in markers.markers:
                marker.header.stamp = Time(seconds=min(times)).to_msg()
        except (KeyError, ValueError, IndexError, TransformException):
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers = MarkerArray(markers=[clear])
        self._publisher.publish(markers)


def main(args=None):
    """Run the opt-in tracking-frame marker publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = TrackingVisualization()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
