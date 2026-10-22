"""Adapt existing feedback into bounded, read-only RViz markers, paths and display TF."""

from geometry_msgs.msg import Point, PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_bringup.factory.model_factory import create_robot_model
from robot_description.model_parameters import load_robot_model_parameters
from robot_configuration.yaml_loader import load_yaml
from scipy.spatial.transform import Rotation
from robot_interfaces.msg import PredictedTrajectory, StateEstimate
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
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
    """Receive source-stamped observations without publishing actuator commands."""

    PATH_LAYERS = ('target_truth', 'target_estimate', 'target_prediction',
                   'robot_truth', 'robot_estimate')

    def __init__(self):
        """Bind deployment contracts and the independently validated scene engine."""
        from robot_bringup.visualization import Scene, validate_configuration

        super().__init__('tracking_visualization')
        for name in ['configuration_file', 'description_file', 'joint_topic', 'odom_topic',
                     'reference_frame', 'physical_pose_topic', 'target_estimate_topic',
                     'prediction_topic', 'robot_estimate_topic', 'base_frame', 'odometry_frame']:
            self.declare_parameter(name, '')
        self.declare_parameter('tracking_radius', 1.)
        self.declare_parameter('joint_names', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('forward_axis', rclpy.Parameter.Type.DOUBLE_ARRAY)
        cfg = validate_configuration(load_yaml(self.get_parameter('configuration_file').value))
        self._configuration = cfg
        self._frame = cfg['frame_id']
        self._reference = self.get_parameter('reference_frame').value
        self._odometry_frame = self.get_parameter('odometry_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        if self._frame not in {self._reference, self._odometry_frame}:
            raise ValueError('Visualization frame_id must be the fixed reference_frame or '
                             'odometry_frame; moving-frame histories are unsupported')
        if not self._base_frame:
            raise ValueError('Visualization requires the configured base_frame')
        parameters = load_robot_model_parameters(self.get_parameter('description_file').value)
        self._model = create_robot_model(parameters)
        self._names = list(self.get_parameter('joint_names').value)
        self._axis = self.get_parameter('forward_axis').value
        self._scene = Scene(cfg, self._model, self._names, parameters.joint_limits,
                            self._axis, self.get_parameter('tracking_radius').value)
        self._buffer = Buffer(node=self)
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(MarkerArray, cfg['topic'], qos)
        self._path_publishers = {
            name: self.create_publisher(Path, cfg['paths_topic'].rstrip('/') + '/' + name, qos)
            for name in self.PATH_LAYERS}
        for key, kind, topic in [
                ('joints', JointState, self.get_parameter('joint_topic').value),
                ('robot_truth', Odometry, self.get_parameter('physical_pose_topic').value),
                ('target_truth', PointStamped, cfg['target_topic']),
                ('target_estimate', StateEstimate, self.get_parameter('target_estimate_topic').value),
                ('prediction', PredictedTrajectory, self.get_parameter('prediction_topic').value),
                ('robot_estimate', StateEstimate, self.get_parameter('robot_estimate_topic').value)]:
            if not topic:
                raise ValueError(f'Visualization requires a topic for {key}')
            self.create_subscription(kind, topic, lambda msg, key=key: self._receive(key, msg), 10)
        self._jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True), post_callback=lambda _: self._scene.reset())
        self._timer = self.create_timer(1 / cfg['rate_hz'], self._publish)

    def _source_transform(self, frame, stamp):
        """Resolve the actual source epoch; missing frame IDs never imply identity."""
        if not frame:
            raise ValueError('Visualization feedback has no source frame')
        if frame == self._frame:
            return np.eye(4)
        transform = self._buffer.lookup_transform(
            self._frame, frame, Time(nanoseconds=int(round(stamp * 1e9))))
        result = transform_matrix(transform.transform)
        if not np.all(np.isfinite(result)):
            raise ValueError('Nonfinite visualization TF')
        return result

    def _receive(self, key, message):
        """Convert message contracts at issuance time and reject malformed feedback."""
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        frame = message.header.frame_id
        try:
            if not np.isfinite(stamp) or stamp > now + self._configuration['future_tolerance']:
                return False
            if key == 'joints':
                data = {'names': list(message.name), 'position': list(message.position),
                        'velocity': list(message.velocity)}
                if not np.all(np.isfinite(data['position'] + data['velocity'])):
                    return False
                return self._scene.receive(key, stamp, frame, data, now)
            if key == 'prediction':
                # A forecast can be mapped with a single transform only between the
                # supported fixed world/odometry frames, never a moving camera/base.
                if frame not in {self._reference, self._odometry_frame}:
                    return False
                if (message.state_dimension != 6 or message.horizon == 0 or
                        len(message.timestamps) != message.horizon or
                        len(message.states) != message.horizon * 6):
                    return False
                states = np.asarray(message.states, dtype=float).reshape((-1, 6))
                if not np.all(np.isfinite(states)) or not np.all(np.isfinite(message.timestamps)):
                    return False
                transform = self._source_transform(frame, stamp)
                states[:, :3] = states[:, :3] @ transform[:3, :3].T + transform[:3, 3]
                states[:, 3:] = states[:, 3:] @ transform[:3, :3].T
                data = {'states': states.tolist(), 'timestamps': list(message.timestamps)}
            elif key in {'target_truth', 'target_estimate'}:
                if key == 'target_truth':
                    position = np.array([message.point.x, message.point.y, message.point.z])
                else:
                    mean = np.asarray(message.mean, dtype=float)
                    if mean.shape != (6,) or not np.all(np.isfinite(mean)):
                        raise ValueError('Invalid target state')
                    position = mean[:3]
                if not np.all(np.isfinite(position)):
                    raise ValueError('Invalid target position')
                transform = self._source_transform(frame, stamp)
                data = {'position': (transform[:3, :3] @ position + transform[:3, 3]).tolist()}
            elif key in {'robot_truth', 'robot_estimate'}:
                if key == 'robot_truth':
                    pose = message.pose.pose
                    position = [pose.position.x, pose.position.y, pose.position.z]
                    orientation = [pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]
                else:
                    mean = np.asarray(message.mean, dtype=float)
                    count = len(self._names)
                    if mean.shape != (5 + 2 * count,) or not np.all(np.isfinite(mean)):
                        raise ValueError('Invalid robot state')
                    position = [mean[0], mean[1], 0.]
                    orientation = Rotation.from_euler('z', mean[2]).as_quat()
                if (not np.all(np.isfinite(np.r_[position, orientation])) or
                        not np.isclose(np.linalg.norm(orientation), 1., atol=1e-3, rtol=0.)):
                    raise ValueError('Invalid robot pose')
                transform = self._source_transform(frame, stamp)
                rotation = transform[:3, :3] @ Rotation.from_quat(orientation).as_matrix()
                data = {'position': (transform[:3, :3] @ position + transform[:3, 3]).tolist(),
                        'orientation': Rotation.from_matrix(rotation).as_quat().tolist()}
            else:
                return False
            accepted = self._scene.receive(key, stamp, self._frame, data, now)
            if accepted and key == 'robot_truth':
                display = TransformStamped()
                display.header.stamp = message.header.stamp
                display.header.frame_id = self._frame
                display.child_frame_id = self._configuration['model_frame_prefix'] + self._base_frame
                translation, rotation = display.transform.translation, display.transform.rotation
                translation.x, translation.y, translation.z = data['position']
                rotation.x, rotation.y, rotation.z, rotation.w = data['orientation']
                self._broadcaster.sendTransform(display)
            return accepted
        except (ValueError, TypeError, IndexError, OverflowError, TransformException):
            self._scene.invalidate(key, stamp)
            return False

    def _publish(self):
        """Publish a complete replacement scene, including empty unavailable paths."""
        now = self.get_clock().now().nanoseconds * 1e-9
        scene = self._scene.snapshot(now)
        frame = scene['frame_id']
        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = Time(nanoseconds=int(round(now * 1e9))).to_msg()
        clear.action = Marker.DELETEALL
        markers = MarkerArray(markers=[clear])
        kinds = {'sphere': Marker.SPHERE, 'arrow': Marker.ARROW,
                 'line_list': Marker.LINE_LIST, 'text': Marker.TEXT_VIEW_FACING}
        for item in scene['markers']:
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = Time(nanoseconds=int(round(item['stamp'] * 1e9))).to_msg()
            marker.ns, marker.id = item['ns'], item['id']
            marker.action, marker.type = Marker.ADD, kinds[item['kind']]
            marker.pose.orientation.w = 1.
            marker.scale.x, marker.scale.y, marker.scale.z = item['scale']
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = item['color']
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                             for p in item.get('points', [])]
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                item.get('position', [0., 0., 0.]))
            marker.text = item.get('text', '')
            marker.lifetime = Duration(seconds=self._configuration['lifetime']).to_msg()
            markers.markers.append(marker)
        self._publisher.publish(markers)
        for name, publisher in self._path_publishers.items():
            path = Path()
            path.header.frame_id = frame
            path.header.stamp = clear.header.stamp
            for sample in scene['paths'][name]:
                pose = PoseStamped()
                pose.header.frame_id = frame
                pose.header.stamp = Time(nanoseconds=int(round(sample['stamp'] * 1e9))).to_msg()
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = sample['position']
                pose.pose.orientation.w = 1.
                path.poses.append(pose)
            publisher.publish(path)


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
