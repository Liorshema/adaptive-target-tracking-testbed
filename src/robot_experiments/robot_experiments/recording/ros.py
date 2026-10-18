"""Own one ROS observer/context; leave domain selection and process lifecycle outside."""
from importlib import import_module
from time import monotonic

from .normalization import STREAM_TYPES
from .storage import TelemetryStore


class Recorder(TelemetryStore):
    """Observe configured source topics using an independently owned ROS context.

    The runner sets ROS_DOMAIN_ID before construction. No default rclpy context,
    estimator clock, or wall-time fallback participates in source timestamps.
    Best-effort volatile subscriptions accept Gazebo sensor and reliable sources.
    """

    def __init__(self, context, directory):
        super().__init__(context, directory)
        self._ros_context = None
        self._node = None
        self._executor = None
        self._callback_count = 0
        try:
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            self._ros_context = Context()
            self._ros_context.init(args=[])
            self._node = Node('experiment_recorder', context=self._ros_context)
            self._executor = SingleThreadedExecutor(context=self._ros_context)
            self._executor.add_node(self._node)
            for stream, topic in context.get('topics', {}).items():
                if not topic:
                    continue
                if stream not in STREAM_TYPES:
                    raise ValueError(f'unknown telemetry stream: {stream}')
                module, name = STREAM_TYPES[stream]
                message_type = getattr(import_module(module), name)
                qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                self._node.create_subscription(
                    message_type, topic, lambda message, key=stream: self._receive(key, message), qos)
        except BaseException:
            self.close()
            raise

    def _receive(self, stream, message):
        self._callback_count += 1
        self.record(stream, message)

    def spin_once(self, timeout=.05):
        """Wait once, then drain ready subscriptions for at most 128 callbacks/10 ms.

        A lifecycle iteration must consume more than one source callback when
        telemetry arrives faster than its polling rate. Nonblocking drain calls
        stop immediately when the executor has no ready subscription to service.
        """
        if self._closed:
            raise RuntimeError('recorder is closed')
        previous = self._callback_count
        self._executor.spin_once(timeout_sec=timeout)
        deadline = monotonic() + .01
        for _ in range(127):
            if self._callback_count == previous or monotonic() >= deadline:
                break
            previous = self._callback_count
            self._executor.spin_once(timeout_sec=0.)

    def close(self):
        if self._closed:
            return
        try:
            try:
                if self._executor is not None:
                    self._executor.shutdown(timeout_sec=1.)
            finally:
                try:
                    if self._node is not None:
                        self._node.destroy_node()
                finally:
                    if self._ros_context is not None:
                        self._ros_context.try_shutdown()
        finally:
            super().close()
