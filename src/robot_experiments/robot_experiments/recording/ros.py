"""Own one ROS observer/context; leave domain selection and process lifecycle outside."""
import sys
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

    def __init__(self, context, directory, *, diagnostics=None):
        """Retain the existing subscription topology and default disabled diagnostics."""
        super().__init__(context, directory, diagnostics=diagnostics)
        self._ros_context = None
        self._node = None
        self._executor = None
        self._callback_count = 0
        try:
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor, TimeoutObject
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            self._ros_context = Context()
            self._ros_context.init(args=[])
            self._node = Node('experiment_recorder', context=self._ros_context)
            if diagnostics is not None:
                from rclpy.utilities import get_rmw_implementation_identifier
                diagnostics.capabilities['rmw_implementation'] = (
                    get_rmw_implementation_identifier())
                diagnostics.capabilities['callback_metadata'] = 'public two-argument callback'
                diagnostics.capabilities['subscription_order'] = [
                    s for s, t in context.get('topics', {}).items() if t]
            self._executor = SingleThreadedExecutor(context=self._ros_context)
            # Stable public argument identity retains ready-iterator progress
            # when changing from the initial wait to a nonblocking drain.
            self._spin_timeout = TimeoutObject(0.)
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
                if diagnostics is None:
                    def callback(message, key=stream):
                        self._receive(key, message)
                else:
                    callback = self._timed_callback(stream)
                self._node.create_subscription(message_type, topic, callback, qos)
        except BaseException:
            self.close()
            raise

    def _receive(self, stream, message):
        self._callback_count += 1
        self.record(stream, message)

    def _timed_callback(self, stream):
        """Require two arguments so Jazzy selects its supported MessageInfo callback."""
        def receive(message, info):
            self._receive_timed(stream, message, info)
        return receive

    def _receive_timed(self, stream, message, info):
        entry = self._observer_timing.now()
        self._callback_count += 1
        self.record(stream, message, message_info=info, callback_entry=entry)

    def _spin(self, timeout):
        self._spin_timeout.timeout = timeout
        timing = self._observer_timing
        if timing is None:
            self._executor.spin_once(timeout_sec=self._spin_timeout)
        else:
            start = timing.now()
            try:
                self._executor.spin_once(timeout_sec=self._spin_timeout)
            finally:
                primary = sys.exception()
                try:
                    timing.executor_call(timeout, start, timing.now())
                except Exception as error:
                    timing.failed(error)
                    if primary is None:
                        raise

    def spin_once(self, timeout=.05):
        """Wait once, then drain ready subscriptions for at most 128 callbacks/10 ms.

        A lifecycle iteration must consume more than one source callback when
        telemetry arrives faster than its polling rate. Nonblocking drain calls
        stop immediately when the executor has no ready subscription to service.
        """
        if self._closed:
            raise RuntimeError('recorder is closed')
        timing = self._observer_timing
        if timing is not None:
            timing.begin_batch(timeout)
        reason = 'exception'
        try:
            previous = self._callback_count
            self._spin(timeout)
            deadline = monotonic() + .01
            if timing is not None:
                timing.start_drain()
            for _ in range(127):
                if self._callback_count == previous:
                    reason = 'no_callback'
                    break
                if monotonic() >= deadline:
                    reason = 'time_budget'
                    break
                previous = self._callback_count
                self._spin(0.)
            else:
                reason = 'count_budget'
        finally:
            if timing is not None:
                primary = sys.exception()
                try:
                    timing.end_batch(reason)
                except Exception as error:
                    timing.failed(error)
                    if primary is None:
                        raise

    def close(self):
        """Close ROS resources, scientific handles and optional diagnostic summary."""
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
