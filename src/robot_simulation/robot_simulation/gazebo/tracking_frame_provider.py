
"""Read tracking-frame position using ROS 2 TF2."""

import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

from robot_simulation.state.physical_context import PhysicalPositionHistory

from tf2_ros import Buffer, TransformException, TransformListener


class TrackingFrameProvider:
    """Provide the tracking-frame position in a reference frame."""

    def __init__(
        self,
        node: Node,
        reference_frame: str,
        tracking_frame: str,
        max_age: float | None = None,
        future_tolerance: float = 0.,
        history_duration: float | None = None,
    ) -> None:
        if not isinstance(reference_frame, str) or not reference_frame.strip():
            raise ValueError("Reference frame must be non-empty.")

        if not isinstance(tracking_frame, str) or not tracking_frame.strip():
            raise ValueError("Tracking frame must be non-empty.")

        self._reference_frame = reference_frame
        self._tracking_frame = tracking_frame
        if max_age is not None and (not np.isfinite(max_age) or max_age <= 0):
            raise ValueError('Tracking TF maximum age must be finite and positive.')
        if not np.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError('Tracking TF future tolerance must be finite and nonnegative.')
        self._max_age = max_age
        self._future_tolerance = future_tolerance
        self._clock = node.get_clock()
        self._history = (PhysicalPositionHistory(reference_frame, "tf:"+tracking_frame, max_age, history_duration)
                         if max_age is not None else None)

        if history_duration is not None:
            # The contextual deployment publishes a direct world->camera transform.
            # Cache each source sample even while a target pose-service call is pending.
            self._direct_subscription = node.create_subscription(
                TFMessage, '/tf', self._receive_direct, qos_profile_sensor_data)
        self._buffer = Buffer(node=node)
        self._listener = TransformListener(
            self._buffer,
            node,
            spin_thread=False,
        )

    def position_world(self) -> np.ndarray | None:
        """Return the latest tracking-frame position, if available."""
        try:
            transform = self._buffer.lookup_transform(
                self._reference_frame,
                self._tracking_frame,
                Time(),
            )
        except TransformException:
            return None

        if self._max_age is not None:
            timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            age = self._clock.now().nanoseconds * 1e-9 - timestamp
            if age > self._max_age or age < -self._future_tolerance:
                return None

        translation = transform.transform.translation

        position = np.array(
            [
                translation.x,
                translation.y,
                translation.z,
            ],
            dtype=float,
        )

        if not np.all(np.isfinite(position)):
            raise ValueError(
                "Tracking-frame position is non-finite."
            )

        if self._history is not None:
            stamp = transform.header.stamp
            self._history.add(position, stamp.sec + stamp.nanosec*1e-9, self._reference_frame)
        return position

    def _receive_direct(self, message):
        if self._history is None:
            return
        now = self._clock.now().nanoseconds*1e-9
        for transform in message.transforms:
            stamp = transform.header.stamp
            timestamp = stamp.sec+stamp.nanosec*1e-9
            if (transform.header.frame_id != self._reference_frame
                    or transform.child_frame_id != self._tracking_frame
                    or timestamp > now+self._future_tolerance):
                continue
            p = transform.transform.translation
            try:
                self._history.add(np.array([p.x,p.y,p.z]),timestamp,self._reference_frame)
            except ValueError:
                continue

    def position_at(self, time, refresh=True):
        """Return a cached causal TF sample for contextual integration."""
        if refresh:
            self.position_world()
        sample = self._history.at(time) if self._history is not None else None
        return None if sample is None else sample.position_world.copy()

    def clear_history(self):
        """Discard cached samples at a simulation clock reset."""
        if self._history is not None:
            self._history.clear()
