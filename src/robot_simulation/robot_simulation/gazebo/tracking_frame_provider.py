
"""Read tracking-frame position using ROS 2 TF2."""

from collections import Counter, deque

import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
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
        history_capacity: int | None = None,
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
        self._insertions = Counter()
        self._recent_arrivals = deque(maxlen=64)
        self._last_received = self._last_accepted = self._receipt_clock = None
        self._request = self._valid_until = self._selected_time = None
        self._failed_selections = 0
        self._wait_since = None
        self._wait_reason = None
        self._history = (PhysicalPositionHistory(reference_frame, "tf:"+tracking_frame, max_age, history_duration, history_capacity)
                         if max_age is not None else None)

        if history_duration is not None:
            # The contextual deployment publishes a direct world->camera transform.
            # Historical integration needs every causal sample while callbacks or
            # pose acknowledgements lag. Sensor-data QoS (best effort, depth 5)
            # permanently loses that interval under backlog. Match reliable TF
            # delivery, bounded by the deployment's existing history capacity.
            # Standalone callers without a capacity use TF2's standard depth.
            self._direct_subscription = node.create_subscription(
                TFMessage, '/tf', self._receive_direct,
                QoSProfile(depth=history_capacity if history_capacity is not None else 100))
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
            self._history.add(position, stamp.sec + stamp.nanosec*1e-9,
                              self._reference_frame,
                              receipt_time=self._clock.now().nanoseconds*1e-9)
        return position

    def _receive_direct(self, message):
        if self._history is None:
            return
        now = self._clock.now().nanoseconds * 1e-9
        for transform in message.transforms:
            if transform.child_frame_id != self._tracking_frame:
                continue
            stamp = transform.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            self._last_received, self._receipt_clock = timestamp, now
            if timestamp < 0:
                self._insertions['invalid'] += 1
                self._recent_arrivals.append((timestamp, now, 'invalid'))
                continue
            if transform.header.frame_id != self._reference_frame:
                self._insertions['frame_mismatch'] += 1
                self._recent_arrivals.append((timestamp, now, 'frame_mismatch'))
                continue
            p = transform.transform.translation
            try:
                outcome = self._history.add(
                    np.array([p.x, p.y, p.z]), timestamp, self._reference_frame,
                    receipt_time=now)
            except ValueError:
                outcome = 'invalid'
            self._insertions[outcome] += 1
            self._recent_arrivals.append((timestamp, now, outcome))
            if outcome == 'accepted':
                self._last_accepted = timestamp

    def position_at(self, time, refresh=True, valid_until=None):
        """Return a cached causal TF sample for contextual integration."""
        if refresh:
            self.position_world()
        self._request, self._valid_until = time, valid_until
        sample = self._history.at(time) if self._history is not None else None
        reason = None
        if sample is None:
            previous = [s for s in self._history._samples if s.timestamp <= time] if self._history else []
            reason = 'expired' if previous else 'no_preceding_sample'
        elif valid_until is not None:
            try:
                sample.position_at(valid_until, self._reference_frame)
            except ValueError:
                sample = None
                reason = 'interval_expired'
        self._wait_reason = reason
        self._selected_time = None if sample is None else sample.timestamp
        if sample is None:
            self._failed_selections += 1
            if self._wait_since is None:
                self._wait_since = self._clock.now().nanoseconds * 1e-9
        else:
            self._failed_selections = 0
            self._wait_since = None
        return None if sample is None else sample.position_world.copy()

    def diagnostics(self):
        """Return bounded metadata only; diagnostics never alter context selection."""
        samples = self._history._samples if self._history is not None else []
        request = self._request
        preceding = [s.timestamp for s in samples if request is not None and s.timestamp <= request]
        following = [s.timestamp for s in samples if request is not None and s.timestamp > request]
        now = self._clock.now().nanoseconds * 1e-9
        return {
            'last_received_source_time': self._last_received,
            'last_accepted_source_time': self._last_accepted,
            'receipt_clock': self._receipt_clock,
            'insertions': dict(self._insertions),
            'recent_arrivals': list(self._recent_arrivals),
            'history_size': len(samples),
            'history_range': [samples[0].timestamp, samples[-1].timestamp] if samples else [],
            'history_capacity': self._history.capacity if self._history is not None else None,
            'requested_time': request, 'valid_until': self._valid_until,
            'selected_source_time': self._selected_time,
            'preceding_time': preceding[-1] if preceding else None,
            'following_time': following[0] if following else None,
            'max_age': self._max_age, 'wait_reason': self._wait_reason,
            'consecutive_failed_selections': self._failed_selections,
            'wait_duration': 0. if self._wait_since is None else max(0., now-self._wait_since),
        }

    def clear_history(self):
        """Discard cached samples and selection diagnostics at a clock reset."""
        if self._history is not None:
            self._history.clear()
        self._wait_since = None
        self._failed_selections = 0
        self._request = self._valid_until = self._selected_time = None
        self._last_received = self._last_accepted = self._receipt_clock = None
        self._wait_reason = None
        self._insertions.clear()
        self._recent_arrivals.clear()
