
"""Read tracking-frame position using ROS 2 TF2."""

import numpy as np

from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener


class TrackingFrameProvider:
    """Provide the tracking-frame position in a reference frame."""

    def __init__(
        self,
        node: Node,
        reference_frame: str,
        tracking_frame: str,
    ) -> None:
        if not isinstance(reference_frame, str) or not reference_frame.strip():
            raise ValueError("Reference frame must be non-empty.")

        if not isinstance(tracking_frame, str) or not tracking_frame.strip():
            raise ValueError("Tracking frame must be non-empty.")

        self._reference_frame = reference_frame
        self._tracking_frame = tracking_frame

        self._buffer = Buffer()
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

        return position
