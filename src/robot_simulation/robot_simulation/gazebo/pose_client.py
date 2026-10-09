
"""ROS 2 client for setting an entity pose in Gazebo."""

from concurrent.futures import Future

import numpy as np
from rclpy.node import Node

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from robot_simulation.gazebo.target_adapter import TargetPose


class GazeboPoseClient:
    """Send pose commands to a configured Gazebo entity."""

    def __init__(
        self,
        node: Node,
        service_name: str,
        entity_name: str,
        entity_type: int = Entity.MODEL,
    ) -> None:
        if not service_name or not service_name.strip():
            raise ValueError("Service name must not be empty.")

        if not entity_name or not entity_name.strip():
            raise ValueError("Entity name must not be empty.")

        self._entity_name = entity_name
        self._entity_type = entity_type

        self._client = node.create_client(
            SetEntityPose,
            service_name,
        )

    def service_ready(self) -> bool:
        """Return whether the Gazebo pose service is available."""
        return self._client.service_is_ready()

    def send_pose(self, pose: TargetPose) -> Future:
        """Submit a non-blocking pose request.

        The returned future completes when the service responds.
        A completed future must still be checked for success.
        """
        position = np.asarray(pose.position, dtype=float)
        orientation = np.asarray(
            pose.orientation_xyzw, dtype=float
        )

        if position.shape != (3,) or not np.all(
            np.isfinite(position)
        ):
            raise ValueError("Position must be a finite 3D vector.")

        if orientation.shape != (4,) or not np.all(
            np.isfinite(orientation)
        ):
            raise ValueError("Orientation must be a finite quaternion.")

        if not self.service_ready():
            raise RuntimeError("Gazebo pose service is not ready.")

        request = SetEntityPose.Request()

        request.entity.name = self._entity_name
        request.entity.type = self._entity_type

        request.pose.position.x = float(position[0])
        request.pose.position.y = float(position[1])
        request.pose.position.z = float(position[2])

        request.pose.orientation.x = float(orientation[0])
        request.pose.orientation.y = float(orientation[1])
        request.pose.orientation.z = float(orientation[2])
        request.pose.orientation.w = float(orientation[3])

        return self._client.call_async(request)
