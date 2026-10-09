
"""ROS 2 node for Gazebo-clock-driven target simulation."""

from dataclasses import replace
from typing import Any

import numpy as np
import rclpy

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from rosgraph_msgs.msg import Clock

from robot_models.types import TargetState

from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.gazebo.pose_client import GazeboPoseClient
from robot_simulation.gazebo.target_adapter import GazeboTargetAdapter
from robot_simulation.gazebo.tracking_frame_provider import (
    TrackingFrameProvider,
)
from robot_simulation.integration.time_stepper import TimeStepper
from robot_simulation.state.simulation_state import SimulationState


def required_vector(node: Node, name: str) -> np.ndarray:
    """Read a finite 3D vector from ROS parameters."""
    values = node.get_parameter(name).value
    vector = np.asarray(values, dtype=float)

    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite 3D vector.")

    return vector


def required_string(node: Node, name: str) -> str:
    """Read a required non-empty string parameter."""
    value = node.get_parameter(name).value

    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Missing ROS parameter: {name}")

    return value


class SimulationNode(Node):
    """Run target dynamics against the Gazebo simulation clock."""

    def __init__(self) -> None:
        super().__init__("simulation_node")

        self._declare_parameters()

        configs = [
            load_yaml(required_string(self, name))
            for name in (
                "target_config",
                "environment_config",
                "simulation_config",
            )
        ]

        engine, dt = SimulationFactory.with_defaults().create(
            *configs
        )

        integration_config = configs[2]["simulation"]["integration"]
        max_steps = integration_config["max_steps"]

        self._stepper = TimeStepper(
            engine=engine,
            dt=dt,
            max_steps=max_steps,
        )
        self._dt = dt

        self._initial_position = required_vector(
            self, "initial_position"
        )
        self._initial_velocity = required_vector(
            self, "initial_velocity"
        )

        self._tracking_source = required_string(
            self, "tracking_source"
        ).lower()

        if self._tracking_source not in ("static", "tf"):
            raise ValueError(
                "tracking_source must be 'static' or 'tf'."
            )

        self._static_tracking_position = None
        self._tracking_provider = None

        if self._tracking_source == "static":
            self._static_tracking_position = required_vector(
                self, "tracking_position"
            )
        else:
            self._tracking_provider = TrackingFrameProvider(
                node=self,
                reference_frame=required_string(
                    self, "reference_frame"
                ),
                tracking_frame=required_string(
                    self, "tracking_frame"
                ),
            )

        self._state: SimulationState | None = None
        self._pending = False

        self._adapter = GazeboTargetAdapter()
        self._pose_client = GazeboPoseClient(
            node=self,
            service_name=required_string(self, "pose_service"),
            entity_name=required_string(self, "entity_name"),
        )

        self._clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            self._on_clock,
            10,
        )

        self.get_logger().info(
            f"Simulation ready: dt={dt:.6f}s, "
            f"max_steps={max_steps}, "
            f"tracking_source={self._tracking_source}"
        )

    def _declare_parameters(self) -> None:
        """Declare ROS parameters with explicit types."""
        string_parameters = (
            "target_config",
            "environment_config",
            "simulation_config",
            "pose_service",
            "entity_name",
            "tracking_source",
            "reference_frame",
            "tracking_frame",
        )

        vector_parameters = (
            "initial_position",
            "initial_velocity",
            "tracking_position",
        )

        for name in string_parameters:
            self.declare_parameter(
                name,
                Parameter.Type.STRING,
            )

        for name in vector_parameters:
            self.declare_parameter(
                name,
                Parameter.Type.DOUBLE_ARRAY,
            )

    def _read_tracking_position(self) -> np.ndarray | None:
        """Resolve the tracking position from the selected source."""
        if self._tracking_source == "static":
            return self._static_tracking_position.copy()

        return self._tracking_provider.position_world()

    def _make_initial_state(
        self,
        time: float,
        tracking_position: np.ndarray,
    ) -> SimulationState:
        """Create the initial simulation state."""
        return SimulationState(
            target=TargetState(
                position=self._initial_position.copy(),
                velocity=self._initial_velocity.copy(),
            ),
            tracking_position_world=tracking_position,
            time=time,
        )

    def _on_clock(self, message: Clock) -> None:
        """Advance the target to the current Gazebo time."""
        now = (
            float(message.clock.sec)
            + float(message.clock.nanosec) * 1e-9
        )

        if not np.isfinite(now):
            self.get_logger().error("Invalid simulation clock.")
            return

        # Only one Gazebo pose request may be in flight.
        if self._pending:
            return

        # Do not advance the simulation without valid tracking data.
        try:
            tracking_position = self._read_tracking_position()
        except ValueError as error:
            self.get_logger().error(
                f"Invalid tracking position: {error}"
            )
            return

        if tracking_position is None:
            return

        if self._state is None:
            self._state = self._make_initial_state(
                now, tracking_position
            )
            self.get_logger().info(
                f"Simulation initialized at t={now:.3f}s"
            )
            return

        # Reinitialize if Gazebo simulation time moves backwards.
        if now < self._state.time:
            self.get_logger().warning(
                "Gazebo clock reset detected."
            )
            self._state = self._make_initial_state(
                now, tracking_position
            )
            return

        if now - self._state.time < self._dt:
            return

        if not self._pose_client.service_ready():
            return

        # Zero-order hold: use the latest tracking position
        # throughout the next integration interval.
        current = replace(
            self._state,
            tracking_position_world=tracking_position,
        )

        try:
            candidate = self._stepper.advance(
                state=current,
                target_time=now,
            )
        except ValueError as error:
            self.get_logger().error(
                f"Time synchronization failed: {error}"
            )
            return

        pose = self._adapter.to_pose(candidate.target)

        try:
            self._pending = True
            future = self._pose_client.send_pose(pose)

            future.add_done_callback(
                lambda completed, next_state=candidate:
                self._on_pose_result(completed, next_state)
            )
        except Exception as error:
            self._pending = False
            self.get_logger().error(
                f"Failed to send target pose: {error}"
            )

    def _on_pose_result(
        self,
        future: Any,
        candidate: SimulationState,
    ) -> None:
        """Commit the state after Gazebo accepts the target pose."""
        try:
            response = future.result()

            if response is None or not response.success:
                self.get_logger().error(
                    "Gazebo rejected the target pose."
                )
                return

            self._state = candidate

        except Exception as error:
            self.get_logger().error(
                f"Gazebo pose request failed: {error}"
            )

        finally:
            self._pending = False


def main(args=None) -> None:
    """Start the ROS 2 simulation node."""
    rclpy.init(args=args)
    node = None

    try:
        node = SimulationNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
