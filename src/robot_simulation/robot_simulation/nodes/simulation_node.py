
"""ROS 2 node for Gazebo-clock-driven target simulation."""

from dataclasses import replace
from typing import Any

import numpy as np
import rclpy

from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

from robot_models.target.target_state import TargetState

from robot_configuration.yaml_loader import load_yaml
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.gazebo.pose_client import GazeboPoseClient
from robot_simulation.gazebo.target_adapter import GazeboTargetAdapter
from robot_simulation.gazebo.tracking_frame_provider import (
    TrackingFrameProvider,
)
from robot_simulation.integration.time_stepper import TimeStepper
from robot_simulation.measurements.synthetic_measurement import (
    SyntheticMeasurementGenerator,
)
from robot_simulation.state.simulation_state import SimulationState
from robot_simulation.adapters.target_state import state_to_vector


def required_vector(node: Node, name: str) -> np.ndarray:
    """Read a finite 3D vector from ROS parameters."""
    vector = np.asarray(
        node.get_parameter(name).value,
        dtype=float,
    )

    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite 3D vector.")

    return vector


def required_string(node: Node, name: str) -> str:
    """Read a required non-empty string parameter."""
    value = node.get_parameter(name).value

    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Missing ROS parameter: {name}")

    return value


def make_position_message(
    position: np.ndarray,
    timestamp: float,
    frame_id: str,
) -> PointStamped:
    """Convert a 3D position to a stamped ROS message."""
    position = np.asarray(position, dtype=float)

    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("PointStamped requires a finite 3D position.")

    message = PointStamped()
    message.header.stamp = Time(seconds=timestamp).to_msg()
    message.header.frame_id = frame_id

    message.point.x = float(position[0])
    message.point.y = float(position[1])
    message.point.z = float(position[2])

    return message


class SimulationNode(Node):
    """Run target simulation and publish observations."""

    def __init__(self) -> None:
        super().__init__("simulation_node")

        self._declare_parameters()

        target_config = load_yaml(
            required_string(self, "target_config")
        )
        environment_config = load_yaml(
            required_string(self, "environment_config")
        )
        simulation_config = load_yaml(
            required_string(self, "simulation_config")
        )
        measurement_config = load_yaml(
            required_string(self, "measurement_config")
        )

        engine, dt = SimulationFactory.with_defaults().create(
            target_config,
            environment_config,
            simulation_config,
        )
        self.get_logger().info(
            f'Simulation components: dynamics={type(engine.dynamics).__name__}, '
            f'integrator={type(engine.integrator).__name__}')

        integration_config = simulation_config["simulation"][
            "integration"
        ]

        self._stepper = TimeStepper(
            engine=engine,
            dt=dt,
            max_steps=integration_config["max_steps"],
        )
        self._dt = dt

        self._initial_position = required_vector(
            self, "initial_position"
        )
        self._initial_velocity = required_vector(
            self, "initial_velocity"
        )

        self._reference_frame = required_string(
            self, "reference_frame"
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
                reference_frame=self._reference_frame,
                tracking_frame=required_string(
                    self, "tracking_frame"
                ),
                max_age=self.get_parameter('tracking_max_age').value,
                future_tolerance=self.get_parameter('tracking_future_tolerance').value,
            )

        self._state: SimulationState | None = None
        self._pending = False

        self._adapter = GazeboTargetAdapter()
        self._pose_client = GazeboPoseClient(
            node=self,
            service_name=required_string(self, "pose_service"),
            entity_name=required_string(self, "entity_name"),
        )

        # Ground-truth output.
        ground_truth_config = simulation_config["simulation"][
            "ground_truth"
        ]

        self._ground_truth_publisher = self.create_publisher(
            PointStamped,
            ground_truth_config["topic"],
            10,
        )

        # Synthetic position measurements.
        sensor_config = measurement_config["measurements"][
            "target_position"
        ]

        self._measurement_enabled = bool(
            sensor_config["enabled"]
        )
        self._measurement_generator = None
        self._measurement_publisher = None
        self._measurement_period = None
        self._last_measurement_time = None

        if self._measurement_enabled:
            components = sensor_config["observation"][
                "components"
            ]

            if components != ["position"]:
                raise ValueError(
                    "PointStamped sensor output requires "
                    "observation.components: [position]."
                )

            state_dim = (
                self._initial_position.size
                + self._initial_velocity.size
            )
            position_dim = self._initial_position.size

            H = np.zeros((position_dim, state_dim))
            H[:, :position_dim] = np.eye(position_dim)

            R = np.asarray(
                sensor_config["noise"]["covariance"],
                dtype=float,
            )

            rate_hz = float(sensor_config["rate_hz"])

            if not np.isfinite(rate_hz) or rate_hz <= 0:
                raise ValueError(
                    "Measurement rate_hz must be positive."
                )

            self._measurement_period = 1.0 / rate_hz

            self._measurement_generator = (
                SyntheticMeasurementGenerator(
                    observation_matrix=H,
                    covariance=R,
                    rng=np.random.default_rng(
                        sensor_config["random_seed"]
                    ),
                )
            )

            self._measurement_publisher = self.create_publisher(
                PointStamped,
                sensor_config["topic"],
                10,
            )

        self._clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            self._on_clock,
            10,
        )

        self.get_logger().info(
            f"Simulation ready: dt={dt:.6f}s, "
            f"tracking_source={self._tracking_source}, "
            f"measurements_enabled={self._measurement_enabled}"
        )

    def _declare_parameters(self) -> None:
        """Declare ROS parameters with explicit types."""
        string_parameters = (
            "target_config",
            "environment_config",
            "simulation_config",
            "measurement_config",
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
                name, Parameter.Type.STRING
            )

        for name in vector_parameters:
            self.declare_parameter(
                name, Parameter.Type.DOUBLE_ARRAY
            )
        self.declare_parameter('tracking_max_age', 0.2)
        self.declare_parameter('tracking_future_tolerance', 0.01)

    def _read_tracking_position(self) -> np.ndarray | None:
        """Resolve the configured tracking-position source."""
        if self._tracking_source == "static":
            return self._static_tracking_position.copy()

        return self._tracking_provider.position_world()

    def _make_initial_state(
        self,
        time: float,
        tracking_position: np.ndarray,
    ) -> SimulationState:
        """Construct the initial simulation state."""
        return SimulationState(
            target=TargetState(
                position=self._initial_position.copy(),
                velocity=self._initial_velocity.copy(),
            ),
            tracking_position_world=tracking_position,
            time=time,
        )

    def _publish_ground_truth(
        self,
        state: SimulationState,
    ) -> None:
        """Publish the committed target position."""
        message = make_position_message(
            position=state.target.position,
            timestamp=state.time,
            frame_id=self._reference_frame,
        )

        self._ground_truth_publisher.publish(message)

    def _publish_measurement(
        self,
        state: SimulationState,
    ) -> None:
        """Generate and publish a scheduled noisy observation."""
        if not self._measurement_enabled:
            return

        timestamp = state.time

        if self._last_measurement_time is not None:
            elapsed = timestamp - self._last_measurement_time

            if elapsed < self._measurement_period - 1e-9:
                return

        measurement = self._measurement_generator.measure(
            state=state_to_vector(state.target),
            timestamp=timestamp,
        )

        message = make_position_message(
            position=measurement.value,
            timestamp=measurement.timestamp,
            frame_id=self._reference_frame,
        )

        self._measurement_publisher.publish(message)
        self._last_measurement_time = timestamp

    def _on_clock(self, message: Clock) -> None:
        """Advance the target to the current Gazebo time."""
        now = (
            float(message.clock.sec)
            + float(message.clock.nanosec) * 1e-9
        )

        if not np.isfinite(now):
            self.get_logger().error("Invalid simulation clock.")
            return

        if self._pending:
            return

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

        if now < self._state.time:
            self.get_logger().warning(
                "Gazebo clock reset detected."
            )
            self._state = self._make_initial_state(
                now, tracking_position
            )
            self._last_measurement_time = None
            return

        if now - self._state.time < self._dt:
            return

        if not self._pose_client.service_ready():
            return

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
        """Commit state after Gazebo accepts the target pose."""
        try:
            response = future.result()

            if response is None or not response.success:
                self.get_logger().error(
                    "Gazebo rejected the target pose."
                )
                return

            self._state = candidate

            self._publish_ground_truth(candidate)
            self._publish_measurement(candidate)

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
