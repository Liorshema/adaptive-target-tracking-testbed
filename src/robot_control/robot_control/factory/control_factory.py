
"""Construct a safe tracking pipeline from explicit configuration.

Component implementations are selected through injected registries.
This module has no ROS or Gazebo dependencies.
"""

from collections.abc import Mapping
from typing import Any

import numpy as np

from robot_models.constraints.acceleration_limits import AccelerationLimits
from robot_models.constraints.joint_limits import JointLimits
from robot_models.constraints.velocity_limits import VelocityLimits
from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.interfaces.robot_model import RobotModel
from robot_models.whole_body.composer import WholeBodyComposer

from robot_control.adapters.planar_kinematics_adapter import (
    PlanarKinematicsAdapter,
)
from robot_control.factory.component_registry import ComponentRegistry
from robot_control.interfaces.controller import Controller
from robot_control.pipeline.control_execution_pipeline import (
    ControlExecutionPipeline,
)
from robot_control.pipeline.safe_tracking_controller import (
    SafeTrackingController,
)
from robot_control.pipeline.tracking_control_orchestrator import (
    TrackingControlOrchestrator,
)
from robot_control.safety.safe_control_pipeline import SafeControlPipeline
from robot_control.tracking.pose_reference_generator import (
    PoseReferenceGenerator,
)
from robot_control.tracking.reference_selector import ReferenceSelector
from robot_control.tracking.task_space_error import TaskSpaceError
from robot_control.tracking.tracking_frame_kinematics import (
    TrackingFrameKinematics,
)
from robot_control.tracking.tracking_reference_builder import (
    TrackingReferenceBuilder,
)


def _section(
    config: Mapping[str, Any],
    name: str,
) -> Mapping[str, Any]:
    """Return a required configuration section."""
    value = config.get(name)

    if not isinstance(value, Mapping):
        raise ValueError(
            f"Configuration section '{name}' must be a mapping"
        )

    return value


def _array(value: Any, name: str) -> np.ndarray:
    """Convert configuration values to finite float arrays."""
    result = np.asarray(value, dtype=np.float64)

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")

    return result


class ControlFactory:
    """Build a complete compute-only pipeline from registries.

    Required registries:
        base, manipulator, geometry, policy, controller, adapter

    Registries are supplied externally, so this class does not
    hard-code robot or controller implementation names.
    """

    REGISTRY_NAMES = (
        "base",
        "manipulator",
        "geometry",
        "policy",
        "controller",
        "adapter",
    )

    def __init__(
        self,
        registries: Mapping[str, ComponentRegistry],
    ) -> None:
        """Store and validate component registries."""
        if not isinstance(registries, Mapping):
            raise TypeError("registries must be a mapping")

        for name in self.REGISTRY_NAMES:
            if name not in registries:
                raise ValueError(
                    f"Missing component registry: '{name}'"
                )

            if not isinstance(registries[name], ComponentRegistry):
                raise TypeError(
                    f"Registry '{name}' must be ComponentRegistry"
                )

        self._registries = dict(registries)

    def _create(
        self,
        category: str,
        specification: Mapping[str, Any],
    ) -> Any:
        """Construct a registered component from its specification."""
        component_type = specification.get("type")

        parameters = specification.get("parameters", {})

        if not isinstance(component_type, str):
            raise ValueError(
                f"Component type for '{category}' must be a string"
            )

        return self._registries[category].create(
            component_type,
            parameters,
        )

    def build(
        self,
        config: Mapping[str, Any],
    ) -> ControlExecutionPipeline:
        """Create a configured compute-only tracking pipeline.

        Args:
            config: Nested component and safety configuration.

        Returns:
            ControlExecutionPipeline ready to receive validated data.

        Raises:
            ValueError: For missing or incompatible configuration.
            TypeError: For incompatible registered implementations.
        """
        if not isinstance(config, Mapping):
            raise TypeError("config must be a mapping")

        robot_config = _section(config, "robot")
        tracking_config = _section(config, "tracking")
        controller_config = _section(config, "controller")
        safety_config = _section(config, "safety")
        adapter_config = _section(config, "adapter")

        # 1. Build robot components.
        base = self._create(
            "base",
            _section(robot_config, "base"),
        )
        manipulator = self._create(
            "manipulator",
            _section(robot_config, "manipulator"),
        )

        if not isinstance(base, BaseModel):
            raise TypeError("Registered base must implement BaseModel")

        if not isinstance(manipulator, ManipulatorModel):
            raise TypeError(
                "Registered manipulator must implement "
                "ManipulatorModel"
            )

        model: RobotModel = WholeBodyComposer(
            base=base,
            manipulator=manipulator,
        )

        velocity_dimension = model.generalized_velocity_dimension

        # 2. Build reference generation.
        geometry = self._create(
            "geometry",
            _section(tracking_config, "geometry"),
        )
        policy = self._create(
            "policy",
            _section(tracking_config, "policy"),
        )

        selector = ReferenceSelector(
            geometry=geometry,
            policy=policy,
        )

        pose_config = _section(tracking_config, "pose")

        pose_generator = PoseReferenceGenerator(
            forward_axis_local=_array(
                pose_config["forward_axis_local"],
                "forward_axis_local",
            ),
            up_axis_local=_array(
                pose_config["up_axis_local"],
                "up_axis_local",
            ),
            world_up=_array(
                pose_config["world_up"],
                "world_up",
            ),
        )

        reference_builder = TrackingReferenceBuilder(
            selector=selector,
            pose_generator=pose_generator,
        )

        # 3. Build the selected task-space controller.
        task = TaskSpaceError(
            selection_matrix=_array(
                controller_config["task_selection"],
                "task_selection",
            ),
        )

        controller_parameters = dict(
            controller_config.get("parameters", {})
        )

        controller_parameters["task"] = task

        controller = self._registries["controller"].create(
            controller_config["type"],
            controller_parameters,
        )

        if not isinstance(controller, Controller):
            raise TypeError(
                "Registered controller must implement Controller"
            )

        # 4. Configure the controlled tracking frame.
        tracking_frame = None
        frame_config = tracking_config.get('frame')
        if frame_config is not None:
            frame_config = _section(tracking_config, 'frame')
            tracking_frame = TrackingFrameKinematics(
                transform_end_effector_tracking=_array(
                    frame_config['transform_end_effector_tracking'],
                    'transform_end_effector_tracking',
                ),
            )

        # 5. Build prediction-based tracking.
        tracking = TrackingControlOrchestrator(
            model=model,
            controller=controller,
            reference_builder=reference_builder,
            lookahead_time=float(
                tracking_config.get("lookahead_time", 0.0)
            ),
            tracking_frame_kinematics=tracking_frame,
        )

        # 5. Build safety constraints.
        velocity_config = _section(safety_config, "velocity")

        velocity_limits = VelocityLimits(
            lower=_array(
                velocity_config["lower"],
                "velocity.lower",
            ),
            upper=_array(
                velocity_config["upper"],
                "velocity.upper",
            ),
        )

        if velocity_limits.lower.shape != (velocity_dimension,):
            raise ValueError(
                "Velocity limits dimension does not match robot model"
            )

        acceleration_limits = None

        if "acceleration" in safety_config:
            acceleration_config = _section(
                safety_config,
                "acceleration",
            )
            acceleration_limits = AccelerationLimits(
                lower=_array(
                    acceleration_config["lower"],
                    "acceleration.lower",
                ),
                upper=_array(
                    acceleration_config["upper"],
                    "acceleration.upper",
                ),
            )

            if acceleration_limits.lower.shape != (
                velocity_dimension,
            ):
                raise ValueError(
                    "Acceleration limits dimension does not "
                    "match robot model"
                )

        joint_limits = None
        joint_velocity_indices = None

        if "joint" in safety_config:
            joint_config = _section(safety_config, "joint")

            joint_limits = JointLimits(
                lower=_array(
                    joint_config["lower"],
                    "joint.lower",
                ),
                upper=_array(
                    joint_config["upper"],
                    "joint.upper",
                ),
            )

            joint_velocity_indices = np.asarray(
                joint_config["velocity_indices"],
            )

            if joint_limits.lower.shape != (manipulator.dof,):
                raise ValueError(
                    "Joint limits dimension does not match "
                    "manipulator DOF"
                )

        safety = SafeControlPipeline(
            velocity_limits=velocity_limits,
            acceleration_limits=acceleration_limits,
            joint_limits=joint_limits,
            joint_velocity_indices=joint_velocity_indices,
        )

        # 6. Build the robot-state adapter.
        adapter = self._create("adapter", adapter_config)

        if not isinstance(adapter, PlanarKinematicsAdapter):
            raise TypeError(
                "Registered adapter must be PlanarKinematicsAdapter"
            )

        if adapter._joint_count != manipulator.dof:
            raise ValueError(
                "Adapter joint count does not match manipulator DOF"
            )

        if base.velocity_dimension != (
            adapter.BASE_VELOCITY_DIMENSION
        ):
            raise ValueError(
                "Adapter base velocity dimension does not "
                "match robot model"
            )

        # 7. Compose the complete compute-only pipeline.
        safe_controller = SafeTrackingController(
            tracking=tracking,
            safety=safety,
        )

        return ControlExecutionPipeline(
            kinematics_adapter=adapter,
            controller=safe_controller,
        )
