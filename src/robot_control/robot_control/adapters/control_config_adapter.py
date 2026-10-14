
"""Translate ROS control parameters into ControlFactory configuration.

Robot-model parameters and physical safety limits are supplied
separately. No ROS imports or robot-specific constants are required.
"""

from collections.abc import Mapping
from copy import deepcopy
from typing import Any

import numpy as np


class ControlConfigAdapter:
    """Build nested factory configuration from independent sources."""

    @staticmethod
    def _section(
        source: Mapping[str, Any],
        name: str,
    ) -> Mapping[str, Any]:
        """Extract a required mapping."""
        value = source.get(name)

        if not isinstance(value, Mapping):
            raise ValueError(
                f"'{name}' must be a configuration mapping"
            )

        return value

    @staticmethod
    def _vector(value: Any, name: str) -> np.ndarray:
        """Validate a finite, nonempty numeric vector."""
        try:
            result = np.asarray(value, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{name} must be numeric"
            ) from exc

        if (
            result.ndim != 1
            or result.size == 0
            or not np.all(np.isfinite(result))
        ):
            raise ValueError(
                f"{name} must be a finite nonempty vector"
            )

        return result

    @classmethod
    def convert(
        cls,
        ros_parameters: Mapping[str, Any],
        robot_configuration: Mapping[str, Any],
        safety_configuration: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Build the nested configuration expected by ControlFactory.

        Args:
            ros_parameters: Flat control_node ROS parameters.
            robot_configuration: Base and manipulator specifications.
            safety_configuration: Physical velocity and optional
                acceleration and joint constraints.

        Returns:
            Independent nested dictionary for ControlFactory.build().

        Raises:
            TypeError: If a top-level argument is not a mapping.
            ValueError: If parameters are missing or inconsistent.
        """
        sources = {
            "ros_parameters": ros_parameters,
            "robot_configuration": robot_configuration,
            "safety_configuration": safety_configuration,
        }

        for name, source in sources.items():
            if not isinstance(source, Mapping):
                raise TypeError(f"{name} must be a mapping")

        params = ros_parameters

        try:
            joint_count = params["joint_count"]
            task_axes = params["controller_task_axes"]
            task_gains = params["controller_task_gains"]
            damping = params["controller_damping"]
            lookahead = params["tracking_lookahead_time"]
            radius = params["tracking_radius"]
            forward_axis = params["tracking_forward_axis"]
            up_axis = params["tracking_up_axis"]
            world_up = params["world_up_axis"]
        except KeyError as exc:
            raise ValueError(
                f"Missing ROS control parameter: {exc.args[0]}"
            ) from exc

        if (
            isinstance(joint_count, (bool, np.bool_))
            or not isinstance(joint_count, (int, np.integer))
            or joint_count < 0
        ):
            raise ValueError(
                "joint_count must be a nonnegative integer"
            )

        axes = np.asarray(task_axes)

        if (
            axes.ndim != 1
            or axes.size == 0
            or not np.issubdtype(axes.dtype, np.integer)
            or np.any(axes < 0)
            or np.any(axes >= 6)
            or len(np.unique(axes)) != len(axes)
        ):
            raise ValueError(
                "controller_task_axes must contain distinct "
                "integers from 0 to 5"
            )

        gains = cls._vector(task_gains, "controller_task_gains")

        if gains.size != axes.size or np.any(gains <= 0):
            raise ValueError(
                "controller_task_gains must be positive and "
                "match controller_task_axes"
            )

        selection = np.eye(6)[axes].tolist()

        base = deepcopy(
            dict(cls._section(robot_configuration, "base"))
        )
        manipulator = deepcopy(
            dict(cls._section(robot_configuration, "manipulator"))
        )
        safety = deepcopy(dict(safety_configuration))

        cls._section(safety, "velocity")

        for name in ("base", "manipulator"):
            specification = (
                base if name == "base" else manipulator
            )

            if not isinstance(specification.get("type"), str):
                raise ValueError(
                    f"Robot {name} must specify a type"
                )

        configured_types = (
            ("base_model_type", base["type"]),
            ("manipulator_model_type", manipulator["type"]),
        )

        for parameter_name, actual_type in configured_types:
            if params.get(parameter_name) != actual_type:
                raise ValueError(
                    f"{parameter_name} disagrees with "
                    "robot configuration"
                )

        manipulator_parameters = manipulator.get(
            "parameters", {}
        )

        if not isinstance(manipulator_parameters, Mapping):
            raise ValueError(
                "Manipulator parameters must be a mapping"
            )

        if (
            "joint_count" in manipulator_parameters
            and manipulator_parameters["joint_count"] != joint_count
        ):
            raise ValueError(
                "Manipulator joint_count disagrees with ROS"
            )

        adapter_type = params.get("kinematics_adapter_type")

        if not isinstance(adapter_type, str):
            raise ValueError(
                "kinematics_adapter_type must be a string"
            )

        for name in (
            "controller_type",
            "reference_geometry_type",
            "reference_policy_type",
        ):
            if not isinstance(params.get(name), str):
                raise ValueError(f"{name} must be a string")

        # These must be supplied from physical model configuration,
        # not inferred from test values or arbitrary defaults.
        velocity = cls._section(safety, "velocity")

        for bound_name in ("lower", "upper"):
            if bound_name not in velocity:
                raise ValueError(
                    f"Missing safety velocity {bound_name} bounds"
                )

        # Validate configuration quantities before construction.
        for name, value in (
            ("controller_damping", damping),
            ("tracking_radius", radius),
        ):
            if (
                not np.isfinite(value)
                or float(value) <= 0.0
            ):
                raise ValueError(
                    f"{name} must be positive and finite"
                )

        if not np.isfinite(lookahead) or lookahead < 0.0:
            raise ValueError(
                "tracking_lookahead_time must be nonnegative"
            )

        return {
            "robot": {
                "base": base,
                "manipulator": manipulator,
                **({'tracking_frame': deepcopy(robot_configuration['tracking_frame'])}
                   if 'tracking_frame' in robot_configuration else {}),
            },
            "tracking": {
                "geometry": {
                    "type": params["reference_geometry_type"],
                },
                "policy": {
                    "type": params["reference_policy_type"],
                    "parameters": {
                        "radius": float(radius),
                    },
                },
                "pose": {
                    "forward_axis_local": cls._vector(
                        forward_axis,
                        "tracking_forward_axis",
                    ).tolist(),
                    "up_axis_local": cls._vector(
                        up_axis,
                        "tracking_up_axis",
                    ).tolist(),
                    "world_up": cls._vector(
                        world_up,
                        "world_up_axis",
                    ).tolist(),
                },
                "lookahead_time": float(lookahead),
            },
            "controller": {
                "type": params["controller_type"],
                "task_selection": selection,
                "parameters": {
                    "gains": np.diag(gains).tolist(),
                    "damping": float(damping),
                },
            },
            "safety": safety,
            "adapter": {
                "type": adapter_type,
                "parameters": {
                    "joint_count": int(joint_count),
                },
            },
        }
