
"""Adapt project robot description to control and safety configuration."""

from collections.abc import Mapping
from typing import Any
from copy import deepcopy

import numpy as np

from robot_models.spatial.rotations import rpy_to_rotation
from robot_models.spatial.transforms import make_transform

from robot_control.factory.project_components import (
    robot_configuration_from_description,
)
from robot_control.adapters.control_config_adapter import ControlConfigAdapter
from robot_control.safety.input_validator import InputValidationConfig


def configured_control(parameters, description, strategy=None, *, lookahead='', feedforward=''):
    """Assemble the existing project configuration for both node and preflight."""
    if not parameters.get('safety_enabled', True):
        raise ValueError('Compute-only mode requires safety_enabled=true')
    lower, upper = parameters['base_velocity_lower'], parameters['base_velocity_upper']
    if lower is None or upper is None or len(lower) != 2 or len(upper) != 2:
        raise ValueError('Compute-only mode requires explicit base_velocity_lower and base_velocity_upper arrays in [v, omega] order')
    if len(description['robot']['manipulator']['joints']) != parameters['joint_count']:
        raise ValueError('ROS joint_count does not match robot description')
    robot, safety, _ = project_control_configuration(description, lower, upper)
    config = ControlConfigAdapter.convert(parameters, robot, safety)
    if strategy is not None:
        config = apply_strategy_configuration(config, strategy)
    if feedforward:
        if feedforward not in ['true', 'false']:
            raise ValueError('feedforward_override must be true, false or empty')
        config['tracking']['target_velocity_feedforward'] = feedforward == 'true'
    if lookahead:
        config['tracking']['lookahead_time'] = float(lookahead)
    validate_runtime_parameters(parameters, config['tracking']['lookahead_time'])
    return config


def validate_runtime_parameters(parameters, lookahead_time):
    """Use the same pure freshness and rate contracts before ROS startup."""
    rate = parameters['validation_rate']
    if not np.isfinite(rate) or rate <= 0:
        raise ValueError('validation_rate must be finite and positive')
    return InputValidationConfig(
        max_robot_state_age=parameters['max_robot_state_age'],
        max_prediction_age=parameters['max_prediction_age'],
        future_tolerance=parameters['future_tolerance'], lookahead_time=lookahead_time)


def apply_strategy_configuration(config, strategy):
    """Bind a selected project strategy to the existing shared tracking settings."""
    if not isinstance(strategy, Mapping) or set(strategy).difference({'controller', 'tracking'}):
        raise ValueError('Strategy may configure controller and tracking only')
    result = deepcopy(config)
    controller = deepcopy(strategy['controller'])
    if (not isinstance(controller, Mapping) or
            set(controller) - {'type', 'parameters', 'task', 'uses_safety_bounds', 'task_selection'}):
        raise ValueError('Unsupported controller strategy keys')
    task = controller.get('task')
    if task is not None:
        if not isinstance(task, Mapping) or set(task) - {'type', 'parameters'}:
            raise ValueError('Unsupported controller task keys')
        parameters = task.setdefault('parameters', {})
        if task['type'] == 'distance_pointing':
            pose = result['tracking']['pose']
            shared = {'radius': result['tracking']['policy']['parameters']['radius'],
                      'forward_axis_local': pose['forward_axis_local'],
                      'tangent_axis_local': pose['up_axis_local']}
        elif task['type'] == 'cartesian':
            shared = {'selection_matrix': result['controller']['task_selection']}
        else:
            shared = {}
        if set(parameters).intersection(shared):
            raise ValueError('Task geometry must use shared tracking configuration')
        parameters.update(shared)
    result['controller'] = controller
    overrides = strategy.get('tracking', {})
    if set(overrides).difference({
            'prediction_sampling', 'target_velocity_feedforward', 'lookahead_time'}):
        raise ValueError('Unsupported strategy tracking override')
    result['tracking'].update(overrides)
    return result


def _bounds(
    lower: Any,
    upper: Any,
    name: str,
) -> tuple[np.ndarray, np.ndarray]:
    """Validate compatible lower and upper bounds."""
    low = np.asarray(lower, dtype=np.float64)
    high = np.asarray(upper, dtype=np.float64)

    if (
        low.ndim != 1
        or low.size == 0
        or low.shape != high.shape
        or not np.all(np.isfinite(low))
        or not np.all(np.isfinite(high))
        or np.any(low > high)
    ):
        raise ValueError(f"Invalid {name} bounds")

    return low, high


def project_control_configuration(
    description: Mapping[str, Any],
    base_velocity_lower: Any,
    base_velocity_upper: Any,
) -> tuple[dict[str, Any], dict[str, Any], np.ndarray]:
    """Extract model, safety limits and tracking-frame transform.

    Args:
        description: Parsed robot description.
        base_velocity_lower: Lower limits for [v, omega].
        base_velocity_upper: Upper limits for [v, omega].

    Returns:
        Tuple containing:
            - Robot model configuration.
            - Safety configuration.
            - Fixed transform T_E_T.

    Notes:
        Joint ordering follows the robot description.

        Base velocity limits must be supplied explicitly and are
        not inferred from wheel geometry.

        The current SerialArmModel end-effector frame is expected
        to coincide with wrist_link.
    """
    if not isinstance(description, Mapping):
        raise TypeError("description must be a mapping")

    robot = description["robot"]

    model_config = robot_configuration_from_description(
        description
    )

    # --------------------------------------------------------
    # 1. Base velocity limits
    # --------------------------------------------------------

    base_lower, base_upper = _bounds(
        base_velocity_lower,
        base_velocity_upper,
        "base velocity",
    )

    if base_lower.shape != (2,):
        raise ValueError(
            "Base velocity bounds must have shape (2,)"
        )

    # --------------------------------------------------------
    # 2. Manipulator configuration
    # --------------------------------------------------------

    joints = list(
        robot["manipulator"]["joints"].values()
    )

    joint_count = len(joints)

    if joint_count == 0:
        raise ValueError(
            "Manipulator must contain joints"
        )

    joint_lower, joint_upper = _bounds(
        [joint["lower"] for joint in joints],
        [joint["upper"] for joint in joints],
        "joint position",
    )

    joint_speeds = np.asarray(
        [joint["velocity"] for joint in joints],
        dtype=np.float64,
    )

    if (
        not np.all(np.isfinite(joint_speeds))
        or np.any(joint_speeds <= 0.0)
    ):
        raise ValueError(
            "Joint velocity limits must be positive"
        )

    # --------------------------------------------------------
    # 3. Whole-body velocity bounds
    # --------------------------------------------------------

    # Generalized velocity ordering:
    # [v, omega, dq1, ..., dqn]

    velocity_lower = np.concatenate(
        (base_lower, -joint_speeds)
    )

    velocity_upper = np.concatenate(
        (base_upper, joint_speeds)
    )

    safety_config = {
        "velocity": {
            "lower": velocity_lower.tolist(),
            "upper": velocity_upper.tolist(),
        },
        "joint": {
            "lower": joint_lower.tolist(),
            "upper": joint_upper.tolist(),
            "velocity_indices": list(
                range(
                    base_lower.size,
                    base_lower.size + joint_count,
                )
            ),
        },
    }

    # --------------------------------------------------------
    # 4. Tracking-frame transform
    # --------------------------------------------------------

    frame = robot["tracking_frame"]

    if frame["parent"] != "wrist_link":
        raise ValueError(
            "Tracking-frame parent does not match the current "
            "SerialArmModel end-effector frame"
        )

    translation = np.asarray(
        frame["translation"],
        dtype=np.float64,
    )

    rpy = np.asarray(
        frame["rotation_rpy"],
        dtype=np.float64,
    )

    if (
        translation.shape != (3,)
        or rpy.shape != (3,)
        or not np.all(np.isfinite(translation))
        or not np.all(np.isfinite(rpy))
    ):
        raise ValueError(
            "Invalid tracking-frame pose"
        )

    # Reuse existing SO(3) and SE(3) utilities.
    transform_et = make_transform(
        rotation=rpy_to_rotation(rpy),
        position=translation,
    )

    model_config['tracking_frame'] = deepcopy(dict(frame))
    return model_config, safety_config, transform_et
