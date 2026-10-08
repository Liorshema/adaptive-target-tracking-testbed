"""Load mathematical robot-model parameters from robot description."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class DifferentialDriveParameters:
    """Parameters required by a differential-drive model."""

    wheel_radius: float
    track_width: float


@dataclass(frozen=True)
class SerialArmParameters:
    """Parameters required by a serial-arm model."""

    joint_axes: np.ndarray
    body_height: float
    arm_base_height: float
    link_1_length: float
    link_2_length: float
    wrist_length: float


@dataclass(frozen=True)
class TrackingFrameParameters:
    """Fixed tracking-frame transform relative to its parent link."""

    parent: str
    translation: np.ndarray
    rotation_rpy: np.ndarray


@dataclass(frozen=True)
class JointLimitParameters:
    """Manipulator joint limits and actuator-related bounds."""

    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray
    damping: np.ndarray


@dataclass(frozen=True)
class RobotModelParameters:
    """Parameters required to construct the mathematical robot model."""

    base: DifferentialDriveParameters
    manipulator: SerialArmParameters
    joint_limits: JointLimitParameters
    tracking_frame: TrackingFrameParameters


def load_robot_model_parameters(
    config_path: str | Path | None = None,
) -> RobotModelParameters:
    """
    Load and validate robot-model parameters.

    When no path is supplied, load robot.yaml from the installed
    robot_description package.
    """
    path = (
        Path(config_path)
        if config_path is not None
        else _default_config_path()
    )

    with path.open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError('Robot configuration must be a mapping.')

    robot = _require_mapping(data, 'robot')
    body = _require_mapping(robot, 'body')
    wheels = _require_mapping(robot, 'wheels')
    manipulator = _require_mapping(robot, 'manipulator')
    arm_base = _require_mapping(manipulator, 'base')
    link_1 = _require_mapping(manipulator, 'link1')
    link_2 = _require_mapping(manipulator, 'link2')
    wrist = _require_mapping(manipulator, 'wrist')
    joints = _require_mapping(manipulator, 'joints')
    tracking_frame = _require_mapping(robot, 'tracking_frame')

    joint_names = tuple(joints.keys())

    if not joint_names:
        raise ValueError(
            'Manipulator must define at least one joint.'
        )

    joint_configs = [
        _require_mapping(joints, name)
        for name in joint_names
    ]

    joint_axes = np.asarray(
        [
            config['axis']
            for config in joint_configs
        ],
        dtype=float,
    )

    joint_count = len(joint_names)

    if joint_axes.shape != (joint_count, 3):
        raise ValueError(
            'Manipulator joint axes must have shape '
            f'({joint_count}, 3).'
        )

    axis_norms = np.linalg.norm(
        joint_axes,
        axis=1,
    )

    if np.any(axis_norms <= 0.0):
        raise ValueError(
            'Manipulator joint axes must be non-zero.'
        )

    joint_axes = (
        joint_axes
        / axis_norms[:, np.newaxis]
    )

    wheel_radius = _positive_float(
        wheels,
        'radius',
    )

    wheel_y_offset = _positive_float(
        wheels,
        'y_offset',
    )

    # Distance between left and right wheel centerlines.
    track_width = 2.0 * wheel_y_offset

    base_parameters = DifferentialDriveParameters(
        wheel_radius=wheel_radius,
        track_width=track_width,
    )

    manipulator_parameters = SerialArmParameters(
        joint_axes=joint_axes,
        body_height=_positive_float(
            body,
            'height',
        ),
        arm_base_height=_positive_float(
            arm_base,
            'height',
        ),
        link_1_length=_positive_float(
            link_1,
            'length',
        ),
        link_2_length=_positive_float(
            link_2,
            'length',
        ),
        wrist_length=_positive_float(
            wrist,
            'length',
        ),
    )

    joint_limit_parameters = JointLimitParameters(
        lower=_joint_vector(
            joint_configs,
            'lower',
            size=joint_count,
        ),
        upper=_joint_vector(
            joint_configs,
            'upper',
            size=joint_count,
        ),
        velocity=_joint_vector(
            joint_configs,
            'velocity',
            size=joint_count,
        ),
        effort=_joint_vector(
            joint_configs,
            'effort',
            size=joint_count,
        ),
        damping=_joint_vector(
            joint_configs,
            'damping',
            size=joint_count,
        ),
    )

    if np.any(
        joint_limit_parameters.lower
        > joint_limit_parameters.upper
    ):
        raise ValueError(
            'Joint lower limits cannot exceed upper limits.'
        )

    tracking_frame_parameters = TrackingFrameParameters(
        parent=str(tracking_frame['parent']),
        translation=_vector(
            tracking_frame,
            'translation',
            size=3,
        ),
        rotation_rpy=_vector(
            tracking_frame,
            'rotation_rpy',
            size=3,
        ),
    )

    return RobotModelParameters(
        base=base_parameters,
        manipulator=manipulator_parameters,
        joint_limits=joint_limit_parameters,
        tracking_frame=tracking_frame_parameters,
    )


def _default_config_path() -> Path:
    """Return the installed robot.yaml path."""
    package_share = Path(
        get_package_share_directory(
            'robot_description'
        )
    )

    return (
        package_share
        / 'config'
        / 'robot'
        / 'robot.yaml'
    )


def _require_mapping(
    mapping: dict[str, Any],
    key: str,
) -> dict[str, Any]:
    """Return a required nested mapping."""
    value = mapping.get(key)

    if not isinstance(value, dict):
        raise ValueError(
            f'Missing or invalid mapping: {key}'
        )

    return value


def _positive_float(
    mapping: dict[str, Any],
    key: str,
) -> float:
    """Return a required positive scalar parameter."""
    if key not in mapping:
        raise ValueError(
            f'Missing parameter: {key}'
        )

    value = float(mapping[key])

    if value <= 0.0:
        raise ValueError(
            f'Parameter {key} must be positive.'
        )

    return value


def _joint_vector(
    joint_configs: list[dict[str, Any]],
    key: str,
    size: int,
) -> np.ndarray:
    """Collect one joint parameter into a vector."""
    try:
        vector = np.asarray(
            [
                config[key]
                for config in joint_configs
            ],
            dtype=float,
        )
    except KeyError as error:
        raise ValueError(
            f'Missing joint parameter: {key}'
        ) from error

    if vector.shape != (size,):
        raise ValueError(
            f'Joint parameter {key} must have shape ({size},).'
        )

    return vector

def _vector(
    mapping: dict[str, Any],
    key: str,
    size: int,
) -> np.ndarray:
    """Return a required fixed-size vector parameter."""
    if key not in mapping:
        raise ValueError(
            f'Missing vector parameter: {key}'
        )

    vector = np.asarray(
        mapping[key],
        dtype=float,
    )

    if vector.shape != (size,):
        raise ValueError(
            f'Parameter {key} must have shape ({size},).'
        )

    return vector
