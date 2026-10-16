"""Tests for reusable robot-description parameters and component specs."""

from importlib import import_module
from pathlib import Path
import subprocess
import sys
from types import MappingProxyType

import numpy as np
import pytest
from robot_description import model_parameters
import yaml


@pytest.fixture
def description():
    """Provide an ordered three-joint description with non-unit axes."""
    return {
        'robot': {
            'body': {'height': 0.2},
            'wheels': {'radius': 0.15, 'y_offset': 0.34},
            'manipulator': {
                'base': {'height': 0.08},
                'link1': {'length': 0.4},
                'link2': {'length': 0.36},
                'wrist': {'length': 0.12},
                'joints': {
                    'tip': {
                        'axis': [0.0, -2.0, 0.0],
                        'lower': -0.3, 'upper': 0.3,
                        'velocity': 0.4, 'effort': 8.0, 'damping': 0.1,
                    },
                    'yaw': {
                        'axis': [0.0, 0.0, 3.0],
                        'lower': -1.0, 'upper': 1.0,
                        'velocity': 0.6, 'effort': 10.0, 'damping': 0.2,
                    },
                    'middle': {
                        'axis': [0.0, -5.0, 0.0],
                        'lower': -0.7, 'upper': 0.7,
                        'velocity': 0.8, 'effort': 12.0, 'damping': 0.3,
                    },
                },
            },
            'tracking_frame': {
                'parent': 'wrist_link',
                'translation': [0.12, 0.0, 0.0],
                'rotation_rpy': [0.0, 0.0, 0.0],
            },
        },
    }


def test_component_specs_normalize_axes_for_three_joints(description):
    """The shared adapter must derive axes from the authoritative mapping."""
    configuration = import_module('robot_description.adapters.configuration')

    assert configuration.robot_component_configuration(description) == {
        'base': {
            'type': 'differential_drive',
            'parameters': {'wheel_radius': 0.15, 'track_width': 0.68},
        },
        'manipulator': {
            'type': 'serial_arm',
            'parameters': {
                'joint_axes': [
                    [0.0, -1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0],
                ],
                'body_height': 0.2,
                'arm_base_height': 0.08,
                'link_1_length': 0.4,
                'link_2_length': 0.36,
                'wrist_length': 0.12,
            },
        },
    }


def test_joint_configuration_preserves_names_and_limit_order(description):
    """Consumers must receive limits in exactly the configured joint order."""
    configuration = import_module('robot_description.adapters.configuration')

    assert configuration.robot_joint_configuration(description) == {
        'names': ['tip', 'yaw', 'middle'],
        'count': 3,
        'lower': [-0.3, -1.0, -0.7],
        'upper': [0.3, 1.0, 0.7],
        'velocity': [0.4, 0.6, 0.8],
        'effort': [8.0, 10.0, 12.0],
        'damping': [0.1, 0.2, 0.3],
    }


def test_mapping_parser_accepts_read_only_mappings(description):
    """Parsing an existing document must not require YAML or mutate it."""
    description['robot'] = MappingProxyType(description['robot'])
    parameters = model_parameters.robot_model_parameters_from_description(
        MappingProxyType(description)
    )

    assert parameters.manipulator.joint_axes.shape == (3, 3)
    np.testing.assert_allclose(
        parameters.joint_limits.velocity, [0.4, 0.6, 0.8]
    )
    np.testing.assert_allclose(
        parameters.tracking_frame.translation, [0.12, 0.0, 0.0]
    )


def test_file_loader_normalizes_explicit_path(description, tmp_path):
    """The established path loader must preserve normalized parameters."""
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump(description), encoding='utf-8')

    parameters = model_parameters.load_robot_model_parameters(path)

    assert parameters.base.track_width == 0.68
    assert parameters.manipulator.joint_axes.shape == (3, 3)
    np.testing.assert_allclose(
        np.linalg.norm(parameters.manipulator.joint_axes, axis=1),
        [1.0, 1.0, 1.0],
    )


@pytest.mark.parametrize('helper', [
    'robot_component_configuration', 'robot_joint_configuration',
])
def test_shared_helpers_reject_invalid_joint_axes(description, helper):
    """A zero joint axis must not reach downstream model construction."""
    description['robot']['manipulator']['joints']['tip']['axis'] = [0, 0, 0]
    configuration = import_module('robot_description.adapters.configuration')

    with pytest.raises(ValueError, match='non-zero'):
        getattr(configuration, helper)(description)


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_mapping_parser_rejects_non_finite_geometry(description, value):
    """Non-finite physical parameters must never produce usable specs."""
    description['robot']['body']['height'] = value

    with pytest.raises(ValueError, match='finite'):
        model_parameters.robot_model_parameters_from_description(description)


def test_mapping_parser_rejects_non_finite_joint_limits(description):
    """Invalid actuator limits must fail at the description boundary."""
    description['robot']['manipulator']['joints']['tip']['velocity'] = np.nan

    with pytest.raises(ValueError, match='finite'):
        model_parameters.robot_model_parameters_from_description(description)


def test_mapping_parser_rejects_non_finite_joint_axes(description):
    """Axis normalization must not silently propagate NaN values."""
    joints = description['robot']['manipulator']['joints']
    joints['tip']['axis'] = [0, np.nan, 0]

    with pytest.raises(ValueError, match='finite'):
        model_parameters.robot_model_parameters_from_description(description)


def test_component_helper_import_requires_no_ros_or_model_package():
    """Configuration extraction must work in a Python-only process."""
    package_root = Path(__file__).resolve().parents[1]
    script = """
import importlib.abc
import sys

class RejectRuntimeImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        blocked = {'ament_index_python', 'rclpy', 'robot_models'}
        if fullname.split('.')[0] in blocked:
            raise ImportError('Unexpected runtime dependency: ' + fullname)

sys.meta_path.insert(0, RejectRuntimeImports())
sys.path.insert(0, sys.argv[1])
from robot_description.adapters.configuration import robot_component_configuration
"""
    result = subprocess.run(
        [sys.executable, '-I', '-c', script, str(package_root)],
        capture_output=True, text=True, check=False,
    )

    assert result.returncode == 0, result.stderr
