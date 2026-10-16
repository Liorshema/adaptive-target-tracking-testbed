"""Experimental configuration and diagnostic ownership must preserve defaults."""

import importlib.util
from copy import deepcopy
from pathlib import Path

from launch import LaunchContext

import pytest

from robot_configuration.yaml_loader import load_yaml

import yaml

SOURCE = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def launch_log_directory(tmp_path, monkeypatch):
    """Keep launch action construction independent of the user's ROS log tree."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))


def test_aggressive_environment_changes_only_sinusoidal_parameters():
    """List replacement must preserve the original centering field exactly."""
    root = SOURCE / 'robot_simulation/config/scenarios'
    base = load_yaml(root / 'environment_oscillatory.yaml')
    experiment = load_yaml(root / 'environment_oscillatory_aggressive_reference.yaml')
    expected = deepcopy(base)
    expected['environment']['acceleration_fields'][1]['parameters'] = {
        'amplitude': [0., .55, .45], 'frequency': [0., .25, .35],
        'phase': [0., 0., 1.57079632679]}
    assert experiment == expected
    scene = load_yaml(root / 'reactive_target_oscillatory_aggressive.yaml')
    base_scene = load_yaml(root / 'reactive_target_oscillatory.yaml')
    base_scene['simulation']['environment_config'] = (
        'config/scenarios/environment_oscillatory_aggressive.yaml')
    assert scene == base_scene
    assert scene['tracking']['source'] == 'tf'
    master = load_yaml(
        SOURCE / 'robot_bringup/config/reactive_tracking_oscillatory_aggressive.yaml')
    assert master['simulation_config'] == (
        'config/scenarios/reactive_target_oscillatory_aggressive.yaml')


def test_allocation_changes_only_base_penalties():
    """Read plain strategy YAML exactly as the production Control loader does."""
    root = SOURCE / 'robot_control/config'
    original = load_yaml(root / 'whole_body.yaml')
    experiment = yaml.safe_load((root / 'whole_body_oscillatory_arm_preferred.yaml').read_text())
    expected = deepcopy(original)
    expected['controller']['parameters']['motion_weights'] = [3., 4., 3., 3., 3., 2.]
    assert experiment == expected


@pytest.mark.parametrize('visualize,override,expected', [
    ('false', '', ''), ('true', '', 'true'), ('false', 'true', 'true'),
    ('false', 'false', 'false'), ('true', 'true', 'true')])
def test_ground_truth_is_independent_of_marker_node(visualize, override, expected):
    """Physical diagnostics can stay enabled without an unused visualizer."""
    path = SOURCE / 'robot_bringup/launch/closed_loop.launch.py'
    spec = importlib.util.spec_from_file_location('closed_loop_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        'config': 'config/reactive_tracking_oscillatory.yaml', 'gui': 'false',
        'enable_actuation': 'false', 'publish_commands': 'false',
        'visualize_tracking': visualize, 'publish_ground_truth': override,
        'strategy_config': '', 'lookahead': '', 'feedforward': ''})
    nodes = module.configure(context)
    arguments = dict(nodes[0].launch_arguments)
    assert arguments['publish_ground_truth'] == expected
    executables = [node.node_executable
                   for node in nodes if hasattr(node, 'node_executable')]
    assert ('tracking_visualization' in executables) is (visualize == 'true')


@pytest.mark.parametrize('visualize,override', [('true', 'false'), ('false', 'invalid')])
def test_inconsistent_diagnostic_selection_is_rejected(visualize, override):
    """Do not silently create a visualization missing its physical pose input."""
    path = SOURCE / 'robot_bringup/launch/closed_loop.launch.py'
    spec = importlib.util.spec_from_file_location('closed_loop_invalid_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        'config': 'config/reactive_tracking_oscillatory.yaml', 'gui': 'false',
        'enable_actuation': 'false', 'publish_commands': 'false',
        'visualize_tracking': visualize, 'publish_ground_truth': override,
        'strategy_config': '', 'lookahead': '', 'feedforward': ''})
    with pytest.raises(ValueError, match='ground_truth|visualization'):
        module.configure(context)


def test_user_wide_stress_environment_and_live_tf_are_preserved():
    """M02 must reproduce the requested stress input without target retuning."""
    root = SOURCE / 'robot_simulation/config/scenarios'
    fields = load_yaml(root / 'environment_oscillatory_aggressive.yaml')[
        'environment']['acceleration_fields']
    assert fields[0]['parameters'] == {
        'center': [2.2, 0., .85],
        'gain_matrix': [[.25, 0., 0.], [0., .15, 0.], [0., 0., .20]]}
    assert fields[1]['parameters'] == {
        'amplitude': [0., 1., .85], 'frequency': [0., .20, .25],
        'phase': [0., 0., 1.57079632679]}
    assert load_yaml(root / 'reactive_target_oscillatory_aggressive.yaml')[
        'tracking']['source'] == 'tf'
