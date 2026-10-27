"""One existing deployment plus optional read-only model/RViz, no second control stack."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
from robot_experiments.configuration import resolve_system_resource

SRC = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def writable_launch_logs(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros-log'))


def module():
    path = SRC/'robot_bringup/launch/visualization.launch.py'
    spec = importlib.util.spec_from_file_location('visualization_launch', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def context(**overrides):
    c = LaunchContext()
    c.launch_configurations.update({
        'system_config': str(resolve_system_resource(SRC / 'robot_experiments/config/systems/nominal_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)),
        'visualization_config': 'config/tracking_visualization.yaml',
        'gui': 'false', 'rviz': 'false', 'enable_actuation': 'false', **overrides})
    return c


def test_installed_wrapper_has_single_existing_system_and_prefixed_rsp(monkeypatch):
    m = module()
    monkeypatch.setattr(m, 'get_package_share_directory', lambda name: str(SRC/name))
    actions = m.configure(context())
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 1
    included = includes[0].launch_description_source.get_launch_description(context())
    assert 'strategy_config' in [
        e.name for e in included.entities if isinstance(
            e, DeclareLaunchArgument)]
    nodes = [a for a in actions if isinstance(a, Node)]
    assert len(nodes) == 1 and nodes[0].node_executable == 'robot_state_publisher'
    args = dict(includes[0].launch_arguments)
    assert args['enable_actuation'] == 'false' and args['visualize_tracking'] == 'true'
    assert args['gui'] == 'false'
    assert evaluate_parameters(context(), nodes[0]._Node__parameters)[
        0]['frame_prefix'] == 'visual/'


def test_bad_visual_config_rejects_before_deployment_actions(tmp_path, monkeypatch):
    m = module()
    monkeypatch.setattr(m, 'get_package_share_directory', lambda name: str(SRC/name))
    p = tmp_path/'bad.yaml'
    p.write_text('schema_version: 9\n')
    with pytest.raises(ValueError):
        m.configure(context(visualization_config=str(p)))


def test_gui_preflight_does_not_invent_environment_values():
    m = module()
    assert m.has_display({}) is False
    assert m.has_display({'DISPLAY': ':99'}) is True
    assert m.has_display({'WAYLAND_DISPLAY': 'actual-display'}) is True


def test_namespaced_robot_model_consumes_actual_joint_states(monkeypatch):
    m = module()
    monkeypatch.setattr(m, 'get_package_share_directory', lambda name: str(SRC/name))
    actions = m.configure(context())
    node = next(a for a in actions if isinstance(a, Node))
    remappings = node._Node__remappings
    assert any('joint_states' == src[0].perform(
        context()) and '/joint_states' == dst[0].perform(context()) for src, dst in remappings)


def test_missing_display_keeps_headless_system_and_no_gui_clients(monkeypatch):
    from launch.actions import ExecuteProcess
    m = module()
    monkeypatch.setattr(m, 'get_package_share_directory', lambda name: str(SRC/name))
    monkeypatch.setattr(m, 'has_display', lambda: False)
    actions = m.configure(context(gui='true', rviz='true'))
    assert sum(isinstance(a, IncludeLaunchDescription) for a in actions) == 1
    assert not any(isinstance(a, ExecuteProcess) and not isinstance(a, Node) for a in actions)
    assert [a.node_executable for a in actions if isinstance(a, Node)] == ['robot_state_publisher']
