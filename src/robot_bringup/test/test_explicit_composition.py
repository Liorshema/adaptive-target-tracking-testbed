"""Explicit composition preserves existing owner contracts and launch assembly."""

import importlib.util
from pathlib import Path

import pytest

from robot_bringup.factory.system_configuration import load_system_configuration

from robot_configuration.yaml_loader import load_yaml

import yaml


SRC = Path(__file__).resolve().parents[2]
SYSTEM = 'config/systems/mobile_manipulator_pose_tracking.yaml'
LEGACY = 'config/system_manipulation_D_localized.yaml'


def share(package):
    """Resolve real package-owned fixtures without ROS discovery."""
    return SRC / package


def document():
    """Express the existing system with explicit owners, before adding production resources."""
    data = load_yaml(share('robot_bringup') / LEGACY)
    profile = load_yaml(share('robot_bringup') / 'config/reactive_manipulation_localized.yaml')
    data['schema_version'] = 2
    data['estimation']['calibration'] = {
        'package': 'robot_bringup', 'path': profile['estimation_overrides']}
    data['simulation'] = {
        'config': {'package': 'robot_simulation', 'path': profile['simulation_config']},
        'robot': {'package': 'robot_simulation', 'path': 'config/gazebo/robot_wrist15.yaml'},
        'actuation': {'package': 'robot_simulation', 'path': profile['actuation_config']},
        'localization_sensor': {
            'package': 'robot_simulation', 'path': 'config/sensors/planar_localization.yaml'},
        'tracking_tf': profile['tracking_tf']}
    return data


def write(tmp_path, data, name='system.yaml'):
    """Write only isolated fixture inputs."""
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data))
    return path


def load(path, **kwargs):
    """Use the real M07 assembler and component factories."""
    return load_system_configuration(path, share, **kwargs)


def test_explicit_assembly_reuses_all_existing_component_preflight(tmp_path):
    """The adapter must retain component identities, calibration, routing and safe defaults."""
    old = load(share('robot_bringup') / LEGACY)
    new = load(write(tmp_path, document()))
    for key in ['selection', 'identities', 'runtime', 'estimation_parameters',
                'control_parameters', 'prediction_topics', 'description_path', 'strategy_path']:
        assert new[key] == old[key], key
    assert new['composition']['schema_version'] == 2
    assert new['profile']['tracking_tf'] == old['profile']['tracking_tf']
    assert new['estimation_parameters']['wheel_linear_velocity_variance'] == .02
    assert new['estimation_parameters']['wheel_angular_velocity_variance'] == .04


@pytest.mark.parametrize('version', [True, 0, 3, 2.0, '2'])
def test_explicit_version_rejects_noninteger_or_unsupported_values(tmp_path, version):
    """Do not reinterpret a malformed manifest as a legacy profile."""
    data = document()
    data['schema_version'] = version
    with pytest.raises(ValueError, match='schema_version'):
        load(write(tmp_path, data))


@pytest.mark.parametrize('section,key', [
    ('robot', 'description'), ('robot', 'model'), ('estimation', 'robot'),
    ('estimation', 'target'), ('prediction', 'type'), ('prediction', 'config'),
    ('control', 'type'), ('simulation', 'config'), ('simulation', 'robot'),
    ('simulation', 'actuation'), ('simulation', 'tracking_tf')])
def test_required_component_selections_cannot_be_omitted(tmp_path, section, key):
    """Every selected role has an explicit validated authority."""
    data = document()
    data[section].pop(key)
    with pytest.raises(ValueError):
        load(write(tmp_path, data))


@pytest.mark.parametrize('mutation,match', [
    ('robot_description', 'robot.description'), ('robot_deployment', 'simulation.robot'),
    ('sensor', 'localization_sensor'), ('calibration', 'robot_description_config'),
    ('calibration_unknown', 'unknown'), ('overlap', 'competing parameter owners'),
    ('predictor', 'prediction'), ('model', 'model'), ('controller', 'control'),
    ('runtime', 'runtime'), ('actuation', 'actuation'), ('tf', 'tracking_tf'),
    ('profile_as_deployment', 'simulation'), ('v1_with_v2_fields', 'simulation|estimation'),
])
def test_explicit_conflicts_and_invalid_owners_fail_closed(tmp_path, mutation, match):
    """Conflicting sources never silently replace physical, fusion or safety policy."""
    data = document()
    sim = data['simulation']
    if mutation == 'robot_description':
        data['robot']['description']['path'] = 'config/robot/robot.yaml'
    elif mutation == 'robot_deployment':
        sim['robot']['path'] = 'config/gazebo/robot.yaml'
    elif mutation == 'sensor':
        sim['localization_sensor']['path'] = 'config/sensors/planar_localization_noise_study.yaml'
    elif mutation in ['calibration', 'calibration_unknown']:
        cal = load_yaml(share('robot_bringup') / 'config/calibration/manipulation_pose.yaml')
        cal['robot_description_config' if mutation == 'calibration' else 'unknown_key'] = (
            'config/robot/robot.yaml' if mutation == 'calibration' else 1.)
        data['estimation']['calibration'] = str(write(tmp_path, cal, 'calibration.yaml'))
    elif mutation == 'overlap':
        target = load_yaml(share('robot_estimation') / 'config/target_estimator.yaml')
        target['estimation_node']['ros__parameters']['wheel_linear_velocity_variance'] = .1
        data['estimation']['target']['config'] = str(write(tmp_path, target, 'target.yaml'))
    elif mutation == 'predictor':
        data['prediction']['type'] = 'unknown'
    elif mutation == 'model':
        data['robot']['model']['base'] = 'unknown'
    elif mutation == 'controller':
        data['control']['type'] = 'unknown'
    elif mutation == 'runtime':
        data['runtime']['enable_actuation'] = 'true'
    elif mutation == 'actuation':
        actuator = load_yaml(share('robot_simulation') / 'config/actuation.yaml')
        actuator['actuation']['command_topic'] = '/wrong'
        sim['actuation'] = str(write(tmp_path, actuator, 'actuation.yaml'))
    elif mutation == 'tf':
        sim['tracking_tf']['rate_hz'] = 0.
    elif mutation == 'profile_as_deployment':
        sim['config'] = {'package': 'robot_bringup', 'path': 'config/closed_loop.yaml'}
    elif mutation == 'v1_with_v2_fields':
        data['schema_version'] = 1
    with pytest.raises(ValueError, match=match):
        load(write(tmp_path, data))


@pytest.mark.parametrize('reference', [
    {'package': 'robot_bringup', 'path': '../../outside.yaml'},
    {'package': 'robot_bringup', 'path': '/etc/passwd'},
    {'package': 'Not_A_Package', 'path': 'config/missing.yaml'},
    {'package': 'missing_package', 'path': 'config/missing.yaml'},
    {'package': 'robot_bringup', 'path': 'config/missing.yaml'},
    {'package': 'robot_bringup'},
])
def test_new_resource_slots_preserve_shared_path_protection(tmp_path, reference):
    """New selectors use the shared resolver, including package traversal checks."""
    data = document()
    data['estimation']['calibration'] = reference
    with pytest.raises(ValueError, match='calibration|Resource|resource'):
        load(write(tmp_path, data))


def test_new_reference_resolves_at_declaring_parent_and_runtime_overrides_win(tmp_path):
    """Keep inheritance origins and explicit false runtime/strategy precedence."""
    parent = tmp_path / 'parent'
    parent.mkdir()
    data = document()
    cal = load_yaml(share('robot_bringup') / 'config/calibration/manipulation_pose.yaml')
    write(parent, cal, 'calibration.yaml')
    data['estimation']['calibration'] = 'calibration.yaml'
    write(parent, data, 'base.yaml')
    path = write(tmp_path, {'extends': 'parent/base.yaml'})
    plan = load(path, overrides={'enable_actuation': 'true', 'publish_commands': 'false',
                                 'strategy_config': 'none'})
    assert plan['profile']['estimation_overrides'] == parent / 'calibration.yaml'
    assert plan['runtime']['enable_actuation'] is True
    assert plan['runtime']['publish_commands'] is False
    assert plan['identities']['controller'].endswith('.JacobianController')
    with pytest.raises(ValueError, match='profile|schema 2'):
        load(path, overrides={'config': 'config/closed_loop.yaml'})


def launch(tmp_path, monkeypatch, system):
    """Construct actual launch actions without starting any process."""
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        's3a_launch', share('robot_bringup') / 'launch/closed_loop.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda p: str(share(p)))
    context = LaunchContext()
    context.launch_configurations.update({key: '' for key in [
        'config', 'gui', 'enable_actuation', 'visualize_tracking', 'publish_ground_truth',
        'publish_commands', 'strategy_config', 'lookahead', 'feedforward']})
    context.launch_configurations['system_config'] = str(system)
    return module, context


def test_production_composition_is_shallow_and_delivers_equal_launch_parameters(
        tmp_path, monkeypatch):
    """Both routes construct the same node parameters and simulation deployment values."""
    from launch.actions import IncludeLaunchDescription
    from launch_ros.utilities import evaluate_parameters
    plans = []
    for system in [LEGACY, SYSTEM]:
        module, context = launch(tmp_path, monkeypatch, share('robot_bringup') / system)
        actions = module.configure(context)
        nodes = {str(a.node_executable): evaluate_parameters(context, a._Node__parameters)
                 for a in actions if hasattr(a, 'node_executable')}
        include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
        arguments = dict(include.launch_arguments)
        plans.append((nodes, load_yaml(arguments['config']),
                      {k: v for k, v in arguments.items() if k != 'config'}))
    assert plans[0] == plans[1]
    raw = yaml.safe_load((share('robot_bringup') / SYSTEM).read_text())
    assert 'extends' not in raw
    assert raw['simulation']['config']['package'] == 'robot_simulation'


def test_bad_direct_composition_is_rejected_before_constructing_process_actions(
        tmp_path, monkeypatch):
    """Preflight must run before any Node is constructed."""
    data = document()
    data['simulation']['robot']['path'] = 'config/gazebo/robot.yaml'
    module, context = launch(tmp_path, monkeypatch, write(tmp_path, data))
    calls = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: calls.append(kwargs))
    with pytest.raises(ValueError, match='simulation.robot'):
        module.configure(context)
    assert not calls


@pytest.mark.parametrize('section,key', [
    ('world', 'source'), ('world', 'terrain_config'), ('target', 'config'),
    ('robot', 'config'), ('robot', 'joint_position_hold'), ('simulation', 'config'),
    ('simulation', 'environment_config'), ('simulation', 'measurement_config')])
def test_direct_deployment_cannot_use_manifest_resource_syntax(tmp_path, section, key):
    """Preflight must reject resource forms that the unchanged simulation launcher cannot read."""
    data = document()
    deployment = load_yaml(share('robot_simulation') / data['simulation']['config']['path'])
    deployment[section][key] = {'package': 'robot_simulation', 'path': deployment[section][key]}
    data['simulation']['config'] = str(write(tmp_path, deployment, 'deployment.yaml'))
    with pytest.raises(ValueError, match='deployment resource'):
        load(write(tmp_path, data))


def test_tracking_numeric_inputs_are_delivered_as_ros_doubles(tmp_path):
    """Integer YAML numbers must not pass preflight then fail typed ROS declarations."""
    data = document()
    data['simulation']['tracking_tf']['rate_hz'] = 20
    data['simulation']['tracking_tf']['future_tolerance'] = 0
    plan = load(write(tmp_path, data))
    assert type(plan['profile']['tracking_tf']['rate_hz']) is float
    assert type(plan['profile']['tracking_tf']['future_tolerance']) is float
    assert plan['profile']['tracking_tf']['rate_hz'] == 20.


def test_manifest_estimator_types_remain_authoritative_over_calibration(tmp_path, monkeypatch):
    """Preserve M07's existing typed selector authority through actual launch parameters."""
    from launch_ros.utilities import evaluate_parameters
    data = document()
    cal = load_yaml(share('robot_bringup') / 'config/calibration/manipulation_pose.yaml')
    cal.update(robot_estimator_type='not_real', target_estimator_type='not_real')
    data['estimation']['calibration'] = str(write(tmp_path, cal, 'calibration.yaml'))
    path = write(tmp_path, data)
    plan = load(path)
    assert plan['identities']['robot_estimator'].endswith('.RobotStateEstimator')
    assert plan['identities']['target_estimator'].endswith('.TargetStateEstimator')
    module, context = launch(tmp_path, monkeypatch, path)
    node = next(a for a in module.configure(context)
                if getattr(a, 'node_executable', None) == 'estimation_node')
    effective = {}
    for values in evaluate_parameters(context, node._Node__parameters):
        effective.update(values)
    assert effective['robot_estimator_type'] == 'robot_ekf'
    assert effective['target_estimator_type'] == 'kalman'


@pytest.mark.parametrize('selection', ['omitted', 'null'])
def test_optional_localization_can_be_absent_when_deployment_has_no_producer(tmp_path, selection):
    """Do not invent a requirement for a built-in producer when fusion is not selected."""
    data = document()
    deployment = load_yaml(share('robot_simulation') / data['simulation']['config']['path'])
    deployment.pop('localization_sensor_config')
    data['simulation']['config'] = str(write(tmp_path, deployment, 'deployment.yaml'))
    if selection == 'omitted':
        data['simulation'].pop('localization_sensor')
    else:
        data['simulation']['localization_sensor'] = None
    data['estimation']['robot']['config']['path'] = 'config/robot_estimator.yaml'
    plan = load(write(tmp_path, data))
    sensor = plan['sensor_inventory']['sources']['external_planar_localization']
    assert not sensor['producer']['configured']
    assert not sensor['fusion']['enabled']
