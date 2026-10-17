"""Unified composition uses actual owner factories and rejects incompatible deployments."""

from importlib import import_module
from pathlib import Path

import pytest
import yaml

SRC = Path(__file__).resolve().parents[2]


def share(package):
    return SRC / package


def load(path, **kwargs):
    module = import_module('robot_bringup.factory.system_configuration')
    return module.load_system_configuration(path, share, **kwargs)


def example(name='system_nominal.yaml'):
    return SRC / 'robot_bringup/config' / name


def changed(tmp_path, section, key, value):
    data = yaml.safe_load(example().read_text())
    data[section][key] = value
    path = tmp_path / 'modified.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def test_nominal_selects_actual_owner_components_and_safe_defaults():
    plan = load(example())
    assert plan['identities']['predictor'].endswith('.IMMPredictor')
    assert plan['identities']['controller'].endswith('.WeightedLeastSquaresController')
    assert plan['identities']['robot_estimator'].endswith('.RobotStateEstimator')
    assert plan['identities']['target_estimator'].endswith('.TargetStateEstimator')
    assert plan['runtime']['enable_actuation'] is False
    assert plan['runtime']['publish_commands'] is False
    assert plan['description_path'] == (share('robot_description') / 'config/robot/robot.yaml').resolve()


def test_alternates_change_real_selected_implementations():
    cv = load(example('system_cv.yaml'))
    dls = load(example('system_dls.yaml'))
    assert cv['identities']['predictor'].endswith('.ConstantVelocityPredictor')
    assert dls['identities']['controller'].endswith('.JacobianController')


def test_explicit_launch_false_and_strategy_override_win_over_manifest():
    plan = load(example(), overrides={'enable_actuation': 'true', 'gui': 'false',
                                     'strategy_config': 'none', 'publish_commands': 'false'})
    assert plan['runtime']['enable_actuation'] is True
    assert plan['runtime']['gui'] is False
    assert plan['runtime']['publish_commands'] is False
    assert plan['identities']['controller'].endswith('.JacobianController')


@pytest.mark.parametrize('section,key,value,match', [
    ('prediction', 'type', 'constant_acceleration', 'prediction'),
    ('control', 'type', 'imaginary', 'control'),
    ('runtime', 'enable_actuation', 'true', 'runtime.enable_actuation'),
    ('runtime', 'enable_actuation', 1, 'runtime.enable_actuation'),
    ('runtime', 'guui', False, 'guui'),
    ('control', 'joint_limits', [0, 1], 'joint_limits'),
])
def test_manifest_errors_identify_invalid_path(tmp_path, section, key, value, match):
    with pytest.raises(ValueError, match=match):
        load(changed(tmp_path, section, key, value))


def test_invalid_example_fails_and_never_falls_back():
    with pytest.raises(ValueError, match='prediction'):
        load(example('system_invalid.yaml'))


def test_schema_version_is_strict(tmp_path):
    data = yaml.safe_load(example().read_text())
    data['schema_version'] = True
    path = tmp_path / 'bad.yaml'; path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='schema_version'):
        load(path)


def test_command_publication_requires_explicit_actuation():
    with pytest.raises(ValueError, match='publish_commands'):
        load(example(), overrides={'publish_commands': 'true', 'enable_actuation': 'false'})


def owner_override(tmp_path, section, content):
    """Point only one composition reference at a real modified owner document."""
    owner = tmp_path / 'owner.yaml'
    owner.write_text(yaml.safe_dump(content))
    data = yaml.safe_load(example().read_text())
    if section.startswith('estimation.'):
        data['estimation'][section.split('.')[1]]['config'] = 'owner.yaml'
    else:
        data[section]['config'] = 'owner.yaml'
    manifest = tmp_path / 'experiment.yaml'
    manifest.write_text(yaml.safe_dump(data))
    return manifest


@pytest.mark.parametrize('key,value,match', [
    ('joint_names', ['wrist_joint', 'elbow_joint', 'shoulder_joint', 'base_yaw_joint'], 'joint_names'),
    ('wheel_odom_frame', 'map', 'wheel_odom_frame'),
    ('robot_initial_state', [0.]*12, 'dimension'),
    ('joint_position_varianse', .1, 'joint_position_varianse'),
    ('joint_position_variance', float('nan'), 'finite'),
])
def test_owner_estimation_contract_mismatches_are_not_silently_overridden(tmp_path, key, value, match):
    content = yaml.safe_load((share('robot_estimation') / 'config/robot_estimator.yaml').read_text())
    content['estimation_node']['ros__parameters'][key] = value
    with pytest.raises(ValueError, match=match):
        load(owner_override(tmp_path, 'estimation.robot', content))


def test_owner_control_dof_mismatch_is_rejected(tmp_path):
    content = yaml.safe_load((share('robot_control') / 'config/control.yaml').read_text())
    content['control_node']['ros__parameters']['joint_count'] = 3
    with pytest.raises(ValueError, match='control.joint_count'):
        load(owner_override(tmp_path, 'control', content))


def test_inherited_relative_resource_uses_parent_manifest_directory(tmp_path):
    parent = tmp_path / 'parent'; parent.mkdir()
    config = share('robot_prediction') / 'config/prediction.yaml'
    (parent / 'prediction.yaml').write_bytes(config.read_bytes())
    data = yaml.safe_load(example().read_text())
    data['prediction']['config'] = 'prediction.yaml'
    (parent / 'base.yaml').write_text(yaml.safe_dump(data))
    child = tmp_path / 'child.yaml'; child.write_text('extends: parent/base.yaml\n')
    assert load(child)['prediction_path'] == parent / 'prediction.yaml'


@pytest.mark.parametrize('key,value', [('dt', True), ('dt', float('nan')), ('horizon', 2.5)])
def test_prediction_owner_rejects_invalid_time_contract(tmp_path, key, value):
    content = yaml.safe_load((share('robot_prediction') / 'config/prediction.yaml').read_text())
    content['prediction'][key] = value
    with pytest.raises(ValueError, match='prediction|finite'):
        load(owner_override(tmp_path, 'prediction', content))


def test_unknown_launch_override_is_not_ignored():
    with pytest.raises(ValueError, match='enabel_actuation'):
        load(example(), overrides={'enabel_actuation': 'true'})


def test_reference_inheritance_cycle_reports_manifest_path(tmp_path):
    path = tmp_path / 'cycle.yaml'; path.write_text('extends: cycle.yaml\n')
    with pytest.raises(ValueError, match='cycle.yaml.*cycle'):
        load(path)


def configured_launch(tmp_path, monkeypatch, **overrides):
    import importlib.util
    from launch import LaunchContext
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    path = share('robot_bringup') / 'launch/closed_loop.launch.py'
    spec = importlib.util.spec_from_file_location('m07_closed_loop', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(share(package)))
    context = LaunchContext()
    context.launch_configurations.update({key: '' for key in [
        'system_config', 'config', 'gui', 'enable_actuation', 'publish_commands',
        'visualize_tracking', 'publish_ground_truth', 'strategy_config', 'lookahead', 'feedforward']})
    context.launch_configurations.update(overrides)
    return module, context


def test_unified_launch_wires_real_cv_owner_config_and_safety_defaults(tmp_path, monkeypatch):
    from launch_ros.utilities import evaluate_parameters
    module, context = configured_launch(tmp_path, monkeypatch, system_config=str(example('system_cv.yaml')))
    actions = module.configure(context)
    nodes = {n.node_executable: n for n in actions if hasattr(n, 'node_executable')}
    prediction = evaluate_parameters(context, nodes['prediction_node']._Node__parameters)[-1]
    assert prediction['predictor_type'] == 'constant_velocity'
    assert prediction['adaptation_type'] == 'none'
    assert Path(prediction['configuration_file']).is_file()
    actuator = evaluate_parameters(context, nodes['velocity_actuation']._Node__parameters)[-1]
    control = evaluate_parameters(context, nodes['control_node']._Node__parameters)[-1]
    assert actuator['enabled'] is False
    assert control['publish_commands'] is False


def test_legacy_launch_defaults_still_use_dls_without_actuation(tmp_path, monkeypatch):
    from launch_ros.utilities import evaluate_parameters
    module, context = configured_launch(tmp_path, monkeypatch)
    nodes = {n.node_executable: n for n in module.configure(context) if hasattr(n, 'node_executable')}
    control = evaluate_parameters(context, nodes['control_node']._Node__parameters)[-1]
    assert control['strategy_config_file'] == ''
    assert control['publish_commands'] is False


def test_invalid_manifest_fails_before_any_process_action_is_constructed(tmp_path, monkeypatch):
    module, context = configured_launch(tmp_path, monkeypatch, system_config=str(example('system_invalid.yaml')))
    called = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: called.append(kwargs))
    with pytest.raises(ValueError, match='prediction'):
        module.configure(context)
    assert not called


def test_explicit_false_disables_manifest_actuation(tmp_path):
    path = changed(tmp_path, 'runtime', 'enable_actuation', True)
    plan = load(path, overrides={'enable_actuation': 'false'})
    assert plan['runtime']['enable_actuation'] is False
    assert plan['runtime']['publish_commands'] is False


@pytest.mark.parametrize('section,key,value', [
    ('control', 'validation_rate', -1.),
    ('control', 'max_robot_state_age', -1.),
    ('control', 'max_prediction_age', -1.),
    ('estimation.robot', 'full_state_publish_rate', -1.),
    ('estimation.robot', 'diagnostic_log_period', -1.),
])
def test_node_startup_constraints_also_fail_during_preflight(tmp_path, section, key, value):
    package = 'robot_control' if section == 'control' else 'robot_estimation'
    name = 'control' if section == 'control' else 'robot_estimator'
    node = 'control_node' if section == 'control' else 'estimation_node'
    content = yaml.safe_load((share(package) / 'config' / (name + '.yaml')).read_text())
    content[node]['ros__parameters'][key] = value
    # Keep the profile calibration from masking an explicitly invalid optional value.
    if section != 'control':
        profile = yaml.safe_load((share('robot_bringup') / 'config/closed_loop.yaml').read_text())
        profile['simulation_config'] = 'config/scenarios/reactive_target_oscillatory.yaml'
        (tmp_path / 'profile.yaml').write_text(yaml.safe_dump(profile))
    path = owner_override(tmp_path, section, content)
    if section != 'control':
        data = yaml.safe_load(path.read_text()); data['simulation']['config'] = 'profile.yaml'
        path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match=key + '|positive'):
        load(path)


def test_inherited_owner_parameters_are_delivered_to_ros_as_merged_values(tmp_path, monkeypatch):
    from launch_ros.utilities import evaluate_parameters
    parent = share('robot_estimation') / 'config/robot_estimator.yaml'
    (tmp_path / 'owner.yaml').write_text(f'extends: {parent}\n')
    data = yaml.safe_load(example().read_text())
    data['estimation']['robot']['config'] = 'owner.yaml'
    path = tmp_path / 'manifest.yaml'; path.write_text(yaml.safe_dump(data))
    module, context = configured_launch(tmp_path, monkeypatch, system_config=str(path))
    nodes = {n.node_executable: n for n in module.configure(context) if hasattr(n, 'node_executable')}
    evaluated = evaluate_parameters(context, nodes['estimation_node']._Node__parameters)
    merged = {}
    for item in evaluated:
        if isinstance(item, dict):
            merged.update(item)
    assert merged['joint_count'] == 4
    assert merged['reorder_holdback'] == .03
    assert not any(isinstance(item, Path) and item.name == 'owner.yaml' for item in evaluated)


def test_empty_nonmapping_overrides_are_not_defaulted():
    with pytest.raises(ValueError, match='overrides'):
        load(example(), overrides=[])


def test_prediction_horizon_must_cover_selected_control_lookahead(tmp_path):
    content = yaml.safe_load((share('robot_prediction') / 'config/prediction.yaml').read_text())
    content['prediction']['horizon'] = 1
    with pytest.raises(ValueError, match='horizon.*lookahead'):
        load(owner_override(tmp_path, 'prediction', content))
