"""Nominal, CV and DLS composition must preserve actual owners and launch behavior."""

import importlib.util
from pathlib import Path

import pytest

from robot_bringup.factory.system_configuration import load_system_configuration


import yaml
from robot_experiments.configuration import resolve_system_resource
from functools import partial
from robot_experiments.composition import load_resource
load_yaml = partial(load_resource, share=lambda package: Path(
    __file__).resolve().parents[2] / package)


SRC = Path(__file__).resolve().parents[2]
PAIRS = [('nominal', 'nominal_tracking'), ('cv', 'cv_prediction'), ('dls', 'dls_control')]


def share(package):
    """Resolve actual package resources."""
    return SRC / package


def load(path, **kwargs):
    """Construct the components through the production M07 factory."""
    return load_system_configuration(resolve_system_resource(
        path, Path.cwd(), share), share, **kwargs)


def system(name):
    """Select the additive explicit compositions."""
    return resolve_system_resource(share('robot_experiments') / f'config/systems/{name}.yaml', Path.cwd(
    ), lambda package: Path(__file__).resolve().parents[2] / package)


@pytest.mark.parametrize('old,new', PAIRS)
def test_nominal_family_has_equal_owners_and_actual_implementations(old, new):
    """An actual factory preflight must preserve the historical mathematical selections."""
    from robot_bringup.factory.system_configuration import assemble_system_configuration
    a = assemble_system_configuration(load_yaml(system(new)), system(new), share)
    b = load(system(new))
    for key in ['selection', 'identities', 'runtime', 'estimation_parameters',
                'control_parameters', 'prediction_topics', 'description_path',
                'prediction_path', 'strategy_path']:
        assert a[key] == b[key], key
    for key in ['simulation_config', 'estimation_overrides', 'actuation_config']:
        assert load_yaml(a['profile'][key]) == load_yaml(b['profile'][key])
    assert a['profile']['tracking_tf'] == b['profile']['tracking_tf']
    assert b['composition']['schema_version'] == 2
    assert b['identities']['predictor'].endswith(
        '.ConstantVelocityPredictor' if old == 'cv' else '.IMMPredictor')
    assert b['identities']['controller'].endswith(
        '.JacobianController' if old == 'dls' else '.WeightedLeastSquaresController')
    assert b['description_path'] == share('robot_description') / 'config/robot/robot.yaml'
    sensor = b['sensor_inventory']['sources']['external_planar_localization']
    assert not sensor['producer']['configured'] and not sensor['fusion']['enabled']


def test_variant_documents_express_only_the_genuine_component_differences():
    """Explicit documents retain only the intended effective family differences."""
    from copy import deepcopy
    nominal = load_yaml(system('nominal_tracking'))
    cv = load_yaml(system('cv_prediction'))
    dls = load_yaml(system('dls_control'))
    expected_cv = deepcopy(nominal)
    expected_cv['prediction'].update(type='constant_velocity', adaptation='none', fusion='none')
    assert cv == expected_cv
    expected_dls = deepcopy(nominal)
    expected_dls['control'].update(type='jacobian', strategy=None)
    assert dls == expected_dls


@pytest.mark.parametrize('mutation', [
    'predictor', 'missing_predictor', 'adaptation', 'controller', 'missing_controller',
    'strategy_conflict', 'path', 'package', 'robot', 'description', 'calibration'])
def test_nominal_family_rejects_invalid_selections_before_launch(tmp_path, mutation):
    """Existing preflight must reject malformed variant and physical owner selections."""
    data = load_yaml(system('nominal_tracking'))
    if mutation == 'predictor':
        data['prediction']['type'] = 'unknown'
    elif mutation == 'missing_predictor':
        data['prediction'].pop('type')
    elif mutation == 'adaptation':
        data['prediction']['fusion'] = 'none'
    elif mutation == 'controller':
        data['control']['type'] = 'unknown'
    elif mutation == 'missing_controller':
        data['control'].pop('type')
    elif mutation == 'strategy_conflict':
        data['control']['type'] = 'jacobian'
    elif mutation == 'path':
        data['simulation']['config'] = {
            'package': 'robot_simulation',
            'path': '../../outside.yaml'}
    elif mutation == 'package':
        data['simulation']['config'] = {
            'package': 'missing_package',
            'path': 'config/deployment.yaml'}
    elif mutation == 'robot':
        data['simulation']['robot']['path'] = 'config/gazebo/robot_wrist15.yaml'
    elif mutation == 'description':
        data['robot']['description']['path'] = 'config/robot/robot_wrist15.yaml'
    elif mutation == 'calibration':
        path = tmp_path / 'calibration.yaml'
        path.write_text('unknown_parameter: 1.0\n')
        data['estimation']['calibration'] = str(path)
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load(path)


def actions(tmp_path, monkeypatch, path, overrides, *, mapping=False):
    """Evaluate the real launch adapter without acquiring a runtime."""
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch_ros.utilities import evaluate_parameters
    spec = importlib.util.spec_from_file_location(
        'nominal_launch', share('robot_bringup') / 'launch/closed_loop.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    import robot_bringup.deployment.launching as module
    monkeypatch.setattr(module, 'get_package_share_directory', lambda p: str(share(p)))
    context = LaunchContext()
    context.launch_configurations.update(dict.fromkeys([
        'config', 'gui', 'enable_actuation', 'visualize_tracking', 'publish_ground_truth',
        'publish_commands', 'strategy_config', 'lookahead', 'feedforward'], ''))
    context.launch_configurations.update(system_config=str(path), **overrides)
    from robot_bringup.factory.system_configuration import assemble_system_configuration
    plan = (assemble_system_configuration(load_yaml(path), path, share, overrides)
            if mapping else None)
    result = module.configure(context, plan=plan)
    nodes = {str(a.node_executable): evaluate_parameters(context, a._Node__parameters)
             for a in result if hasattr(a, 'node_executable')}
    include = next(a for a in result if isinstance(a, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    return nodes, load_yaml(arguments.pop('config')), arguments


@pytest.mark.parametrize('old,new', PAIRS)
@pytest.mark.parametrize('overrides', [{}, {
    'enable_actuation': 'true', 'publish_commands': 'false', 'gui': 'false',
    'publish_ground_truth': 'true', 'lookahead': '0.1', 'feedforward': 'false'}, {
    'strategy_config': 'none', 'publish_commands': 'false'}])
def test_actual_launch_inputs_and_override_precedence_are_equal(
        tmp_path, monkeypatch, old, new, overrides):
    """Default and explicit compute-only runtime routes deliver identical ROS parameters."""
    before = actions(tmp_path, monkeypatch,
                     system(new), overrides)
    after = actions(tmp_path, monkeypatch, system(new), overrides, mapping=True)
    assert before == after
