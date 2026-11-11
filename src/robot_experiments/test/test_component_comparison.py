"""Controlled component variables use real owner preflight and exact projections."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from robot_configuration.resources import resolve_resource
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.components import describe_component
from robot_experiments.comparison_variables import (
    project_components, validate_component_policy,
)
from robot_experiments.configuration import load_experiment, materialize_run
from robot_experiments.configuration import resolve_system_resource


SRC = Path(__file__).resolve().parents[2]
SHARE = lambda name: SRC / name


def policy(domain, path, values=None):
    variable = {'name': 'studied_variable', 'path': path}
    if values is not None:
        variable['values'] = values
    return {'schema_version': 2, 'domain': domain, 'variables': [variable]}


@pytest.fixture(scope='module')
def materialized(tmp_path_factory):
    root = tmp_path_factory.mktemp('component-comparison')
    count = 0

    def prepare(*, selection='nominal_tracking', delta=None, overrides=None, dynamics_model=None,
                owner_share=None):
        nonlocal count
        count += 1
        lookup = owner_share or SHARE
        source = resolve_system_resource({
            'package': 'robot_experiments', 'path': 'config/experiments.yaml',
            'group': ['compositions.common', 'compositions.selections.' + selection],
        }, root, lookup)
        if dynamics_model:
            manifest = load_yaml(source)
            simulation = load_yaml(resolve_resource(manifest['simulation']['config'], source.parent, lookup))
            target = load_yaml(resolve_resource(simulation['target']['config'], source.parent, lookup))
            old = target['target']['dynamics']['parameters']
            metadata = describe_component('dynamics', dynamics_model)
            shared = {key: value for key, value in old.items()
                      if key in metadata['schema']['properties']}
            target['target']['dynamics'] = {
                'type': dynamics_model, 'parameters': {**metadata['defaults'], **shared}}
            target_file = root / f'target-{count}.yaml'
            target_file.write_text(yaml.safe_dump(target))
            simulation['target']['config'] = str(target_file)
            simulation_file = root / f'simulation-{count}.yaml'
            simulation_file.write_text(yaml.safe_dump(simulation))
            delta = {**(delta or {}), 'simulation': {'config': str(simulation_file)}}
        if delta:
            system = root / f'system-{count}.yaml'
            system.write_text(yaml.safe_dump({'extends': str(source), **delta}))
            source = system
        document = {
            'schema_version': 1,
            'experiment': {
                'name': 'component-audit', 'duration': 2., 'settling_time': 1.,
                'seeds': [42], 'variants': [{'name': 'condition', 'system_config': str(source)}],
                'metrics': ['tracking_distance', 'prediction_rmse'],
                'output': {'directory': str(root / 'results')},
                'runtime': {'enable_actuation': False},
            },
        }
        path = root / f'experiment-{count}.yaml'
        path.write_text(yaml.safe_dump(document))
        spec = load_experiment(path, lookup)
        if overrides:
            spec['variants'][0]['owner_overrides'] = overrides
        run = root / f'run-{count}'
        run.mkdir()
        return materialize_run(spec['variants'][0], 42, run, lookup, spec['runtime'])

    return prepare


def project(prepared, declaration):
    return project_components(prepared['comparison_contract'],
                              prepared['scientific_conditions'], declaration)


def test_real_predictor_choice_preserves_every_other_condition(materialized):
    a = materialized()
    b = materialized(delta={'prediction': {'type': 'constant_velocity'}})
    declaration = policy('component_models', 'components.predictor', ['imm', 'constant_velocity'])
    before = deepcopy(a)
    left, right = project(a, declaration), project(b, declaration)
    assert left[:2] == right[:2]
    assert left[2]['studied_variable']['value'] == 'imm'
    assert right[2]['studied_variable']['value'] == 'constant_velocity'
    assert a == before
    assert left[1]['prediction_configuration'] == a['scientific_conditions']['prediction_configuration']
    b['scientific_conditions']['component_selection']['adaptation_type'] = 'none'
    assert project(a, declaration)[:2] != project(b, declaration)[:2]


def test_real_jacobian_gain_change_projects_only_active_gain_leaf(materialized):
    a = materialized(selection='dls_control')
    b = materialized(selection='dls_control', overrides={
        'control': {'controller_task_gains': [2., 2., 2.]}})
    declaration = policy('component_parameters', 'parameters.controller.gains',
                         [[1., 1., 1.], [2., 2., 2.]])
    assert project(a, declaration)[:2] == project(b, declaration)[:2]
    invariant = project(a, declaration)[1]['controller_parameters']
    assert invariant['controller_damping'] == 1.
    assert invariant['safety_enabled'] is True
    assert 'controller_task_gains' not in invariant
    b['scientific_conditions']['controller_parameters']['controller_damping'] *= 2
    assert project(a, declaration)[:2] != project(b, declaration)[:2]


def test_real_strategy_gain_change_does_not_mask_unused_flat_parameters(materialized):
    a = materialized()
    b = materialized(overrides={'strategy': {'controller': {
        'parameters': {'gains': [2., 6., 6.]}}}})
    declaration = policy('component_parameters', 'parameters.controller.gains')
    assert project(a, declaration)[:2] == project(b, declaration)[:2]
    b['scientific_conditions']['controller_parameters']['controller_task_gains'] = [3., 3., 3.]
    assert project(a, declaration)[:2] != project(b, declaration)[:2]


def test_real_predictor_parameter_and_estimator_process_parameter(materialized):
    a = materialized()
    prediction = materialized(overrides={'prediction': {'predictors': {'imm': {
        'parameters': {'likelihood_variance': .5}}}}})
    declaration = policy('component_parameters', 'parameters.predictor.likelihood_variance')
    assert project(a, declaration)[:2] == project(prediction, declaration)[:2]
    estimation = materialized(overrides={'estimation': {'linear_acceleration_noise_std': .2}})
    declaration = policy('component_parameters',
                         'parameters.robot_estimator.linear_acceleration_noise_std')
    assert project(a, declaration)[:2] == project(estimation, declaration)[:2]


def test_estimator_alias_identity_does_not_mask_bindings_or_algorithm_parameters(materialized):
    a = materialized()
    b = materialized(delta={'estimation': {'robot': {'type': 'ekf'}}})
    declaration = policy('component_models', 'components.robot_estimator', ['robot_ekf', 'ekf'])
    assert project(a, declaration)[:2] == project(b, declaration)[:2]
    assert 'robot_algorithm_parameters' in project(a, declaration)[1]['estimator_parameters']
    b['scientific_conditions']['estimator_parameters']['robot_process_model'] = 'different'
    with pytest.raises(ValueError, match='binding|process_model'):
        project(b, declaration)


def test_target_dynamics_parameter_remains_a_dynamics_study(materialized):
    a = materialized()
    b = materialized(overrides={'target': {'target': {'dynamics': {'parameters': {'a_max': .3}}}}})
    declaration = policy('target_dynamics', 'parameters.target_dynamics.a_max')
    assert project(a, declaration)[:2] == project(b, declaration)[:2]
    assert project(a, declaration)[2]['studied_variable']['path'].startswith('parameters.target_dynamics.')


def test_descriptive_ranking_keeps_declared_different_driving_dynamics_separate(materialized):
    from robot_experiments.benchmarking import rank_descriptive_runs
    from robot_experiments.comparison import comparison_views
    from test_descriptive_ranking import observed

    a = materialized()
    b = materialized(overrides={'target': {'target': {'dynamics': {'parameters': {'a_max': .3}}}}})
    declaration = policy('target_dynamics', 'parameters.target_dynamics.a_max')
    rows = [observed('a', .3), observed('b', .4)]
    for row, prepared in zip(rows, [a, b]):
        contract = {**prepared['comparison_contract'], 'duration': 2., 'settling_time': 1.}
        row['comparison'] = comparison_views(contract, declaration,
                                             prepared['scientific_conditions'])
        row['metrics']['evaluation_window'] = {'start': 1., 'end': 3., 'unit': 's'}
    assert rows[0]['comparison']['invariant_contract'] == rows[1]['comparison']['invariant_contract']
    result = rank_descriptive_runs(rows)
    assert len(result['scopes']) == 2, result
    assert not result['eligible']


def test_model_specific_dynamics_defaults_are_exact_and_shared_leaves_stay_protected(materialized):
    a = materialized()
    b = materialized(dynamics_model='fight_flight')
    dynamics = b['comparison_contract']['target_dynamics']['target']['dynamics']
    declaration = policy('target_dynamics', 'components.target_dynamics',
                         ['saturated_fight_flight', 'fight_flight'])
    assert project(a, declaration)[:2] == project(b, declaration)[:2]
    dynamics['parameters']['damping'] *= 2
    assert project(a, declaration)[:2] != project(b, declaration)[:2]
    dynamics['parameters']['alpha'] = 2.
    with pytest.raises(ValueError, match='default|undeclared'):
        project(b, declaration)


@pytest.mark.parametrize('path', [
    'parameters.controller', 'parameters.controller.*', 'parameters.controller.safety_enabled',
    'parameters.controller.base_velocity_lower', 'parameters.predictor.dt',
    'parameters.robot_estimator.joint_count', 'parameters.robot_estimator.joint_bounds_enabled',
    'parameters.robot_estimator.max_joint_state_age',
    'parameters.robot_estimator.robot_process_model',
    'parameters.robot_estimator.robot_algorithm_parameters',
    'parameters.robot_estimator.localization_enabled',
    'parameters.target_estimator.target_spatial_dimension',
    'parameters.target_dynamics.tangential',
])
def test_protected_or_container_paths_are_rejected(path):
    domain = 'target_dynamics' if 'target_dynamics' in path else 'component_parameters'
    with pytest.raises(ValueError):
        validate_component_policy(policy(domain, path))


def test_typed_policy_uses_registered_schema_values_and_registered_role_choices():
    with pytest.raises(ValueError):
        validate_component_policy(policy('component_models', 'components.predictor'))
    with pytest.raises(ValueError):
        validate_component_policy(policy('component_models', 'components.predictor', ['unregistered']))
    with pytest.raises(ValueError):
        validate_component_policy(policy('component_parameters', 'parameters.controller.damping', [-1.]))
    with pytest.raises(ValueError):
        validate_component_policy(policy('component_models', 'components.adaptation', ['none']))
    with pytest.raises(ValueError):
        validate_component_policy(policy('component_parameters', 'parameters.target_dynamics.a_max'))
    declaration = policy('component_parameters', 'parameters.controller.gains', [[1., 2., 3.]])
    assert validate_component_policy(declaration) == declaration


def test_missing_mirror_conflicting_identity_and_out_of_policy_value_fail_closed(materialized):
    a = materialized()
    declaration = policy('component_models', 'components.predictor', ['constant_velocity'])
    with pytest.raises(ValueError, match='permitted'):
        project(a, declaration)
    declaration = policy('component_parameters', 'parameters.predictor.likelihood_variance')
    del a['scientific_conditions']['prediction_configuration']['predictors']['imm']['parameters']['likelihood_variance']
    with pytest.raises(ValueError, match='missing|absent|required'):
        project(a, declaration)
    declaration = policy('component_models', 'components.predictor', ['imm', 'constant_velocity'])
    with pytest.raises(ValueError, match='missing|absent|required'):
        project(a, declaration)


def test_existing_default_controller_pair_keeps_task_and_lookahead_mismatch(materialized):
    a = materialized()
    b = materialized(selection='dls_control')
    declaration = policy('component_models', 'components.controller',
                         ['weighted_least_squares', 'jacobian'])
    assert project(a, declaration)[:2] != project(b, declaration)[:2]


def test_multiple_intrinsic_estimator_leaves_keep_each_projection(materialized, monkeypatch):
    """A future owner algorithm can add leaves without changing comparison code."""
    from robot_configuration.components import schema_for
    from robot_estimation.factory.algorithm_parameters import encode_algorithm_parameters
    from robot_experiments import components

    a = materialized()
    registries = components.component_registries()
    defaults = {'ridge': 1., 'scale': 2.}
    schema = schema_for(defaults)
    registries['estimator'].register('test_intrinsic', lambda **kwargs: None,
                                   defaults=defaults, schema=schema)
    metadata = registries['target_estimator'].describe('kf')
    metadata['defaults']['algorithm_parameters'] = defaults
    metadata['schema']['properties']['algorithm_parameters'] = schema
    metadata['capabilities']['algorithm'] = 'test_intrinsic'
    registries['target_estimator'].register(
        'test_target', lambda **kwargs: None,
        defaults=metadata['defaults'], schema=metadata['schema'],
        capabilities=metadata['capabilities'])
    monkeypatch.setattr(components, 'component_registries', lambda: registries)
    a['scientific_conditions']['component_selection']['target_estimator_type'] = 'test_target'
    a['scientific_conditions']['estimator_parameters']['target_algorithm_parameters'] = (
        encode_algorithm_parameters('test_intrinsic', defaults))
    b = deepcopy(a)
    b['scientific_conditions']['estimator_parameters']['target_algorithm_parameters'] = (
        encode_algorithm_parameters('test_intrinsic', {'ridge': 3., 'scale': 4.}))
    declaration = policy('component_parameters', 'parameters.target_estimator.algorithm_parameters.ridge')
    declaration['variables'].append({
        'name': 'scale', 'path': 'parameters.target_estimator.algorithm_parameters.scale'})
    assert project(a, declaration)[:2] == project(b, declaration)[:2]


def test_comparison_options_discover_exact_registered_paths_and_are_detached():
    from robot_experiments.comparison_variables import comparison_options

    options = {item['path']: item for item in comparison_options()}
    predictor = options['components.predictor']
    assert predictor == {
        'path': 'components.predictor', 'domain': 'component_models', 'kind': 'model',
        'role': 'predictor', 'model_values': ['constant_velocity', 'imm']}
    assert options['parameters.target_dynamics.tangential.acceleration']['domain'] == 'target_dynamics'
    assert options['parameters.controller.gains']['kind'] == 'parameter'
    assert 'parameters.robot_estimator.joint_bounds_enabled' not in options
    assert 'parameters.robot_estimator.max_joint_state_age' not in options
    assert 'parameters.robot_estimator.robot_process_model' not in options
    assert 'parameters.target_dynamics.tangential' not in options
    for option in options.values():
        declaration = policy(option['domain'], option['path'],
                             option['model_values'] if option['kind'] == 'model' else None)
        assert validate_component_policy(declaration) == declaration
    predictor['model_values'].clear()
    assert comparison_options()[0]['model_values']


def test_real_registered_estimator_choice_projects_only_default_intrinsic_leaves(
        materialized, tmp_path):
    import importlib.util
    from robot_estimation.factory.algorithm_parameters import encode_algorithm_parameters

    specification = importlib.util.spec_from_file_location(
        'comparison_model_context', Path(__file__).parent / 'model_fixtures/context.py')
    fixture = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(fixture)
    with fixture.model_context(tmp_path / 'owners', SHARE) as isolated_share:
        a = materialized(delta={'estimation': {'target': {'type': 'kf'}}}, owner_share=isolated_share)
        b = materialized(delta={'estimation': {'target': {'type': 'example_fading_kf'}}},
                         owner_share=isolated_share)
        declaration = policy('component_models', 'components.target_estimator',
                             ['kf', 'example_fading_kf'])
        assert project(a, declaration)[:2] == project(b, declaration)[:2]
        b['scientific_conditions']['estimator_parameters']['target_position_variance'][0] *= 2
        assert project(a, declaration)[:2] != project(b, declaration)[:2]
        b['scientific_conditions']['estimator_parameters']['target_algorithm_parameters'] = (
            encode_algorithm_parameters('example_fading_kf', {'fading_factor': 2.}))
        with pytest.raises(ValueError, match='default|undeclared'):
            project(b, declaration)
