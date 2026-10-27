"""Composition and seed materialization contracts, independent of ROS."""
from pathlib import Path
import pytest
import yaml
from robot_experiments.configuration import load_experiment, materialize_run
from functools import partial
from robot_experiments.composition import load_resource
from robot_experiments.configuration import resolve_system_resource
load_yaml = partial(load_resource, share=lambda package: Path(__file__).resolve().parents[2] / package)

SRC=Path(__file__).resolve().parents[2]
SHARE=lambda name:SRC/name


def test_localization_materialization_uses_independent_seed_and_records_contract(tmp_path):
    """The resolved-run inventory must describe materialized resources and seeds."""
    spec = load_experiment(document(tmp_path, variants=[{
        'name': 'localized', 'system_config': {
            'package': 'robot_bringup',
            'path': 'config/system_manipulation_D_localized.yaml'}}]), SHARE)
    run = tmp_path / 'run'
    run.mkdir()
    prepared = materialize_run(spec['variants'][0], 917, run, SHARE, spec['runtime'])
    sim = load_yaml(prepared['simulation_path'])
    sensor = load_yaml(sim['localization_sensor_config'])['localization_sensor']
    target = load_yaml(sim['simulation']['measurement_config'])['measurements']['target_position']
    assert sensor['random_seed'] != target['random_seed']
    assert target['random_seed'] == 917
    assert prepared['context']['topics']['localization_measurement'] == sensor['topic']
    assert prepared['comparison_contract']['localization_sensor']['localization_sensor'] == sensor
    assert prepared['snapshot']['seed_consumers']['localization_sensor.random_seed'] == (
        sensor['random_seed'])

    assert 'sensor_inventory' in prepared['plan'], (
        'resolved-run metadata must include M07 inventory')
    sources = prepared['plan']['sensor_inventory']['sources']
    assert sources['external_planar_localization']['simulated_effects']['random_seed'] == (
        sensor['random_seed'])
    assert sources['target_position']['simulated_effects']['random_seed'] == 917
    assert sources['external_planar_localization']['configuration']['producer_resource'] == (
        sim['localization_sensor_config'])

def document(tmp_path,**changes):
 data={'schema_version':1,'experiment':{'name':'paired','duration':2.,'settling_time':1.,'seeds':[42],
 'variants':[{'name':'imm','system_config':{'package':'robot_bringup','path':'config/system_nominal.yaml'}}],
 'metrics':['tracking_distance','pointing_error','prediction_rmse','joint_speed_violation','realtime_factor'],
 'output':{'directory':'results/experiments'},'runtime':{'enable_actuation':True}}}
 data['experiment'].update(changes)
 p=tmp_path/'experiment.yaml';p.write_text(yaml.safe_dump(data));return p

def test_real_owner_preflight_and_seed_reaches_measurement_config(tmp_path):
 spec=load_experiment(document(tmp_path),SHARE)
 run=tmp_path/'run';run.mkdir()
 result=materialize_run(spec['variants'][0],917,run,SHARE,spec['runtime'])
 assert result['plan']['identities']['predictor'].endswith('.IMMPredictor')
 context=result['context']
 assert context['topics']['base_truth']=='/simulation/robot/ground_truth'
 sim=yaml.safe_load(result['simulation_path'].read_text())
 measured=yaml.safe_load(Path(sim['simulation']['measurement_config']).read_text())
 assert measured['measurements']['target_position']['random_seed']==917
 assert yaml.safe_load((SRC/'robot_simulation/config/measurements.yaml').read_text())['measurements']['target_position']['random_seed']==42
 assert result['context']['description']==yaml.safe_load((SRC/'robot_description/config/robot/robot.yaml').read_text())
 assert result['plan']['runtime']['enable_actuation'] is True
 assert result['snapshot']['seed_consumers']
 assert result['plan']['description_path'].is_relative_to(run)

@pytest.mark.parametrize('change',[{'duration':True},{'duration':0},{'settling_time':-1},{'seeds':[1,1]},
 {'seeds':[True]},{'metrics':['imaginary']},{'variants':[{'name':'a','system_config':'missing.yaml'}]},
 {'runtime':{'enable_actuation':'yes'}},{'execution':{'wall_timeout':0}},{'typo':1}])
def test_invalid_configs_fail_early(tmp_path,change):
 with pytest.raises(ValueError):load_experiment(document(tmp_path,**change),SHARE)

def test_declaring_file_relative_resources_survive_inheritance(tmp_path):
 base=tmp_path/'parent';base.mkdir()
 source=document(base)
 source_data=yaml.safe_load(source.read_text())
 source_data['experiment']['variants'][0]['system_config']=str(resolve_system_resource(SRC/'robot_experiments/config/systems/cv_prediction.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package))
 source.write_text(yaml.safe_dump(source_data))
 child=tmp_path/'child.yaml';child.write_text('extends: parent/experiment.yaml\nexperiment:\n  seeds: [43]\n')
 spec=load_experiment(child,SHARE)
 assert spec['seeds']==[43]
 assert spec['variants'][0]['plan']['identities']['predictor'].endswith('.ConstantVelocityPredictor')

def test_prediction_horizon_is_part_of_matched_comparison_contract(tmp_path):
 spec=load_experiment(document(tmp_path),SHARE)
 run=tmp_path/'run';run.mkdir()
 result=materialize_run(spec['variants'][0],917,run,SHARE,spec['runtime'])
 assert result['comparison_contract']['prediction_sampling']['horizon']>0
 assert result['comparison_contract']['prediction_sampling']['dt']>0


def test_optional_physics_trace_is_run_local_and_not_part_of_physics_contract(tmp_path):
 spec=load_experiment(document(tmp_path,execution={'record_actuator_trace':True}),SHARE)
 run=tmp_path/'run';run.mkdir()
 result=materialize_run(spec['variants'][0],42,run,SHARE,spec['runtime'],record_actuator_trace=True)
 sim=yaml.safe_load(result['simulation_path'].read_text())
 hold=yaml.safe_load(Path(sim['robot']['joint_position_hold']).read_text())['joint_position_hold']
 assert hold['trace_directory']==str(run/'physics')
 assert 'trace_directory' not in result['comparison_contract']['holding']['joint_position_hold']
 assert not (run/'physics').exists()


@pytest.mark.parametrize('value',['true',1,None])
def test_trace_option_requires_boolean(tmp_path,value):
 with pytest.raises(ValueError):
  load_experiment(document(tmp_path,execution={'record_actuator_trace':value}),SHARE)


def test_prediction_and_strategy_runtime_inputs_are_isolated(tmp_path):
    """Recorded numerical owners must also be the owners consumed at execution."""
    from robot_bringup.factory.system_configuration import load_system_configuration
    prediction = tmp_path / 'prediction.yaml'
    strategy = tmp_path / 'strategy.yaml'
    prediction.write_text(yaml.safe_dump(load_yaml(SRC/'robot_prediction/config/prediction.yaml')))
    from robot_experiments.launching import launch_profile
    parent_strategy = tmp_path / 'parent_strategy.yaml'
    parent_strategy.write_text(yaml.safe_dump(
        launch_profile('closed_loop', SHARE, controller='whole_body')['strategy']))
    strategy.write_text(yaml.safe_dump({'extends': str(parent_strategy)}))
    system = tmp_path / 'system.yaml'
    system.write_text(yaml.safe_dump({
        'extends': str(resolve_system_resource(SRC/'robot_experiments/config/systems/nominal_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)),
        'prediction': {'config': str(prediction)}, 'control': {'strategy': str(strategy)}}))
    spec = load_experiment(document(tmp_path, variants=[{
        'name': 'imm', 'system_config': str(system)}]), SHARE)
    run = tmp_path/'run'; run.mkdir()
    prepared = materialize_run(spec['variants'][0], 42, run, SHARE, spec['runtime'])
    import hashlib
    resources = prepared['snapshot']['resources']
    assert resources[str(strategy)]['parents'] == [str(parent_strategy)]
    for original in [strategy, parent_strategy]:
        assert resources[str(original)]['sha256'] == hashlib.sha256(original.read_bytes()).hexdigest()
    manifest = load_yaml(prepared['system_path'])
    for path in [manifest['prediction']['config'], manifest['control']['strategy']]:
        assert Path(path).is_relative_to(run/'inputs')
    prediction.write_text('invalid: changed\n')
    strategy.write_text('invalid: changed\n')
    replay = load_system_configuration(resolve_system_resource(prepared['system_path'], Path.cwd(), SHARE), SHARE)
    assert replay['identities'] == prepared['plan']['identities']


def test_prediction_configuration_access_cannot_leak_edits():
    """An editor starts with detached defaults on every access and session."""
    from robot_prediction.configuration.prediction_config import PredictionConfig
    config = PredictionConfig(SRC/'robot_prediction/config/prediction.yaml')
    from copy import deepcopy
    original = deepcopy(config.data)
    config.get('imm')['model_probabilities'][0] = 999.
    config.data['adaptations']['residual']['parameters']['correction_gain'] = 999.
    assert config.data == original


def test_materialization_emits_one_explicit_system_without_forwarding_profile(tmp_path):
    spec = load_experiment(document(tmp_path), SHARE)
    run = tmp_path / 'single-boundary'
    run.mkdir()
    prepared = materialize_run(spec['variants'][0], 42, run, SHARE, spec['runtime'])
    system = yaml.safe_load(prepared['system_path'].read_text())
    assert system['schema_version'] == 2
    assert not (run / 'inputs/profile.yaml').exists()
    assert prepared['plan']['composition']['profile_source_kind'] == 'system_manifest'


def test_new_prediction_family_parameters_survive_materialization(tmp_path, monkeypatch):
    import numpy as np
    from robot_configuration.components import schema_for
    from robot_prediction.baseline.constant_velocity import ConstantVelocityPredictor
    from robot_prediction.configuration.prediction_config import PredictionConfig
    from robot_prediction.factory import system_factory
    from robot_prediction.factory.component_registry import create_prediction_registries

    class CustomPredictor(ConstantVelocityPredictor):
        def __init__(self, offset):
            self.offset = offset

    def registries(config):
        result = create_prediction_registries(config)
        defaults = config.data['predictors']['custom']['parameters']
        result['predictor'].register('custom', CustomPredictor, defaults=defaults,
            schema=schema_for(defaults), capabilities={'deployable': False, 'test_only': True})
        return result

    monkeypatch.setattr(system_factory, 'create_prediction_registries', registries)
    configuration = PredictionConfig(SRC / 'robot_prediction/config/prediction.yaml').data
    configuration['predictors']['custom'] = {'parameters': {'offset': .2}}
    prediction = tmp_path / 'custom_prediction.yaml'
    prediction.write_text(yaml.safe_dump(configuration))
    system = tmp_path / 'custom_system.yaml'
    system.write_text(yaml.safe_dump({
        'extends': str(resolve_system_resource(SRC / 'robot_experiments/config/systems/nominal_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)),
        'prediction': {'type': 'custom', 'config': str(prediction)}}))
    spec = load_experiment(document(tmp_path, variants=[{
        'name': 'custom', 'system_config': str(system)}]), SHARE)
    spec['variants'][0]['owner_overrides'] = {
        'prediction': {'predictors': {'custom': {'parameters': {'offset': .7}}}}}
    run = tmp_path / 'run'; run.mkdir()
    prepared = materialize_run(spec['variants'][0], 42, run, SHARE, spec['runtime'])
    manifest = yaml.safe_load(prepared['system_path'].read_text())
    actual = PredictionConfig(manifest['prediction']['config']).data
    assert actual['predictors']['custom']['parameters']['offset'] == .7
    assert prepared['scientific_conditions']['prediction_configuration'] == actual


def test_partial_dynamics_are_frozen_with_effective_defaults(tmp_path):
    target = tmp_path / 'target.yaml'
    target.write_text(yaml.safe_dump({
        'extends': str(SRC / 'robot_simulation/config/target.yaml'),
        'target': {'dynamics': {'parameters': {'damping': None}}}}))
    # Use a complete independent source with a deliberately omitted known default.
    values = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml',
                        'group': 'resources.target_fight_flight'})
    from robot_simulation.factory.simulation_factory import SimulationFactory
    values['target']['dynamics']['parameters'] = SimulationFactory.with_defaults().dynamics_registry.validate(
        'fight_flight', values['target']['dynamics']['parameters'])
    values['target']['dynamics']['parameters'].pop('damping')
    target.write_text(yaml.safe_dump(values))
    scenario = tmp_path / 'scenario.yaml'
    scenario.write_text(yaml.safe_dump({
        'extends': str(resolve_system_resource(SRC / 'robot_simulation/config/scenarios/oscillatory_reactive_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)),
        'target': {'config': str(target)}}))
    system = tmp_path / 'system.yaml'
    system.write_text(yaml.safe_dump({
        'extends': str(resolve_system_resource(SRC / 'robot_experiments/config/systems/nominal_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)),
        'simulation': {'config': str(scenario)}}))
    spec = load_experiment(document(tmp_path, variants=[{
        'name': 'partial', 'system_config': str(system)}]), SHARE)
    run = tmp_path / 'run'; run.mkdir()
    prepared = materialize_run(spec['variants'][0], 42, run, SHARE, spec['runtime'])
    deployed = load_yaml(prepared['simulation_path'])
    frozen = load_yaml(deployed['target']['config'])['target']['dynamics']['parameters']
    assert frozen['damping'] == .5
    assert str((SRC / 'robot_simulation/config/dynamics.yaml').resolve()) in prepared['snapshot']['resources']
