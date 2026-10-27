"""Explain uses actual seeded inputs and stops before any runtime ownership."""

import hashlib
import json
from pathlib import Path

import pytest

from robot_experiments.catalog import ExperimentCatalog

from robot_experiments import cli, recording, runner
from robot_experiments.configuration import load_experiment

import yaml
from functools import partial
from robot_experiments.composition import load_resource
from robot_experiments.configuration import resolve_system_resource
load_yaml = partial(load_resource, share=lambda package: Path(
    __file__).resolve().parents[2] / package)


SRC = Path(__file__).resolve().parents[2]


def SHARE(name):
    """Resolve actual repository owner resources."""
    return SRC / name


def invoke(name, capsys, *extra):
    """Parse only a successful, complete CLI document."""
    code = cli.main(['--config', str(SRC / 'robot_experiments/config' / name),
                     '--explain', *extra], share=SHARE)
    captured = capsys.readouterr()
    assert code == 0, captured.err
    return json.loads(captured.out)


@pytest.fixture(autouse=True)
def no_runtime(monkeypatch):
    """Any accidental use of the real lifecycle/recorder is a test failure."""
    def forbidden(*args, **kwargs):
        pytest.fail('explain acquired runtime resources')
    for module, name in [(runner, 'DomainLease'), (runner, 'OwnedLaunch'),
                         (runner, 'run_window'), (recording, 'Recorder'),
                         (cli, 'run_experiment')]:
        monkeypatch.setattr(module, name, forbidden)
    monkeypatch.setattr(runner, '_git', lambda: {'worktree_source_sha256': 'test-source'})


@pytest.mark.parametrize('name,variants,seeds', [
    ('nominal.yaml', ['imm'], [42]),
    ('paired_prediction.yaml', ['imm', 'cv'], [42]),
    ('reactive_manipulation.yaml', ['A', 'B', 'C', 'D', 'E'], [42, 917]),
    ('sensor_noise_comparison.yaml', ['baseline', 'higher_noise'], [42]),
])
def test_all_entries_expose_effective_components_without_running(name, variants, seeds, capsys):
    """Losing a variant/seed or substituting filename guesses breaks this contract."""
    doc = invoke(name, capsys)
    assert doc['schema_version'] == 1
    assert doc['mode'] == 'explain'
    assert [(r['variant'], r['seed']) for r in doc['runs']] == [
        (v, s) for s in seeds for v in variants]
    assert doc['comparison']['runtime_eligibility'] is None
    for run in doc['runs']:
        assert run['configuration_validation']['passed'] is True
        assert run['system']['identities']['robot_estimator'].endswith('.RobotStateEstimator')
        assert run['system']['identities']['target_estimator'].endswith('.TargetStateEstimator')
        assert run['control']['runtime']['enable_actuation'] is True
        assert run['control']['runtime']['publish_commands'] is True
        assert run['runtime']['window']['start'] is None
        assert run['runtime']['window']['end'] is None
        assert run['runtime']['window']['basis'] == 'readiness-relative simulation source epoch'
        assert run['scenario']['target_dynamics']['target']['dynamics']['type'] == (
            'saturated_fight_flight')
        assert run['seed_consumers']['measurements.target_position.random_seed'] == run['seed']
        assert 'metrics' not in run
    if name == 'paired_prediction.yaml':
        assert doc['runs'][0]['system']['selection']['predictor_type'] == 'imm'
        assert doc['runs'][1]['system']['selection']['predictor_type'] == 'constant_velocity'
        assert doc['runs'][1]['system']['selection']['adaptation_type'] == 'none'
        assert doc['runs'][1]['system']['identities']['predictor'].endswith(
            '.ConstantVelocityPredictor')
    if name == 'reactive_manipulation.yaml':
        d = doc['runs'][3]
        assert d['robot']['description']['robot']['manipulator']['joints']['wrist']['upper'] == (
            0.2617993877991494)
        field = d['scenario']['environment']['environment']['acceleration_fields'][0]
        assert field['type'] == 'base_relative_guidance'
        assert d['scenario']['tracking']['pose_source'] == 'physical'
        assert doc['comparison']['configuration_compatibility']['compatible'] is False
        assert any('description' in r for r in
                   doc['comparison']['configuration_compatibility']['rejection_reasons'])


def test_noise_study_shows_actual_values_sources_and_both_hashes(capsys):
    """Wrong sensor values/seeds or claiming runtime eligibility is a scientific defect."""
    doc = invoke('sensor_noise_comparison.yaml', capsys)
    assert doc['comparison']['configuration_compatibility']['compatible'] is True
    originals, invariants = set(), set()
    for run, expected in zip(doc['runs'], [
            [.04, .04, .03490658503988659], [.08, .08, .06981317007977318]]):
        variable = run['experimental_variables']['pose_white_noise_std']
        assert variable['value'] == expected
        assert variable['path'] == 'localization_sensor.localization_sensor.white_noise_std'
        assert expected in variable['declared_values']
        assert variable['owner_resources']
        assert all(p in run['sources']['resources'] for p in variable['owner_resources'])
        sensor = run['sensors']['sources']['external_planar_localization']
        assert sensor['simulated_effects']['white_noise_std'] == expected
        assert sensor['simulated_effects']['random_seed'] == 3872720740
        assert run['seed_consumers']['localization_sensor.random_seed'] == 3872720740
        view = run['comparison']
        assert view['original_contract']['duration'] == 8.
        assert view['original_contract']['settling_time'] == 3.
        assert view['original_contract']['max_skew'] == .15
        assert hashlib.sha256(json.dumps(view['original_contract'], sort_keys=True).encode()
                              ).hexdigest() == view['original_hash']
        originals.add(view['original_hash'])
        invariants.add(view['invariant_hash'])
        assert run['comparison']['runtime_eligibility'] is None
    assert len(originals) == 2 and len(invariants) == 1


def test_explain_is_stable_and_cleans_materialization_without_creating_results(
        tmp_path, monkeypatch, capsys):
    """Temp names must not leak into display or create a normal study directory."""
    import tempfile
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    destination = tmp_path / 'results'
    before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in SRC.rglob('*.yaml')}
    a = invoke('sensor_noise_comparison.yaml', capsys, '--output', str(destination))
    b = invoke('sensor_noise_comparison.yaml', capsys, '--output', str(destination))
    assert a == b
    assert not list(tmp_path.iterdir())
    assert str(tmp_path) not in json.dumps(a['runs'])
    assert {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in before} == before
    resources = a['runs'][1]['sources']['resources']
    assert str(SRC / 'robot_simulation/config/sensors/planar_localization.yaml') in resources
    assert str(SRC / 'robot_experiments/config/experiments.yaml') in resources
    assert any('resources.localization_noise' in key for key in resources)
    assert a['experiment']['source_ancestry'] == [
        str(SRC / 'robot_experiments/config/experiments.yaml')]
    assert a['experiment']['experiment_identity']['id'] == 'EXP-SENSOR-NOISE-LEGACY'


def write_experiment(tmp_path, updates):
    """Use real baseline inheritance, without editing the owner YAML."""
    p = tmp_path / 'experiment.yaml'
    data = ExperimentCatalog(share=SHARE).document('EXP-SENSOR-NOISE-LEGACY')
    # These fixtures intentionally introduce malformed declarations to exercise preflight.
    data['experiment'].update(updates)
    p.write_text(yaml.safe_dump(data))
    return p


@pytest.mark.parametrize('variable,reason', [
    ({'name': 'sensor', 'path': 'measurements.measurements.target_position.noise.covariance'},
     None),
    ({'name': 'sensor', 'path': 'localization_sensor.localization_sensor.white_noise_std',
      'values': [[99., 99., 99.]]}, 'effective value not permitted'),
    ({'name': 'sensor', 'path': 'description.robot.manipulator.joints.shoulder.lower'},
     'not a permitted sensor leaf'),
])
def test_declarations_use_existing_authorization_and_value_validation(
        tmp_path, capsys, variable, reason):
    """A declared value is inspected, never assigned or used to bypass policy."""
    p = write_experiment(tmp_path, {'comparison': {
        'schema_version': 1, 'domain': 'sensor_parameters', 'variables': [variable]}})
    code = cli.main(['--config', str(p), '--explain'], share=SHARE)
    captured = capsys.readouterr()
    if reason is None:
        assert code == 0
        assert json.loads(captured.out)['comparison']['configuration_compatibility'][
            'compatible'] is False
    else:
        assert code == 2 and captured.out == ''
        assert reason in json.loads(captured.err)['error']


def test_missing_materialized_leaf_rejects_without_partial_output_and_cleans(
        tmp_path, monkeypatch, capsys):
    """Valid declarations on an absent source fail before any lifecycle call."""
    import tempfile
    scratch = tmp_path / 'scratch'
    scratch.mkdir()
    monkeypatch.setattr(tempfile, 'tempdir', str(scratch))
    p = write_experiment(tmp_path, {'variants': [{
        'name': 'no_source', 'system_config': {
            'package': 'robot_bringup', 'path': 'config/system_nominal.yaml'}}]})
    assert cli.main(['--config', str(p), '--explain'], share=SHARE) == 2
    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'absent from materialized contract' in json.loads(captured.err)['error']
    assert not list(scratch.iterdir())


@pytest.mark.parametrize('failure', ['cycle', 'resource', 'owner'])
def test_bad_inputs_preserve_json_error_boundary(tmp_path, capsys, failure):
    """Inherited, package and owner defects must not be reported as successful explanations."""
    p = tmp_path / 'bad.yaml'
    if failure == 'cycle':
        p.write_text('extends: bad.yaml\n')
    elif failure == 'resource':
        p = write_experiment(tmp_path, {'variants': [{'name': 'bad', 'system_config': {
            'package': 'robot_bringup', 'path': 'config/absent.yaml'}}]})
    else:
        p = write_experiment(tmp_path, {'variants': [{'name': 'bad', 'system_config': {
            'package': 'robot_bringup', 'path': 'config/system_invalid.yaml'}}]})
    assert cli.main(['--config', str(p), '--explain'], share=SHARE) == 2
    captured = capsys.readouterr()
    assert captured.out == '' and json.loads(captured.err)['error']


def test_incompatible_modes_and_unknown_flags_have_no_success_output(capsys):
    """Argparse must reject ambiguous modes before any preparation."""
    for flags in [['--explain', '--validate-only'], ['--explain', '--unknown']]:
        with pytest.raises(SystemExit) as error:
            cli.main(['--config', 'unused.yaml', *flags], share=SHARE)
        assert error.value.code == 2
        assert capsys.readouterr().out == ''


def test_validate_only_and_normal_execution_retain_their_contracts(
        tmp_path, monkeypatch, capsys):
    """Adding explain must not redefine validation or change normal run delegation."""
    p = str(SRC / 'robot_experiments/config/nominal.yaml')
    assert cli.main(['--config', p, '--validate-only'], share=SHARE) == 0
    doc = json.loads(capsys.readouterr().out)
    assert set(doc) == {'schema_version', 'valid', 'variants'}
    seen = []

    def normal(spec, share):
        seen.append(spec)
        return tmp_path / 'study', 3
    monkeypatch.setattr(cli, 'run_experiment', normal)
    assert cli.main(['--config', p, '--output', str(tmp_path)], share=SHARE) == 3
    assert json.loads(capsys.readouterr().out)['exit_code'] == 3
    assert seen[0]['output']['directory'] == tmp_path


def test_normal_run_preparation_equals_explain_before_runtime(
        tmp_path, monkeypatch, capsys):
    """Compare actual normal-path saved evidence to explain, with only display paths normalized."""
    from robot_experiments.execution import RunFailure

    def stop(*args, **kwargs):
        raise RunFailure('test_before_runtime')
    monkeypatch.setattr(runner, 'DomainLease', stop)
    spec = load_experiment(SRC / 'robot_experiments/config/sensor_noise_comparison.yaml', SHARE)
    runner.run_one(spec, spec['variants'][1], 42, tmp_path / 'actual', SHARE)
    saved = json.loads((tmp_path / 'actual/resolved_configuration.json').read_text())
    doc = invoke('sensor_noise_comparison.yaml', capsys)
    explained = doc['runs'][1]
    for key in ['original_contract', 'invariant_contract', 'protected_conditions',
                'original_hash', 'invariant_hash', 'variables']:
        assert explained['comparison'][key] == saved['comparison'][key]
    assert explained['system']['identities'] == saved['system']['identities']
    assert explained['seed_consumers'] == saved['owner_snapshot']['seed_consumers']
    normal = saved['system']['sensor_inventory']

    def display(value):
        if isinstance(value, str):
            return value.replace(str(tmp_path / 'actual'), '<materialized>')
        if isinstance(value, dict):
            return {k: display(v) for k, v in value.items()}
        if isinstance(value, list):
            return [display(v) for v in value]
        return value
    assert explained['sensors'] == display(normal)


def test_late_preparation_failure_cleans_all_runs_and_prints_no_partial_success(
        tmp_path, monkeypatch, capsys):
    """The first successful variant must not leak when the second is rejected."""
    import tempfile
    scratch = tmp_path / 'scratch'
    scratch.mkdir()
    monkeypatch.setattr(tempfile, 'tempdir', str(scratch))
    p = write_experiment(tmp_path, {'comparison': {
        'schema_version': 1, 'domain': 'sensor_parameters', 'variables': [{
            'name': 'pose_white_noise_std',
            'path': 'localization_sensor.localization_sensor.white_noise_std',
            'values': [[.04, .04, .03490658503988659]]}]}})
    assert cli.main(['--config', str(p), '--explain'], share=SHARE) == 2
    captured = capsys.readouterr()
    assert captured.out == '' and 'effective value not permitted' in captured.err
    assert not list(scratch.iterdir())


def test_existing_results_and_dependency_diagnostics_are_preserved(
        tmp_path, monkeypatch, capsys):
    """Explanation prints one JSON document and cannot touch an existing result tree."""
    from robot_experiments import explain
    actual = explain.complete_preparation

    def noisy(*args, **kwargs):
        print('preparation diagnostic')
        return actual(*args, **kwargs)

    monkeypatch.setattr(explain, 'complete_preparation', noisy)
    sentinel = tmp_path / 'previous.json'
    sentinel.write_text('{"previous": true}\n')
    code = cli.main(['--config', 'config/nominal.yaml', '--explain',
                     '--output', str(tmp_path)], share=SHARE)
    captured = capsys.readouterr()
    assert code == 0 and json.loads(captured.out)['mode'] == 'explain'
    assert 'preparation diagnostic' in captured.err
    assert sentinel.read_text() == '{"previous": true}\n'
    assert list(tmp_path.iterdir()) == [sentinel]


def test_protected_scenario_delta_is_explained_by_the_existing_sensor_policy(
        tmp_path, capsys):
    """An authorized noise difference must not conceal a second physical/scenario change."""
    scenario = tmp_path / 'scenario.yaml'
    scenario.write_text(yaml.safe_dump({
        'extends': str(resolve_system_resource(SRC / 'robot_simulation/config/scenarios/'
                       'reactive_manipulation_noise_study.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)),
        'target': {'initial_velocity': [.01, 0., 0.]}}))
    selected = load_yaml(
        resolve_system_resource(
            SRC /
            'robot_experiments/config/systems/localization_noise_study.yaml',
            Path.cwd(),
            lambda package: Path(__file__).resolve().parents[2] /
            package))
    selected['simulation']['config'] = str(scenario)
    system = tmp_path / 'system.yaml'
    system.write_text(yaml.safe_dump(selected))
    p = write_experiment(tmp_path, {'variants': [
        {'name': 'baseline', 'system_config': str(
            resolve_system_resource(SRC / 'robot_experiments/config/systems/mobile_manipulator_pose_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package))},
        {'name': 'higher_noise', 'system_config': str(system)}]})
    assert cli.main(['--config', str(p), '--explain'], share=SHARE) == 0
    doc = json.loads(capsys.readouterr().out)
    decision = doc['comparison']['configuration_compatibility']
    assert decision['compatible'] is False
    assert any('target_initial_velocity' in r for r in decision['rejection_reasons'])
    assert doc['comparison']['runtime_eligibility'] is None
