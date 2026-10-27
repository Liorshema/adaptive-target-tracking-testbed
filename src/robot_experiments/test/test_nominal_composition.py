"""Schema-2 nominal family retains generated inputs and strict scientific semantics."""

import json
from pathlib import Path

import pytest

from robot_experiments.catalog import ExperimentCatalog


from robot_experiments import cli
from robot_experiments.comparison import comparison_reasons
from robot_experiments.configuration import load_experiment, materialize_run
from robot_experiments.preparation import complete_preparation
from robot_experiments.reporting import plain

import yaml
from functools import partial
from robot_experiments.composition import load_resource
from robot_experiments.configuration import resolve_system_resource
load_yaml = partial(load_resource, share=lambda package: Path(
    __file__).resolve().parents[2] / package)


SRC = Path(__file__).resolve().parents[2]
CONFIG = SRC / 'robot_experiments/config'
NEW = CONFIG / 'paired_prediction_composition.yaml'


def share(package):
    """Resolve real source owners."""
    return SRC / package


def normalize(value, directory):
    """Relabel only ephemeral generated run directories, never physical fields."""
    return json.loads(json.dumps(plain(value), sort_keys=True).replace(str(directory), '<run>'))


@pytest.mark.parametrize('seed', [42, 917])
@pytest.mark.parametrize('family', ['paired', 'nominal', 'dls'])
def test_seeded_owner_inputs_contracts_and_runtime_are_equivalent(tmp_path, seed, family):
    """Compare corresponding variants including generated launch plans and scientific views."""
    if family == 'paired':
        paths = [CONFIG / 'paired_prediction.yaml', NEW]
    else:
        paths = []
        for label, name in [('old', f'config/system_{family}.yaml'), ('new',
                            resolve_system_resource(CONFIG / 'systems' / ('nominal_tracking.yaml' if family == 'nominal'
                                                                          else 'dls_control.yaml'), CONFIG, lambda package: Path(__file__).resolve().parents[2] / package))]:
            data = ExperimentCatalog(share=share).document('EXP-NOMINAL')
            data['experiment']['variants'] = [{'name': family, 'system_config': ({'package': 'robot_bringup', 'path': name}
                                                                                 if label == 'old' else str(name))}]
            path = tmp_path / f'{label}.yaml'
            path.write_text(yaml.safe_dump(data))
            paths.append(path)
    a, b = [load_experiment(path, share) for path in paths]
    for key in ['name', 'duration', 'settling_time', 'seeds', 'metrics', 'runtime',
                'execution', 'comparison']:
        assert a.get(key) == b.get(key), key
    checks = [[], []]
    for va, vb in zip(a['variants'], b['variants']):
        runs, directories = [], []
        for label, spec, variant in [('old', a, va), ('new', b, vb)]:
            directory = tmp_path / (label + variant['name'])
            directory.mkdir()
            p = materialize_run(variant, seed, directory, share, spec['runtime'],
                                spec['execution']['record_actuator_trace'])
            complete_preparation(spec, p, 'same-current-source')
            runs.append(p)
            directories.append(directory)
        x, y = runs
        dx, dy = directories
        assert x['comparison'] == y['comparison']
        assert x['scientific_conditions'] == y['scientific_conditions']
        assert normalize(x['plan'], dx) == normalize(y['plan'], dy)
        assert normalize(x['context'], dx) == normalize(y['context'], dy)
        assert x['snapshot']['seed_consumers'] == y['snapshot']['seed_consumers']
        assert x['snapshot'].get('assignments') == y['snapshot'].get('assignments')
        inputs = {p.name: normalize(load_yaml(p), dx) for p in (dx / 'inputs').glob('*.yaml')}
        assert inputs == {p.name: normalize(load_yaml(p), dy)
                          for p in (dy / 'inputs').glob('*.yaml')}
        for group, run in zip(checks, runs):
            group.append({'variant': va['name'], 'seed': seed, 'execution_status': 'completed',
                          'comparison_contract': run['comparison']['original_hash'],
                          'comparison': run['comparison']})
    assert a.get('comparison') is None and b.get('comparison') is None
    variants = [v['name'] for v in a['variants']]
    decisions = [comparison_reasons(r, variants, [seed]) for r in checks]
    assert decisions == [[], []]
    for group in checks:
        group[0]['execution_status'] = 'interrupted'
    assert comparison_reasons(checks[0], variants, [seed]) == comparison_reasons(
        checks[1], variants, [seed]) != []
    for group in checks:
        group[0]['execution_status'] = 'completed'
        group[0]['comparison_contract'] = 'changed-physical-scenario'
    if len(variants) > 1:
        assert comparison_reasons(checks[0], variants, [seed]) == comparison_reasons(
            checks[1], variants, [seed]) != []


def test_explain_reports_real_cv_and_nominal_classes_and_does_not_run(
        tmp_path, monkeypatch, capsys):
    """S1 exposes selection and direct deployment without claiming runtime eligibility."""
    def forbidden(*args, **kwargs):
        pytest.fail('explain launched runtime')

    monkeypatch.setattr(cli, 'run_experiment', forbidden)
    output = tmp_path / 'results'
    assert cli.main(['--config', str(NEW), '--explain', '--output', str(output)], share=share) == 0
    doc = json.loads(capsys.readouterr().out)
    assert not output.exists()
    assert doc['experiment']['name'] == 'paired_prediction'
    assert doc['comparison']['runtime_eligibility'] is None
    assert doc['comparison']['configuration_compatibility']['compatible']
    for run, predictor in zip(doc['runs'], ['.IMMPredictor', '.ConstantVelocityPredictor']):
        assert run['prediction']['implementations']['predictor'].endswith(predictor)
        assert run['control']['implementations']['controller'].endswith(
            '.WeightedLeastSquaresController')
        assert run['system']['composition']['schema_version'] == 2
        assert 'resources.deployment_oscillatory_reactive_tracking' in run['system']['composition']['simulation']


def test_dls_fixture_explain_retains_actual_cartesian_controller(tmp_path, capsys):
    """A test-only study exercises DLS without adding a duplicate production study."""
    data = ExperimentCatalog(share=share).document('EXP-NOMINAL')
    data['experiment']['variants'] = [{'name': 'dls', 'system_config': {
        'package': 'robot_bringup', 'path': 'config/systems/dls_control.yaml'}}]
    path = tmp_path / 'dls.yaml'
    path.write_text(yaml.safe_dump(data))
    assert cli.main(['--config', str(path), '--explain'], share=share) == 0
    run = json.loads(capsys.readouterr().out)['runs'][0]
    assert run['control']['implementations']['controller'].endswith('.JacobianController')
    assert run['prediction']['implementations']['predictor'].endswith('.IMMPredictor')


def test_physical_initial_condition_difference_is_still_rejected(tmp_path):
    """The strict historical contract must retain real scenario fields after migration."""
    system = load_yaml(
        resolve_system_resource(
            share('robot_experiments') /
            'config/systems/nominal_tracking.yaml',
            Path.cwd(),
            lambda package: Path(__file__).resolve().parents[2] /
            package))
    deployment = load_yaml(system['simulation']['config'])
    deployment['target']['initial_position'][0] += .1
    path = tmp_path / 'deployment.yaml'
    path.write_text(yaml.safe_dump(deployment))
    system['simulation']['config'] = str(path)
    path = tmp_path / 'system.yaml'
    path.write_text(yaml.safe_dump(system))
    data = ExperimentCatalog(share=share).document('EXP-PREDICTION-COMPOSITION')
    data['experiment']['variants'][1]['system_config'] = str(path)
    path = tmp_path / 'study.yaml'
    path.write_text(yaml.safe_dump(data))
    spec = load_experiment(path, share)
    checks = []
    for variant in spec['variants']:
        directory = tmp_path / variant['name']
        directory.mkdir()
        p = materialize_run(variant, 42, directory, share, spec['runtime'])
        complete_preparation(spec, p, 'same-source')
        checks.append({'variant': variant['name'], 'seed': 42, 'execution_status': 'completed',
                       'comparison_contract': p['comparison']['original_hash'],
                       'comparison': p['comparison']})
    reasons = comparison_reasons(checks, ['imm', 'cv'], [42], configuration_only=True)
    assert any('initial_position' in reason for reason in reasons)
