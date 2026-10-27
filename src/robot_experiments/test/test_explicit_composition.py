"""Explicit M07 composition preserves the M08 execution and scientific contracts."""

import hashlib
import json
from pathlib import Path

import pytest

from robot_experiments.catalog import ExperimentCatalog


from robot_experiments import cli
from robot_experiments.configuration import load_experiment, materialize_run
from robot_experiments.preparation import complete_preparation
from robot_experiments.reporting import plain

import yaml
from functools import partial
from robot_experiments.composition import load_resource
load_yaml = partial(load_resource, share=lambda package: Path(
    __file__).resolve().parents[2] / package)


SRC = Path(__file__).resolve().parents[2]
OLD = SRC / 'robot_experiments/config/sensor_noise_study.yaml'
NEW = SRC / 'robot_experiments/config/sensor_noise_composition.yaml'


def share(package):
    """Use the actual source owner resources."""
    return SRC / package


def normalized(value, directory):
    """Normalize only proven ephemeral run-local prefixes, preserving array order."""
    return json.loads(json.dumps(plain(value), sort_keys=True).replace(str(directory), '<run>'))


@pytest.mark.parametrize('seed', [42, 917])
def test_old_new_materialized_owners_plans_launch_inputs_and_contracts_equal(tmp_path, seed):
    """Compare actual generated inputs and views, rather than the requested manifest."""
    before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in SRC.rglob('*.yaml')}
    old, new = [load_experiment(path, share) for path in [OLD, NEW]]
    for key in ['name', 'duration', 'settling_time', 'metrics', 'runtime', 'execution',
                'comparison']:
        assert old[key] == new[key], key
    for va, vb in zip(old['variants'], new['variants']):
        prepared = []
        directories = [tmp_path / (va['name'] + '_old'), tmp_path / (vb['name'] + '_new')]
        for spec, variant, directory in zip([old, new], [va, vb], directories):
            directory.mkdir()
            p = materialize_run(variant, seed, directory, share, spec['runtime'], True)
            complete_preparation(spec, p, 'same-source')
            prepared.append(p)
        a, b = prepared
        da, db = directories
        assert a['comparison'] == b['comparison']
        assert a['scientific_conditions'] == b['scientific_conditions']
        assert normalized(a['plan'], da) == normalized(b['plan'], db)
        assert normalized(a['context'], da) == normalized(b['context'], db)
        for path in (da / 'inputs').glob('*.yaml'):
            assert normalized(load_yaml(path), da) == normalized(
                load_yaml(db / 'inputs' / path.name), db), path.name
        assert b['snapshot']['seed_consumers'] == a['snapshot']['seed_consumers']
        assert b['snapshot']['assignments'] == a['snapshot']['assignments']
        captured = [Path(p) for p in b['snapshot']['resources']]
        bringup = [p for p in captured if p.is_relative_to(share('robot_bringup'))]
        assert not bringup
    assert all(hashlib.sha256(p.read_bytes()).hexdigest() == digest
               for p, digest in before.items())


def test_explain_exposes_direct_composition_and_effective_calibration_without_runtime(
        tmp_path, monkeypatch, capsys):
    """Add schema-specific provenance while preserving the existing S1 JSON contract."""
    def forbidden(*args, **kwargs):
        pytest.fail('explain acquired runtime')

    monkeypatch.setattr(cli, 'run_experiment', forbidden)
    output = tmp_path / 'results'
    assert cli.main(['--config', str(NEW), '--explain', '--output', str(output)], share=share) == 0
    captured = capsys.readouterr()
    doc = json.loads(captured.out)
    assert captured.err == ''
    assert doc['comparison']['configuration_compatibility']['compatible']
    assert doc['comparison']['runtime_eligibility'] is None
    for run, noise in zip(doc['runs'], [.04, .08]):
        composition = run['system']['composition']
        assert composition['schema_version'] == 2
        assert 'resources.deployment_base_relative_manipulation' in composition['simulation']
        assert composition['calibration'].endswith('calibration/manipulation_pose.yaml')
        assert run['estimation']['parameters']['wheel_linear_velocity_variance'] == .02
        assert run['experimental_variables']['pose_white_noise_std']['value'][0] == noise
        assert run['prediction']['implementations']['predictor'].endswith('.IMMPredictor')
        assert run['control']['implementations']['controller'].endswith(
            '.WeightedLeastSquaresController')
        assert run['scenario']['target_dynamics']['target']['dynamics']['type'] == (
            'saturated_fight_flight')
    assert not output.exists()


def test_invalid_later_variant_fails_before_any_runtime_for_new_composition(
        tmp_path, monkeypatch, capsys):
    """Preserve S2 all-variant prevalidation through the new system route."""
    data = ExperimentCatalog(share=share).document('EXP-SENSOR-NOISE-COMPOSITION')
    # Keep the inherited global package resource independent of this fixture location.
    bad = [-.1, .08, .06981317007977318]
    data['experiment']['comparison']['variables'][0]['values'].append(bad)
    data['experiment']['variants'][1]['parameters']['pose_white_noise_std'] = bad
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(data))

    def forbidden(*args, **kwargs):
        pytest.fail('invalid later variant launched baseline')

    monkeypatch.setattr(cli, 'run_experiment', forbidden)
    assert cli.main(['--config', str(path)], share=share) == 2
    captured = capsys.readouterr()
    assert json.loads(captured.out)['error']
