"""Named sensor assignments preserve permissions, owner validation and scientific inputs."""

import hashlib
import json
from copy import deepcopy
from pathlib import Path

import pytest

from robot_configuration.yaml_loader import load_yaml

from robot_experiments import cli, recording, runner
from robot_experiments.configuration import load_experiment, materialize_run
from robot_experiments.preparation import complete_preparation

import yaml


SRC = Path(__file__).resolve().parents[2]
NOISE = 'localization_sensor.localization_sensor.white_noise_std'
BASE = [.04, .04, .03490658503988659]
HIGH = [.08, .08, .06981317007977318]
SYSTEM = {'package': 'robot_bringup', 'path': 'config/system_manipulation_D_localized.yaml'}


def share(package):
    """Use real owner resources without ROS package lookup."""
    return SRC / package


def document(path=NOISE, values=None, parameters=None):
    """Use the old study's actual effective settings with independently selected schema2 inputs."""
    old = load_yaml(SRC / 'robot_experiments/config/sensor_noise_comparison.yaml')
    old['schema_version'] = 2
    ex = old['experiment']
    ex['system_config'] = deepcopy(SYSTEM)
    allowed = [BASE, HIGH] if values is None else values
    chosen = [BASE, HIGH] if parameters is None else parameters
    ex['variants'] = [{'name': name, 'parameters': {'sensor_effect': deepcopy(value)}}
                      for name, value in zip(['baseline', 'higher_noise'], chosen)]
    ex['comparison']['variables'] = [{'name': 'sensor_effect', 'path': path, 'values': allowed}]
    return old


def write(tmp_path, data, name='experiment.yaml'):
    """Write fixtures only; production owner YAML is never changed."""
    p = tmp_path / name
    p.write_text(yaml.safe_dump(data))
    return p


def prepare(spec, variant, tmp_path, seed=42):
    """Exercise the shared actual materialization/preparation boundary."""
    tmp_path.mkdir()
    actual = materialize_run(variant, seed, tmp_path, share, spec['runtime'],
                             spec['execution']['record_actuator_trace'])
    complete_preparation(spec, actual, 'same-source')
    return actual


def test_new_installed_entry_matches_old_contracts_and_all_materialized_owners(tmp_path):
    """Assignments must replace only the forwarding mechanism, not any effective behavior."""
    old = load_experiment(SRC / 'robot_experiments/config/sensor_noise_comparison.yaml', share)
    new = load_experiment(SRC / 'robot_experiments/config/sensor_noise_study.yaml', share)
    assert new['system_config'] == (
        SRC / 'robot_bringup/config/system_manipulation_D_localized.yaml')
    for key in ['name', 'seeds', 'metrics', 'duration', 'settling_time', 'runtime', 'execution']:
        assert new[key] == old[key]
    before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in SRC.rglob('*.yaml')}
    for a, b in zip(old['variants'], new['variants']):
        x = prepare(old, a, tmp_path / (a['name'] + '_old'))
        y = prepare(new, b, tmp_path / (b['name'] + '_new'))
        assert x['comparison'] == y['comparison']
        assert x['plan']['identities'] == y['plan']['identities']
        assert x['scientific_conditions'] == y['scientific_conditions']
        assert x['snapshot']['seed_consumers'] == y['snapshot']['seed_consumers']

        def normalize(value, root):
            return json.dumps(value, sort_keys=True, default=str).replace(str(root), '<run>')
        assert normalize(x['plan']['sensor_inventory'], tmp_path / (a['name'] + '_old')) == (
            normalize(y['plan']['sensor_inventory'], tmp_path / (b['name'] + '_new')))
        left = tmp_path / (a['name'] + '_old') / 'inputs'
        right = tmp_path / (b['name'] + '_new') / 'inputs'
        assert sorted(p.name for p in left.glob('*.yaml')) == sorted(
            p.name for p in right.glob('*.yaml'))
        for path in left.glob('*.yaml'):
            assert normalize(load_yaml(path), left.parent) == normalize(
                load_yaml(right / path.name), right.parent)
        assert not any('noise_study' in p for p in y['snapshot']['resources'])
    assert before == {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in before}


def test_noise_explain_has_assignment_origin_actual_values_and_no_runtime(
        tmp_path, capsys, monkeypatch):
    """Requested values cannot replace actual materialized comparison evidence."""
    def forbidden(*args, **kwargs):
        pytest.fail('explain acquired runtime')

    for module, name in [(cli, 'run_experiment'), (runner, 'DomainLease'),
                         (runner, 'OwnedLaunch'), (recording, 'Recorder')]:
        monkeypatch.setattr(module, name, forbidden)
    p = SRC / 'robot_experiments/config/sensor_noise_study.yaml'
    assert cli.main(['--config', str(p), '--explain', '--output', str(tmp_path / 'results')],
                    share=share) == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc['comparison']['configuration_compatibility']['compatible'] is True
    assert doc['comparison']['runtime_eligibility'] is None
    assert len(doc['runs']) == 2
    for run, expected in zip(doc['runs'], [BASE, HIGH]):
        assert run['system']['source'].endswith('system_manipulation_D_localized.yaml')
        v = run['experimental_variables']['pose_white_noise_std']
        assert v['value'] == expected
        assert v['assignment']['requested_value'] == expected
        assert v['assignment']['baseline_value'] == BASE
        assert v['assignment']['owner_source'].endswith('sensors/planar_localization.yaml')
        assert v['assignment']['source']['document'] == str(p)
        assert v['owner_resources'] == [v['assignment']['owner_source']]
        assert run['seed_consumers']['localization_sensor.random_seed'] == 3872720740
    assert not (tmp_path / 'results').exists()


@pytest.mark.parametrize('mutation,match', [
    ('missing', 'assignment'), ('unknown', 'assignment'), ('outside', 'permitted'),
    ('duplicate_name', 'duplicate'), ('duplicate_path', 'duplicate'),
    ('unsupported', 'permitted sensor leaf'), ('no_values', 'values'),
    ('no_policy', 'comparison'), ('mixed', 'system_config'), ('global_in_legacy', 'unknown'),
    ('parameters_in_legacy', 'unknown'), ('boolean_version', 'schema_version'),
])
def test_schema_rejects_ambiguous_missing_and_unauthorized_bindings(tmp_path, mutation, match):
    """Schema2 is explicit and cannot broaden legacy scientific permissions."""
    d = document()
    ex, variants = d['experiment'], d['experiment']['variants']
    variables = ex['comparison']['variables']
    if mutation == 'missing':
        variants[0]['parameters'] = {}
    elif mutation == 'unknown':
        variants[0]['parameters']['unknown'] = BASE
    elif mutation == 'outside':
        variants[0]['parameters']['sensor_effect'] = [1., 1., 1.]
    elif mutation == 'duplicate_name':
        variables.append(deepcopy(variables[0]))
    elif mutation == 'duplicate_path':
        variables.append({**variables[0], 'name': 'second'})
    elif mutation == 'unsupported':
        variables[0]['path'] = 'description.robot.manipulator.joints.shoulder.lower'
    elif mutation == 'no_values':
        variables[0].pop('values')
    elif mutation == 'no_policy':
        ex.pop('comparison')
    elif mutation == 'mixed':
        variants[0]['system_config'] = SYSTEM
    elif mutation == 'global_in_legacy':
        d['schema_version'] = 1
    elif mutation == 'parameters_in_legacy':
        d['schema_version'] = 1
        ex.pop('system_config')
    elif mutation == 'boolean_version':
        d['schema_version'] = True
    with pytest.raises(ValueError, match=match):
        load_experiment(write(tmp_path, d), share)


@pytest.mark.parametrize('path,bad', [
    (NOISE, [.1, .2]), (NOISE, [0., .1, .1]), (NOISE, ['0.1', .1, .1]),
    (NOISE, [True, .1, .1]), (NOISE, [float('inf'), .1, .1]),
    ('localization_sensor.localization_sensor.dropout_probability', -0.01),
    ('localization_sensor.localization_sensor.dropout_probability', 1.01),
    ('localization_sensor.localization_sensor.dropout_probability', True),
    ('localization_sensor.localization_sensor.latency', .2),
    ('localization_sensor.localization_sensor.outlier_offset', [.1, .2]),
    ('measurements.measurements.target_position.noise.covariance', [[.1, 0.], [0., .1]]),
    ('measurements.measurements.target_position.noise.covariance',
     [[.1, .2, 0.], [0., .1, 0.], [0., 0., .1]]),
    ('measurements.measurements.target_position.noise.covariance',
     [[-.1, 0., 0.], [0., .1, 0.], [0., 0., .1]]),
    ('measurements.measurements.target_position.noise.covariance',
     [[True, 0., 0.], [0., .1, 0.], [0., 0., .1]]),
])
def test_invalid_assigned_values_fail_before_runtime(tmp_path, path, bad):
    """Permission to vary never bypasses the producer's shape/type/physics validator."""
    with pytest.raises(ValueError):
        spec = load_experiment(write(tmp_path, document(path, [bad], [bad])), share)
        prepare(spec, spec['variants'][0], tmp_path / 'run')


@pytest.mark.parametrize('path,values', [
    ('localization_sensor.localization_sensor.dropout_probability', [0., .8]),
    ('localization_sensor.localization_sensor.calibration_bias_std',
     [[0., 0., 0.], [.01, .02, .03]]),
    ('localization_sensor.localization_sensor.latency', [0., .04]),
    ('localization_sensor.localization_sensor.latency_jitter', [0., .02]),
    ('localization_sensor.localization_sensor.outlier_probability', [0., .2]),
    ('localization_sensor.localization_sensor.outlier_offset', [[1., 1., .5], [-.5, .2, .1]]),
    ('measurements.measurements.target_position.noise.covariance', [
        [[.01, 0., 0.], [0., .01, 0.], [0., 0., .01]],
        [[.04, 0., 0.], [0., .04, 0.], [0., 0., .04]]]),
])
def test_second_sensor_effects_use_their_own_owner_and_preserve_estimator_assumptions(
        tmp_path, path, values):
    """Dropout and target covariance are actual producer changes, without retuning estimator R."""
    spec = load_experiment(write(tmp_path, document(path, values, values)), share)
    a = prepare(spec, spec['variants'][0], tmp_path / 'a', 917)
    b = prepare(spec, spec['variants'][1], tmp_path / 'b', 917)
    assert a['comparison']['invariant_hash'] == b['comparison']['invariant_hash']
    assert a['plan']['estimation_parameters'] == {
        **b['plan']['estimation_parameters'],
        'robot_description_file': a['plan']['estimation_parameters']['robot_description_file']}
    assert a['snapshot']['seed_consumers'] == b['snapshot']['seed_consumers']
    assert a['comparison']['variables']['sensor_effect']['value'] == values[0]
    assert b['comparison']['variables']['sensor_effect']['value'] == values[1]


def test_missing_producer_is_not_created_by_an_assignment(tmp_path):
    """An authorized localization leaf requires a selected, enabled localization producer."""
    d = document()
    d['experiment']['system_config'] = {
        'package': 'robot_bringup', 'path': 'config/system_nominal.yaml'}
    with pytest.raises(ValueError, match='producer|source|localization'):
        spec = load_experiment(write(tmp_path, d), share)
        prepare(spec, spec['variants'][0], tmp_path / 'run')


def test_duplicate_parameter_keys_are_rejected_without_changing_legacy_yaml_loader(tmp_path):
    """Duplicated assignments must not be hidden by last-key-wins loading."""
    p = write(tmp_path, document())
    text = p.read_text().replace(
        'parameters:\n', 'parameters:\n      sensor_effect: [9, 9, 9]\n', 1)
    p.write_text(text)
    with pytest.raises(ValueError, match='duplicate assignment'):
        load_experiment(p, share)


def test_global_reference_uses_the_declaring_document_and_invalid_inheritance_fails(tmp_path):
    """Global system selection has the same relative-resource semantics as existing variants."""
    parent = tmp_path / 'parent'
    parent.mkdir()
    d = document()
    d['experiment']['system_config'] = 'system.yaml'
    (parent / 'system.yaml').write_text(yaml.safe_dump({
        'extends': str(SRC / 'robot_bringup/config/system_manipulation_D_localized.yaml')}))
    write(parent, d, 'base.yaml')
    child = write(tmp_path, {'extends': 'parent/base.yaml'}, 'child.yaml')
    spec = load_experiment(child, share)
    assert spec['system_config'] == parent / 'system.yaml'
    broken = write(tmp_path, {'extends': 'broken.yaml'}, 'broken.yaml')
    with pytest.raises(ValueError, match='cycle'):
        load_experiment(broken, share)


@pytest.mark.parametrize('mode', [[], ['--explain'], ['--validate-only']])
def test_schema2_failure_after_first_variant_cleans_explain_and_preserves_sources(
        tmp_path, monkeypatch, capsys, mode):
    """A late owner failure must not leave runtime resources or partial success output."""
    import tempfile
    scratch = tmp_path / 'scratch'
    scratch.mkdir()
    monkeypatch.setattr(tempfile, 'tempdir', str(scratch))
    bad = [-.1, .08, .06981317007977318]
    p = write(tmp_path, document(NOISE, [BASE, bad], [BASE, bad]))

    def forbidden(*args, **kwargs):
        pytest.fail('invalid later variant reached experiment execution')

    monkeypatch.setattr(cli, 'run_experiment', forbidden)
    assert cli.main(['--config', str(p), *mode], share=share) == 2
    captured = capsys.readouterr()
    if '--explain' in mode:
        assert captured.out == '' and json.loads(captured.err)['error']
    else:
        assert captured.err == '' and json.loads(captured.out)['error']
    assert not list(scratch.iterdir())


@pytest.mark.parametrize('path', [
    'target_dynamics.target.dynamics.parameters.alpha',
    'description.robot.manipulator.joints.shoulder.lower',
    'controller_parameters.tracking_radius', 'duration',
    'measurements.measurements.target_position.random_seed',
    'localization_sensor.localization_sensor.source_frame',
    'evaluation.definition_sha256',
])
def test_protected_conditions_cannot_be_assigned(tmp_path, path):
    """Sensor assignments cannot be used to hide physical, algorithm, time or frame changes."""
    with pytest.raises(ValueError, match='not a permitted sensor leaf'):
        load_experiment(write(tmp_path, document(path)), share)


def test_yaml_parameter_merges_are_rejected_as_ambiguous(tmp_path):
    """An alias merge must not introduce an implicit second parameter value."""
    p = write(tmp_path, document())
    text = p.read_text().replace('parameters:', 'parameters: &parameters', 1)
    text = text.replace('parameters:\n', 'parameters:\n      <<: *parameters\n', 1)
    p.write_text(text)
    with pytest.raises(ValueError, match='ambiguous assignment'):
        load_experiment(p, share)


def test_disabled_producer_cannot_receive_sensor_assignments(tmp_path):
    """A configured but disabled producer is different from an active measurement source."""
    sensor = write(tmp_path, {
        'extends': str(SRC / 'robot_simulation/config/sensors/planar_localization.yaml'),
        'localization_sensor': {'enabled': False}}, 'sensor.yaml')
    simulation = write(tmp_path, {
        'extends': str(SRC / 'robot_simulation/config/scenarios'
                       / 'base_relative_manipulation.yaml'),
        'localization_sensor_config': str(sensor)}, 'simulation.yaml')
    profile = write(tmp_path, {
        'extends': str(SRC / 'robot_bringup/config/reactive_manipulation_localized.yaml'),
        'simulation_config': str(simulation)}, 'profile.yaml')
    system = write(tmp_path, {
        'extends': str(SRC / 'robot_bringup/config/system_manipulation_D_localized.yaml'),
        'simulation': {'config': str(profile)}}, 'system.yaml')
    d = document()
    d['experiment']['system_config'] = str(system)
    with pytest.raises(ValueError, match='producer unavailable or disabled'):
        spec = load_experiment(write(tmp_path, d), share)
        prepare(spec, spec['variants'][0], tmp_path / 'run')


def test_legacy_and_new_validate_only_keep_identity_output(capsys):
    """Both versions retain the validation CLI's existing identity output shape."""
    for name in ['sensor_noise_comparison.yaml', 'sensor_noise_study.yaml']:
        assert cli.main(['--config', str(SRC / 'robot_experiments/config' / name),
                         '--validate-only'], share=share) == 0
        value = json.loads(capsys.readouterr().out)
        assert set(value) == {'schema_version', 'valid', 'variants'}
        assert set(value['variants']) == {'baseline', 'higher_noise'}


@pytest.mark.parametrize('scope', [
    'root', 'experiment', 'variant', 'comparison', 'variable', 'parameters'])
def test_yaml_merges_cannot_hide_assignment_declarations(tmp_path, scope):
    """Reject merged declaration scopes before duplicate bindings can be hidden."""
    root = yaml.compose(yaml.safe_dump(document()))

    def field(node, key):
        return next(value for name, value in node.value if name.value == key)

    experiment = field(root, 'experiment')
    variant = field(experiment, 'variants').value[0]
    comparison = field(experiment, 'comparison')
    parameters = field(variant, 'parameters')
    parameters.value.append(deepcopy(parameters.value[0]))
    selected = {'root': root, 'experiment': experiment, 'variant': variant,
                'comparison': comparison, 'variable': field(comparison, 'variables').value[0],
                'parameters': parameters}[scope]
    merged = yaml.MappingNode('tag:yaml.org,2002:map', selected.value)
    selected.value = [(yaml.ScalarNode('tag:yaml.org,2002:merge', '<<'), merged)]
    path = tmp_path / 'merged.yaml'
    path.write_text(yaml.serialize(root))
    with pytest.raises(ValueError, match='ambiguous assignment|duplicate assignment'):
        load_experiment(path, share)
