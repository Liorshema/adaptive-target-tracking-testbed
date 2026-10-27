"""Scientific variables may relax exact approved leaves, never surrounding contracts."""

import hashlib
import json
from copy import deepcopy
from pathlib import Path

import pytest

from robot_experiments.benchmarking import summarize
from robot_experiments.configuration import load_experiment, materialize_run

import yaml
from robot_experiments.configuration import resolve_system_resource


SRC = Path(__file__).resolve().parents[2]
PATH = 'localization_sensor.localization_sensor.white_noise_std'


def policy(path=PATH, **variable):
    """Declare one exact leaf without authorizing its siblings."""
    return {'schema_version': 1, 'domain': 'sensor_parameters',
            'variables': [{'name': 'sensor_effect', 'path': path, **variable}]}


def contract():
    """Use independent literal physical, sensor and timing conditions."""
    return {'localization_sensor': {'localization_sensor': {
        'white_noise_std': [.04, .04, .035], 'calibration_bias_std': [.01, .01, .009],
        'dropout_probability': .05, 'random_seed': 123, 'source_frame': 'world'}},
        'description': {'geometry': [1., 2.]}, 'duration': 3., 'settling_time': 1.}


def result(name, value=None, *, spec=None, original=None, conditions=None):
    """Build real comparison views with minimal complete metric evidence."""
    from robot_experiments.comparison import comparison_views
    original = deepcopy(original or contract())
    if value is not None:
        original['localization_sensor']['localization_sensor']['white_noise_std'] = value
    conditions = conditions or {'physics': {'max_step_size': .002},
                                'evaluation': {'required_metrics': ['tracking_distance']}}
    view = comparison_views(original, spec or policy(), conditions)
    metrics = {
        'schema_version': 1,
        'evaluation_window': {'start': 1., 'end': 4., 'unit': 's'},
        'contract': {'alignment': 'nearest', 'max_skew': .08},
        'safety': {'coverage_complete': True, 'accepted': True},
        'streams': {'base_truth': {'accepted': 10}},
        'metrics': {'tracking_distance': {
            'available': True, 'mean': .3, 'unit': 'm', 'sample_count': 10}},
    }
    return {'variant': name, 'seed': 42, 'execution_status': 'completed', 'acceptance': True,
            'comparison_contract': view['original_hash'], 'comparison': view,
            'artifacts': 'runs/' + name, 'metrics': metrics}


def compare(a, b, declaration=None):
    """Exercise the actual aggregation boundary with an explicit policy."""
    return summarize([a, b], ['base', 'other'], [42],
                     comparison_policy=declaration or policy())


def test_original_hash_is_preserved_and_strict_default_still_rejects_noise():
    """Adding optional views cannot reinterpret a historical strict hash."""
    a, b = result('base'), result('other', [.08, .08, .07])
    original = a['comparison']['original_contract']
    assert a['comparison_contract'] == hashlib.sha256(
        json.dumps(original, sort_keys=True).encode()).hexdigest()
    assert summarize([a, b], ['base', 'other'], [42])['comparable'] is False
    assert summarize([a, result('other')], ['base', 'other'], [42])['comparable'] is True
    assert a['comparison']['original_hash'] != b['comparison']['original_hash']
    assert a['comparison']['invariant_hash'] == b['comparison']['invariant_hash']


def test_declared_noise_and_values_are_reported_without_changing_source_contract():
    """Eligibility must retain real values, evidence and normal aggregation."""
    a, b = result('base'), result('other', [.08, .08, .07])
    before = deepcopy(a)
    summary = compare(a, b)
    assert summary['comparable'] is True
    assert a == before
    assert summary['variants']['base']['metrics']['tracking_distance']['mean'] == .3
    assert b['comparison']['variables']['sensor_effect']['value'] == [.08, .08, .07]
    assert summary['comparison']['schema_version'] == 1
    assert summary['comparison']['eligible'] is True
    assert summary['runs'][1]['artifacts'] == 'runs/other'


def test_dropout_is_an_independent_authorized_variable():
    """Selection works for an approved scalar as well as a vector leaf."""
    spec = policy('localization_sensor.localization_sensor.dropout_probability', values=[.05, .2])
    a = result('base', spec=spec)
    original = contract()
    original['localization_sensor']['localization_sensor']['dropout_probability'] = .2
    b = result('other', spec=spec, original=original)
    assert compare(a, b, spec)['comparable'] is True


@pytest.mark.parametrize('key,value', [
    ('dropout_probability', .2), ('random_seed', 999), ('source_frame', 'map')])
def test_undeclared_leaf_in_same_sensor_file_is_rejected(key, value):
    """Authorizing one sensor property must not hide other changes."""
    original = contract()
    original['localization_sensor']['localization_sensor'][key] = value
    summary = compare(result('base'), result('other', [.08, .08, .07], original=original))
    assert summary['comparable'] is False
    assert key in ' '.join(summary['incomparability_reasons'])


@pytest.mark.parametrize('path', [
    'description.geometry', 'holding.backend',
    'localization_sensor', 'localization_sensor.localization_sensor',
    'localization_sensor.localization_sensor.random_seed',
    'localization_sensor.localization_sensor.source_frame',
    'localization_sensor.localization_sensor.white_noise_std.0',
    'localization_sensor.*.white_noise_std', 'imaginary.noise'])
def test_protected_container_nonexistent_and_ambiguous_paths_are_rejected(path):
    """Only exact approved leaves can change, regardless of declaration intent."""
    from robot_experiments.comparison import validate_policy
    with pytest.raises(ValueError, match='path|permitted|protected'):
        validate_policy(policy(path))


@pytest.mark.parametrize('change', ['version', 'domain', 'duplicate', 'unknown', 'values'])
def test_invalid_declarations_fail_before_execution(change):
    """Reject ambiguous versions, aliases, duplicates and nonfinite values."""
    from robot_experiments.comparison import validate_policy
    spec = policy()
    if change == 'version':
        spec['schema_version'] = True
    if change == 'domain':
        spec['domain'] = 'anything'
    if change == 'duplicate':
        spec['variables'].append(deepcopy(spec['variables'][0]))
    if change == 'unknown':
        spec['variables'][0]['mask_parent'] = True
    if change == 'values':
        spec['variables'][0]['values'] = [float('nan')]
    with pytest.raises(ValueError):
        validate_policy(spec)


def test_present_path_and_expected_effective_values_are_required():
    """A valid schema path must still exist and match its optional value set."""
    from robot_experiments.comparison import comparison_views
    with pytest.raises(ValueError, match='absent|missing|exist'):
        comparison_views({'description': {}}, policy(), {})
    with pytest.raises(ValueError, match='value'):
        comparison_views(contract(), policy(values=[[.08, .08, .07]]), {})


def test_physics_algorithms_and_metrics_cannot_hide_behind_sensor_variation():
    """Invariant context and metric definitions remain scientific conditions."""
    a = result('base')
    b = result('other', [.08, .08, .07], conditions={
        'physics': {'max_step_size': .01},
        'evaluation': {'required_metrics': ['tracking_distance']}})
    assert compare(a, b)['comparable'] is False
    b = result('other', [.08, .08, .07])
    b['metrics']['contract']['alignment'] = 'different'
    assert compare(a, b)['comparable'] is False
    b['metrics']['contract'] = deepcopy(a['metrics']['contract'])
    b['metrics']['metrics']['tracking_distance']['unit'] = 'cm'
    assert compare(a, b)['comparable'] is False


@pytest.mark.parametrize('issue', [
    'failed', 'interrupted', 'missing_metric', 'missing_coverage', 'short_window'])
def test_incomplete_or_missing_evidence_never_becomes_complete_comparison(issue):
    """Retain partial evidence while withholding scientifically invalid aggregates."""
    a, b = result('base'), result('other', [.08, .08, .07])
    if issue in ['failed', 'interrupted']:
        b['execution_status'] = 'failed'
        b['status'] = {'error': {'code': issue}}
    if issue == 'missing_metric':
        b['metrics']['metrics']['tracking_distance']['available'] = False
    if issue == 'missing_coverage':
        b['metrics']['safety']['coverage_complete'] = False
    if issue == 'short_window':
        b['metrics']['evaluation_window']['end'] = 2.
    summary = compare(a, b)
    assert summary['comparable'] is False
    assert summary['variants']['base']['metrics'] == {}
    assert summary['runs'][1]['execution_status'] == b['execution_status']


def test_poor_tracking_is_an_outcome_not_comparison_failure():
    """Accuracy and safety acceptance are separate from controlled-setup eligibility."""
    a, b = result('base'), result('other', [.08, .08, .07])
    b['metrics']['metrics']['tracking_distance']['mean'] = 100.
    b['acceptance'] = False
    summary = compare(a, b)
    assert summary['comparable'] is True
    assert summary['variants']['other']['metrics']['tracking_distance']['mean'] == 100.
    assert summary['variants']['other']['unconditional_winner_eligible'] is False


def test_real_inheritance_materialization_and_seeds_are_policy_inputs(tmp_path):
    """Different owner identities with equal resolved values have equal invariants."""
    from robot_experiments.comparison import comparison_views
    source = SRC / 'robot_experiments/config/nominal.yaml'
    from robot_experiments.catalog import ExperimentCatalog
    document = ExperimentCatalog(share=lambda name: SRC/name).document('EXP-NOMINAL')
    document['experiment'].update({'comparison': policy(), 'variants': [{'name': 'base',
                                                                         'system_config': {'package': 'robot_bringup',
                                                                                           'path': 'config/system_manipulation_D_localized.yaml'}}]})
    path = tmp_path / 'experiment.yaml'
    path.write_text(yaml.safe_dump(document))
    spec = load_experiment(path, lambda p: SRC / p)
    for name in ['one', 'two']:
        if name == 'two':
            child = tmp_path / 'equivalent_system.yaml'
            parent = resolve_system_resource(
                SRC / 'robot_experiments/config/systems/mobile_manipulator_pose_tracking.yaml',
                Path.cwd(),
                lambda package: Path(__file__).resolve().parents[2] / package)
            child.write_text(yaml.safe_dump({'extends': str(parent),
                                            'runtime': {'publish_ground_truth': True}}))
            document['experiment']['variants'][0]['system_config'] = str(child)
            path.write_text(yaml.safe_dump(document))
            spec = load_experiment(path, lambda p: SRC / p)
        directory = tmp_path / name
        directory.mkdir()
        prepared = materialize_run(
            spec['variants'][0], 917, directory, lambda p: SRC / p, spec['runtime'])
        view = comparison_views(prepared['comparison_contract'], spec['comparison'],
                                prepared['scientific_conditions'])
        assert view['variables']['sensor_effect']['value'] == [.04, .04, .03490658503988659]
        seeds = prepared['snapshot']['seed_consumers']
        assert seeds['measurements.target_position.random_seed'] == 917
        sensor = prepared['plan']['sensor_inventory']['sources']['external_planar_localization']
        assert sensor['simulated_effects']['random_seed'] == (
            seeds['localization_sensor.random_seed'])
        if name == 'one':
            first = view
        else:
            assert first['invariant_hash'] == view['invariant_hash']


def test_policy_is_not_specific_to_a_localization_implementation():
    """The same leaf mechanism handles another existing measurement producer."""
    original = {'duration': 3., 'measurements': {'measurements': {'target_position': {
        'noise': {'covariance': [[.01, 0.], [0., .01]]}, 'random_seed': 42}}}}
    spec = policy('measurements.measurements.target_position.noise.covariance')
    a = result('base', spec=spec, original=original)
    target = original['measurements']['measurements']['target_position']
    target['noise']['covariance'] = [[.02, 0.], [0., .02]]
    b = result('other', spec=spec, original=original)
    assert compare(a, b, spec)['comparable'] is True


def test_full_physical_difference_is_rejected_with_approved_noise_declaration():
    """An approved noise variable cannot hide changed robot geometry."""
    original = contract()
    original['description']['geometry'][0] = 100.
    assert compare(result('base'), result('other', original=original))['comparable'] is False


def test_forged_masked_hash_cannot_conceal_an_undeclared_change():
    """Recompute views from original data before trusting a supplied hash."""
    a, b = result('base'), result('other')
    b['comparison']['original_contract']['description']['geometry'][0] = 100.
    b['comparison']['invariant_hash'] = a['comparison']['invariant_hash']
    assert compare(a, b)['comparable'] is False


def test_recorded_invariant_contents_must_match_the_recomputed_view():
    """Published invariant contents and their hash must describe the same view."""
    a, b = result('base'), result('other')
    b['comparison']['invariant_contract']['description']['geometry'] = [999.]
    assert compare(a, b)['comparable'] is False


def test_null_evidence_is_rejected_instead_of_crashing_a_partial_study():
    """An absent metrics document is unavailable evidence, not a complete run."""
    a, b = result('base'), result('other')
    b['metrics'] = None
    assert compare(a, b)['comparable'] is False
