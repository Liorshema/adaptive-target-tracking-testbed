"""Experiment revisions are immutable definitions, independent of executions."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from robot_experiments.catalog import ExperimentCatalog, CatalogConflict
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.configuration import resolve_system_resource

SRC = Path(__file__).resolve().parents[2]
SHARE = lambda package: SRC/package


@pytest.fixture
def catalog(tmp_path):
    path = tmp_path/'experiments.yaml'
    import yaml
    state = load_yaml(SHARE('robot_experiments')/'config/experiments.yaml')
    # Start CRUD concurrency fixtures at imported revision 1; shipped migration
    # revisions are exercised separately without rewriting installed defaults.
    for entry in state['experiments'].values():
        entry['revisions'] = entry['revisions'][:1]
    path.write_text(yaml.safe_dump(state, sort_keys=False))
    return ExperimentCatalog(path, SHARE)


def test_builtin_catalog_reproduces_every_entry_and_has_immutable_identity(catalog):
    assert len(catalog.list()) == 7
    for item in catalog.list():
        record = catalog.get(item['id'])
        spec = catalog.prepare(item['id'])
        assert spec['experiment_identity'] == {
            'id': item['id'], 'revision': 1, 'definition_sha256': record['sha256']}
        assert spec['variants'] and spec['source_document'] == record['definition']
        record['definition']['experiment']['seeds'] = [99]
        assert catalog.get(item['id'])['definition']['experiment']['seeds'] == [42] or (
            catalog.get(item['id'])['definition']['experiment']['seeds'] == [42, 917])


def test_revision_conflict_soft_delete_and_purge_preserve_result_files(catalog):
    record = catalog.get('EXP-NOMINAL')
    result = catalog.path.parent/'runs/old/metrics.json'
    result.parent.mkdir(parents=True); result.write_text('{}')
    catalog.create('EXP-CUSTOM', 'Custom', record['definition'])
    old = catalog.get('EXP-CUSTOM')
    definition = deepcopy(old['definition']); definition['experiment']['seeds'] = [43]
    assert catalog.revise('EXP-CUSTOM', definition, expected_revision=1)['revision'] == 2
    assert catalog.get('EXP-CUSTOM', 1) == old
    with pytest.raises(CatalogConflict):
        catalog.revise('EXP-CUSTOM', definition, expected_revision=1)
    catalog.delete('EXP-CUSTOM', expected_revision=2)
    assert 'EXP-CUSTOM' not in [r['id'] for r in catalog.list()]
    with pytest.raises(ValueError, match='deleted'):
        catalog.prepare('EXP-CUSTOM')
    assert catalog.get('EXP-CUSTOM', 1)['sha256'] == old['sha256']
    catalog.purge('EXP-CUSTOM', expected_revision=2)
    assert result.read_text() == '{}'
    with pytest.raises(KeyError): catalog.get('EXP-CUSTOM', 1)
    with pytest.raises(CatalogConflict):
        catalog.create('EXP-CUSTOM', 'Reused identity', record['definition'])


def test_drafts_reset_and_effective_overrides_never_change_source_defaults(catalog, tmp_path):
    before = {p: p.read_bytes() for p in SRC.rglob('*.yaml')}
    draft = catalog.draft('EXP-NOMINAL')
    draft['overrides'] = {'prediction': {'imm': {'likelihood_variance': .5}}}
    catalog.create('EXP-OVERRIDE', 'Override', draft['definition'], draft['overrides'])
    spec = catalog.prepare('EXP-OVERRIDE')
    from robot_experiments.configuration import materialize_run
    path = tmp_path/'run'; path.mkdir()
    prepared = materialize_run(spec['variants'][0], 42, path, SHARE, spec['runtime'])
    assert load_yaml(path/'inputs/prediction.yaml')['predictors']['imm']['parameters']['likelihood_variance'] == .5
    assert prepared['scientific_conditions']['prediction_configuration']['predictors']['imm']['parameters']['likelihood_variance'] == .5
    assert catalog.reset(draft)['overrides'] == {}
    assert catalog.draft()['overrides'] == {}
    assert catalog.get('EXP-NOMINAL')['overrides'] == {}
    assert before == {p: p.read_bytes() for p in before}


@pytest.mark.parametrize('overrides', [
    {'prediction': {'imm': {'typo': 1}}}, {'prediction': {'prediction': {'dt': -1}}},
    {'prediction': {'prediction': {'horizon': True}}}, {'unknown_owner': {}},
    {'strategy': {'controller': {'parameters': {'motion_weights': [1]}}}},
])
def test_invalid_effective_parameters_fail_before_catalog_write(catalog, overrides):
    record = catalog.get('EXP-NOMINAL'); before = catalog.path.read_bytes()
    with pytest.raises(ValueError):
        catalog.create('EXP-BAD', 'Bad', record['definition'], overrides)
    assert catalog.path.read_bytes() == before


def test_list_replacement_and_runtime_precedence_are_deterministic(catalog):
    from robot_configuration.parameter_validation import override_parameters
    defaults = {'matrix': [[1., 2.], [3., 4.]]}
    assert override_parameters(defaults, {'matrix': [[5.]]}) == {'matrix': [[5.]]}
    assert defaults == {'matrix': [[1., 2.], [3., 4.]]}
    record = catalog.get('EXP-NOMINAL')
    catalog.create('EXP-RUNTIME', 'Runtime', record['definition'],
                   {'prediction': {'imm': {'likelihood_variance': .5}}})
    spec = catalog.prepare('EXP-RUNTIME', runtime_overrides={'enable_actuation': False})
    assert spec['runtime']['enable_actuation'] is False
    assert catalog.get('EXP-RUNTIME')['definition']['experiment']['runtime']['enable_actuation'] is True
    with pytest.raises(ValueError): catalog.prepare('EXP-RUNTIME', runtime_overrides={'typo': 1})


def test_catalog_detects_revision_tampering(catalog):
    import yaml
    state = load_yaml(catalog.path)
    state['experiments']['EXP-NOMINAL']['revisions'][0]['definition']['experiment']['seeds'] = [1]
    catalog.path.write_text(yaml.safe_dump(state))
    with pytest.raises(ValueError, match='fingerprint'): catalog.get('EXP-NOMINAL')


def test_concurrent_revision_writers_do_not_lose_history(catalog):
    from concurrent.futures import ThreadPoolExecutor
    definition = catalog.get('EXP-NOMINAL')['definition']
    def revise(seed):
        value = deepcopy(definition); value['experiment']['seeds'] = [seed]
        try:
            catalog.revise('EXP-NOMINAL', value, expected_revision=1)
            return 'written'
        except CatalogConflict:
            return 'conflict'
    with ThreadPoolExecutor(max_workers=2) as pool:
        assert sorted(pool.map(revise, [43, 44])) == ['conflict', 'written']
    assert catalog.get('EXP-NOMINAL', 1)['definition'] == definition
    assert catalog.get('EXP-NOMINAL')['revision'] == 2


def test_worker_uses_accepted_revision_after_catalog_changes(catalog, tmp_path, monkeypatch):
    import uuid
    from robot_experiments import jobs, runner
    from robot_experiments.reporting import write_json
    record = catalog.get('EXP-NOMINAL')
    updated = deepcopy(record['definition']); updated['experiment']['seeds'] = [43]
    catalog.revise('EXP-NOMINAL', updated, expected_revision=1)
    job_id = str(uuid.uuid4()); directory = tmp_path/job_id; directory.mkdir()
    write_json(directory/'request.json', {'job_id': job_id, 'record': record,
               'catalog': str(catalog.path), 'output': str(tmp_path/'results'), 'runtime_overrides': {}})
    write_json(directory/'status.json', {'job_id': job_id, 'state': 'queued'})
    seen = []
    monkeypatch.setattr(jobs, 'ExperimentCatalog', lambda path: catalog)
    monkeypatch.setattr(runner, 'run_experiment', lambda spec, share, **kwargs: (
        seen.append(spec) or (tmp_path/'study', 3)))
    assert jobs.worker(directory) == 3
    assert seen[0]['seeds'] == [42]
    assert seen[0]['experiment_identity']['revision'] == 1
    assert json.loads((directory/'status.json').read_text())['state'] == 'completed'


def test_stop_never_signals_reused_pid(tmp_path, monkeypatch):
    import uuid
    from robot_experiments import jobs
    from robot_experiments.reporting import write_json
    identifier = str(uuid.uuid4()); directory = tmp_path/identifier; directory.mkdir()
    write_json(directory/'status.json', {'state': 'running'})
    write_json(directory/'process.json', {'pid': 1234, 'created': 1.})
    class ReusedProcess:
        def create_time(self): return 2.
        def status(self): return 'running'
        def send_signal(self, signal): pytest.fail('signalled a reused PID')
    monkeypatch.setattr(jobs.psutil, 'Process', lambda pid: ReusedProcess())
    assert jobs.ExperimentJobs(tmp_path).stop(identifier)['state'] == 'worker_lost'


def test_management_cli_keeps_catalog_edits_separate_from_defaults(catalog, capsys):
    from robot_experiments.cli import main
    assert main(['catalog', 'list', '--catalog', str(catalog.path)], share=SHARE) == 0
    assert len(json.loads(capsys.readouterr().out)) == 7
    assert main(['--experiment', 'EXP-NOMINAL', '--catalog', str(catalog.path),
                 '--validate-only'], share=SHARE) == 0
    assert json.loads(capsys.readouterr().out)['valid']
    assert main(['catalog', 'delete', '--catalog', str(catalog.path), '--experiment',
                 'EXP-NOMINAL', '--expected-revision', '1'], share=SHARE) == 0
    capsys.readouterr()
    assert main(['--experiment', 'EXP-NOMINAL', '--catalog', str(catalog.path),
                 '--validate-only'], share=SHARE) == 2


def test_owner_overrides_cannot_replace_explicit_sensor_assignments(catalog, tmp_path):
    from robot_experiments.configuration import materialize_run
    definition = catalog.document('EXP-SENSOR-NOISE')
    path = 'measurements.measurements.target_position.noise.covariance'
    base = [[.005, 0., 0.], [0., .005, 0.], [0., 0., .005]]
    high = [[.01, 0., 0.], [0., .01, 0.], [0., 0., .01]]
    ex = definition['experiment']
    ex['comparison']['variables'] = [{'name': 'covariance', 'path': path, 'values': [base, high]}]
    for variant, value in zip(ex['variants'], [base, high]):
        variant['parameters'] = {'covariance': value}
    overrides = {'measurements': {'measurements': {'target_position': {'noise': {'covariance': base}}}}}
    catalog.create('EXP-COVARIANCE', 'Covariance', definition, overrides)
    spec = catalog.prepare('EXP-COVARIANCE')
    run = tmp_path/'run'; run.mkdir()
    actual = materialize_run(spec['variants'][1], 42, run, SHARE, spec['runtime'])
    assert actual['comparison_contract']['measurements']['measurements']['target_position']['noise']['covariance'] == high
    assert actual['snapshot']['assignments']['covariance']['requested_value'] == high


def test_stop_is_idempotent_and_does_not_interrupt_cleanup(tmp_path, monkeypatch):
    from robot_experiments import jobs
    import uuid
    from robot_experiments.reporting import write_json
    identifier = str(uuid.uuid4()); directory = tmp_path/identifier; directory.mkdir()
    write_json(directory/'status.json', {'state': 'running'})
    write_json(directory/'process.json', {'pid': 1234, 'created': 1.})
    class Worker:
        def create_time(self): return 1.
        def status(self): return 'running'
        def send_signal(self, signal): pytest.fail('stop interrupted worker cleanup')
    monkeypatch.setattr(jobs.psutil, 'Process', lambda pid: Worker())
    # A zero timeout is sufficient to inspect the cancellation side effect.
    for _ in range(2):
        with pytest.raises(TimeoutError): jobs.ExperimentJobs(tmp_path).stop(identifier, timeout=0.)
    assert (directory/'stop.request').exists()


def test_revision_result_is_the_commit_even_if_a_later_writer_advances(catalog, monkeypatch):
    definition = catalog.get('EXP-NOMINAL')['definition']
    original = catalog.get
    reads = []
    def competing_latest(identifier, revision=None):
        value = original(identifier, revision)
        reads.append(value['revision'])
        if len(reads) > 1 and revision is None:
            # A subsequent writer may commit after this operation releases its lock.
            value['revision'] = 3
        return value
    monkeypatch.setattr(catalog, 'get', competing_latest)
    result = catalog.revise('EXP-NOMINAL', definition, expected_revision=1)
    assert result['revision'] == 2


def test_historical_system_resource_is_explicitly_migrated_to_experiment_owner():
    from robot_experiments import configuration
    assert hasattr(configuration, 'resolve_system_resource')
    path = configuration.resolve_system_resource(
        {'package': 'robot_bringup', 'path': 'config/system_nominal.yaml'},
        SRC / 'robot_experiments/config', SHARE)
    assert path == (resolve_system_resource(SHARE('robot_experiments') / 'config/systems/nominal_tracking.yaml', Path.cwd(), lambda package: Path(__file__).resolve().parents[2] / package)).resolve()


@pytest.mark.parametrize('key,value', [
    ('controller_type', 'jacobian'), ('reference_policy_type', 'radial'),
    ('strategy_configuration', '{}')])
def test_parameter_overrides_cannot_compete_with_composition_selection(catalog, key, value):
    definition = catalog.document('EXP-NOMINAL')
    with pytest.raises(ValueError, match='composition'):
        catalog.create('EXP-COMPETING', 'Competing selection', definition,
                       {'control': {key: value}})


def test_shipped_migration_preserves_imported_revisions_and_uses_current_owners():
    catalog = ExperimentCatalog(share=SHARE)
    for item in catalog.list():
        assert item['revision'] == 3
        imported = catalog.get(item['id'], 1)
        current = catalog.get(item['id'], 2)
        assert imported['sha256'] != current['sha256']
        for node in [current['definition']['experiment'],
                     *current['definition']['experiment']['variants']]:
            if 'system_config' in node:
                assert node['system_config']['package'] == 'robot_experiments'
        a, b = catalog.prepare(item['id'], 1), catalog.prepare(item['id'], 2)
        assert [v['plan']['identities'] for v in a['variants']] == [
            v['plan']['identities'] for v in b['variants']]
