"""Management edits only owned metadata; immutable scientific evidence survives."""

import json
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy

import pytest

from robot_experiments.catalog import CatalogConflict, ExperimentCatalog
from robot_experiments.reporting import write_json

from test_backend import backend, recorded_run, share


@pytest.fixture
def api(tmp_path):
    """Initialize an isolated existing owner with registered test results."""
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    return backend(tmp_path, catalog=catalog, roots=[tmp_path / 'results'])


def files(root):
    """Snapshot complete file bytes to detect unintended evidence writes."""
    return {p.relative_to(root): p.read_bytes() for p in root.rglob('*') if p.is_file()}


def test_rename_only_changes_session_title_and_delete_preserves_all_runs(api, tmp_path):
    """Change only the title and delete only the owned reference document."""
    run_id, run = recorded_run(tmp_path / 'results')
    session = api.create_session({'title': 'Original', 'notes': 'Keep notes'})
    session = api.add_run_to_session(session['session_id'], run_id)
    before = files(tmp_path / 'results')
    catalog_before = api.catalog.path.read_bytes()
    selected = api.get_session(session['session_id'])
    renamed = api.rename_session(
        session['session_id'], '  New name  ', expected_token=selected['token']
    )
    expected = deepcopy(session)
    expected['metadata']['title'] = 'New name'
    assert renamed['session'] == expected
    assert renamed['token'] != selected['token']
    with pytest.raises(CatalogConflict):
        api.delete_session(session['session_id'], expected_token=selected['token'])
    assert api.delete_session(session['session_id'], expected_token=renamed['token']) is True
    assert api.delete_session(session['session_id'], expected_token=renamed['token']) is False
    assert api.list_sessions() == []
    with pytest.raises(KeyError):
        api.rename_session(session['session_id'], 'Resurrect', expected_token=renamed['token'])
    assert files(tmp_path / 'results') == before
    assert api.catalog.path.read_bytes() == catalog_before
    assert run.is_dir()


def test_stale_rename_never_overwrites_concurrent_addition(api, tmp_path):
    """Reject stale title edits without losing an appended Run reference."""
    run_id, _ = recorded_run(tmp_path / 'results')
    sid = api.create_session({'title': 'Before'})['session_id']
    selected = api.get_session(sid)
    api.add_run_to_session(sid, run_id)
    with pytest.raises(CatalogConflict):
        api.rename_session(sid, 'After', expected_token=selected['token'])
    actual = api.get_session(sid)['session']
    assert actual['metadata']['title'] == 'Before'
    assert [r['run_id'] for r in actual['runs']] == [run_id]


def test_concurrent_renames_allow_only_one_writer(api):
    """Serialize competing writers and enforce their shared metadata token."""
    sid = api.create_session({'title': 'Before'})['session_id']
    token = api.get_session(sid)['token']
    barrier = threading.Barrier(2)

    def rename(title):
        barrier.wait(timeout=5)
        try:
            api.rename_session(sid, title, expected_token=token)
            return 'written'
        except CatalogConflict:
            return 'conflict'

    with ThreadPoolExecutor(max_workers=2) as pool:
        assert sorted(pool.map(rename, ['A', 'B'])) == ['conflict', 'written']
    assert api.get_session(sid)['session']['metadata']['title'] in {'A', 'B'}


@pytest.mark.parametrize('target', ['document', 'directory', 'lock'])
def test_session_management_rejects_symlink_ownership_escape(api, tmp_path, target):
    """Reject substituted metadata documents, directories and lock files."""
    sid = api.create_session({'title': 'Original'})['session_id']
    token = api.get_session(sid)['token']
    path = api.directory / 'sessions' / (sid + '.json')
    external = tmp_path / 'external'
    external.mkdir()
    if target == 'document':
        copy = external / path.name
        copy.write_bytes(path.read_bytes())
        path.unlink()
        path.symlink_to(copy)
    elif target == 'directory':
        original = path.parent
        original.rename(external / 'sessions')
        original.symlink_to(external / 'sessions', target_is_directory=True)
    else:
        copy = external / 'victim'
        copy.write_text('unchanged')
        path.with_suffix('.lock').symlink_to(copy)
    before = files(external)
    with pytest.raises(ValueError, match='owned|symlink|Session'):
        api.delete_session(sid, expected_token=token)
    assert files(external) == before


@pytest.mark.parametrize(
    'state,cleanup,blocked',
    [
        ('queued', None, True),
        ('running', None, True),
        ('worker_lost', None, True),
        ('failed', False, True),
        ('failed', None, True),
        ('cancelled', True, False),
        ('completed', None, True),
        ('unknown', True, True),
    ],
)
def test_session_delete_checks_active_and_unresolved_jobs(api, state, cleanup, blocked):
    """Require proven terminal cleanup before removing associated metadata."""
    sid = api.create_session({'title': 'Associated'})['session_id']
    token = api.get_session(sid)['token']
    jid = str(uuid.uuid4())
    path = api.jobs.directory / jid
    path.mkdir(parents=True)
    write_json(
        path / 'request.json',
        {'job_id': jid, 'session_id': sid, 'record': {'id': 'EXP-NOMINAL'}},
    )
    status = {'job_id': jid, 'state': state}
    if cleanup is not None:
        status['cleanup_complete'] = cleanup
    write_json(path / 'status.json', status)
    if blocked:
        with pytest.raises(ValueError, match='Job|cleanup'):
            api.delete_session(sid, expected_token=token)
        assert api.get_session(sid)['token'] == token
    else:
        assert api.delete_session(sid, expected_token=token)


def test_legacy_session_run_reference_is_enough_to_block_active_job(api, tmp_path):
    """Recognize historical Session association through its Run study."""
    run_id, run = recorded_run(tmp_path / 'results')
    sid = api.create_session()['session_id']
    api.add_run_to_session(sid, run_id)
    jid = str(uuid.uuid4())
    path = api.jobs.directory / jid
    path.mkdir(parents=True)
    write_json(path / 'request.json', {'job_id': jid, 'record': {'id': 'EXP-RECORDED'}})
    write_json(
        path / 'status.json',
        {'job_id': jid, 'state': 'running', 'study': str(run.parent.parent)},
    )
    with pytest.raises(ValueError, match='Job'):
        api.delete_session(sid, expected_token=api.get_session(sid)['token'])


def test_archive_is_revision_guarded_and_retains_every_historical_revision(api):
    """Keep revision bytes available after an optimistic soft deletion."""
    before = [api.catalog.get('EXP-NOMINAL', n) for n in (1, 2, 3)]
    with pytest.raises(CatalogConflict):
        api.archive_experiment('EXP-NOMINAL', expected_revision=1)
    api.archive_experiment('EXP-NOMINAL', expected_revision=3)
    assert 'EXP-NOMINAL' not in {x['id'] for x in api.list_experiments()}
    assert [api.catalog.get('EXP-NOMINAL', n) for n in (1, 2, 3)] == before


@pytest.mark.parametrize(
    'state,cleanup',
    [('queued', None), ('running', None), ('worker_lost', None), ('failed', False)],
)
def test_archive_blocks_active_and_uncertain_jobs(api, state, cleanup):
    """Refuse archive while execution or cleanup remains unresolved."""
    jid = str(uuid.uuid4())
    path = api.jobs.directory / jid
    path.mkdir(parents=True)
    write_json(path / 'request.json', {'job_id': jid, 'record': {'id': 'EXP-NOMINAL'}})
    status = {'job_id': jid, 'state': state}
    if cleanup is not None:
        status['cleanup_complete'] = cleanup
    write_json(path / 'status.json', status)
    before = api.catalog.path.read_bytes()
    with pytest.raises(ValueError, match='Job|cleanup'):
        api.archive_experiment('EXP-NOMINAL', expected_revision=3)
    assert api.catalog.path.read_bytes() == before


@pytest.mark.parametrize(
    'name,want',
    [
        ('Dynamics Comparison', 'EXP-DYNAMICS-COMPARISON'),
        ('  café / A+B! ', 'EXP-CAFE-A-B'),
        ('robot___ 42', 'EXP-ROBOT-42'),
    ],
)
def test_generated_ids_are_predictable_and_catalog_valid(api, name, want):
    """Normalize display names through one deterministic identity policy."""
    assert api.propose_experiment_id(name) == want
    assert api.validate_experiment_id(want) == want
    assert api.propose_experiment_id(name) == want


def test_non_latin_and_long_names_have_predictable_bounded_identifiers(api):
    """Use stable hashes when transliteration or length cannot supply a slug."""
    assert api.propose_experiment_id('שלום') == 'EXP-B7AC0398EF74'
    identifier = api.propose_experiment_id('a' * 100)
    assert len(identifier) <= 75
    assert identifier != api.propose_experiment_id('a' * 99 + 'b')
    with pytest.raises(ValueError):
        api.propose_experiment_id('   ')


def test_id_collisions_include_deleted_and_permanently_retired(api):
    """Reserve existing identities across all Catalog lifecycle states."""
    with pytest.raises(CatalogConflict):
        api.propose_experiment_id('Nominal')
    api.catalog.delete('EXP-NOMINAL', expected_revision=3)
    with pytest.raises(CatalogConflict):
        api.validate_experiment_id('EXP-NOMINAL')
    api.catalog.purge('EXP-NOMINAL', expected_revision=3)
    with pytest.raises(CatalogConflict):
        api.validate_experiment_id('EXP-NOMINAL')


def test_start_request_pins_session_and_blocks_delete_until_job_cleanup(
    api, tmp_path, monkeypatch
):
    # Keep real preparation/request/status writes; only external process creation is replaced.
    """Persist Session association before worker startup can create a Run."""
    monkeypatch.setattr(api.jobs, '_spawn_worker', lambda directory: None)
    sid = api.create_session({'title': 'Queued'})['session_id']
    result = api.start_run('EXP-NOMINAL', 3, output=tmp_path / 'results', session_id=sid)
    request = json.loads((api.jobs.directory / result['job_id'] / 'request.json').read_text())
    assert request['session_id'] == sid
    token = api.get_session(sid)['token']
    with pytest.raises(ValueError, match='Job'):
        api.delete_session(sid, expected_token=token)
    write_json(
        api.jobs.directory / result['job_id'] / 'status.json',
        {'job_id': result['job_id'], 'state': 'cancelled', 'cleanup_complete': True},
    )
    assert api.delete_session(sid, expected_token=token)
    with pytest.raises(KeyError):
        api.start_run('EXP-NOMINAL', 3, output=tmp_path / 'results', session_id=sid)
    assert len(list(api.jobs.directory.glob('*/request.json'))) == 1


def test_archive_cannot_pass_guard_while_job_creation_is_in_progress(
    api, tmp_path, monkeypatch
):
    """Serialize archive and accepted Job creation at the backend boundary."""
    entered, release = threading.Event(), threading.Event()

    def spawn(directory):
        entered.set()
        assert release.wait(timeout=10)

    monkeypatch.setattr(api.jobs, '_spawn_worker', spawn)
    with ThreadPoolExecutor(max_workers=2) as pool:
        start = pool.submit(api.start_run, 'EXP-NOMINAL', 3, output=tmp_path / 'results')
        assert entered.wait(timeout=10)
        archive = pool.submit(api.archive_experiment, 'EXP-NOMINAL', expected_revision=3)
        assert not archive.done()
        release.set()
        assert start.result(timeout=10)['state'] == 'queued'
        with pytest.raises(ValueError, match='Job'):
            archive.result(timeout=10)
    assert 'EXP-NOMINAL' in {x['id'] for x in api.list_experiments()}


def test_deleted_session_cannot_be_resurrected_by_run_append(api, tmp_path):
    """Prevent delayed append from recreating a removed Session document."""
    run_id, _ = recorded_run(tmp_path / 'results')
    sid = api.create_session()['session_id']
    assert api.delete_session(sid, expected_token=api.get_session(sid)['token'])
    with pytest.raises(KeyError):
        api.add_run_to_session(sid, run_id)
    assert api.list_sessions() == []


def test_listing_sessions_tolerates_deletion_after_directory_snapshot(api, monkeypatch):
    """Tolerate a missing Session during concurrent dropdown refresh."""
    removed = api.create_session({'title': 'Removed'})['session_id']
    survivor = api.create_session({'title': 'Survivor'})['session_id']
    original = api._session

    def read(session_id):
        if session_id == removed:
            (api.directory / 'sessions' / (session_id + '.json')).unlink()
        return original(session_id)

    monkeypatch.setattr(api, '_session', read)
    assert [s['session_id'] for s in api.list_sessions()] == [survivor]


@pytest.mark.parametrize('job_request', [None, [], {'record': None}])
def test_archive_fails_closed_when_job_association_cannot_be_verified(api, job_request):
    """Reject management when owned request association is unreadable."""
    jid = str(uuid.uuid4())
    path = api.jobs.directory / jid
    path.mkdir(parents=True)
    if job_request is not None:
        write_json(path / 'request.json', job_request)
    write_json(
        path / 'status.json',
        {'job_id': jid, 'state': 'running', 'experiment_identity': {'id': 'EXP-NOMINAL'}},
    )
    with pytest.raises(ValueError, match='Job|cleanup'):
        api.archive_experiment('EXP-NOMINAL', expected_revision=3)
    assert 'EXP-NOMINAL' in {x['id'] for x in api.list_experiments()}


def test_archived_selection_cannot_start_but_historical_definition_remains_readable(
    api, tmp_path, monkeypatch
):
    """Reject stale deployment selection while preserving historical evidence."""
    monkeypatch.setattr(api.jobs, '_spawn_worker', lambda directory: None)
    original = api.catalog.get('EXP-NOMINAL', 3)
    api.archive_experiment('EXP-NOMINAL', expected_revision=3)
    with pytest.raises(ValueError, match='deleted'):
        api.start_run('EXP-NOMINAL', 3, output=tmp_path / 'results')
    assert api.catalog.get('EXP-NOMINAL', 3) == original
    assert not list(api.jobs.directory.glob('*/request.json'))


@pytest.mark.parametrize('job_status', [[], None, {}, {'state': []}])
def test_malformed_job_status_fails_closed_with_public_error(api, job_status):
    """Keep malformed owner state from bypassing the active Job guard."""
    jid = str(uuid.uuid4())
    path = api.jobs.directory / jid
    path.mkdir(parents=True)
    write_json(path / 'request.json', {'job_id': jid, 'record': {'id': 'EXP-NOMINAL'}})
    write_json(path / 'status.json', job_status)
    with pytest.raises(ValueError, match='Job|cleanup'):
        api.archive_experiment('EXP-NOMINAL', expected_revision=3)
    assert 'EXP-NOMINAL' in {x['id'] for x in api.list_experiments()}


@pytest.mark.parametrize(
    'state,cleanup,blocked',
    [
        ('queued', None, True),
        ('failed', None, True),
        ('completed', None, True),
        ('cancelled', True, False),
    ],
)
def test_legacy_job_with_unknown_session_blocks_deletion_until_safe(
    api, state, cleanup, blocked
):
    """Conservatively protect Sessions before legacy association is provable."""
    sid = api.create_session({'title': 'Possibly associated'})['session_id']
    jid = str(uuid.uuid4())
    path = api.jobs.directory / jid
    path.mkdir(parents=True)
    write_json(path / 'request.json', {'job_id': jid, 'record': {'id': 'EXP-NOMINAL'}})
    status = {'job_id': jid, 'state': state}
    if cleanup is not None:
        status['cleanup_complete'] = cleanup
    write_json(path / 'status.json', status)
    token = api.get_session(sid)['token']
    if blocked:
        with pytest.raises(ValueError, match='Legacy Job Session association is unknown'):
            api.delete_session(sid, expected_token=token)
        assert api.get_session(sid)['token'] == token
    else:
        assert api.delete_session(sid, expected_token=token)


@pytest.mark.parametrize(
    'job_request,operation',
    [
        ({'record': {}}, 'archive'),
        ({'record': {'id': 'not-an-experiment-id'}}, 'archive'),
        ({'record': {'id': 'EXP-NOMINAL'}, 'session_id': 'not-a-uuid'}, 'delete'),
    ],
)
def test_invalid_job_identity_cannot_bypass_management_guard(api, job_request, operation):
    """Validate required owner identities before deciding association."""
    sid = api.create_session({'title': 'Protected'})['session_id']
    jid = str(uuid.uuid4())
    path = api.jobs.directory / jid
    path.mkdir(parents=True)
    write_json(path / 'request.json', {'job_id': jid, **job_request})
    write_json(path / 'status.json', {'job_id': jid, 'state': 'running'})
    before = api.catalog.path.read_bytes()
    token = api.get_session(sid)['token']
    with pytest.raises(ValueError, match='Job association or cleanup is unknown'):
        if operation == 'archive':
            api.archive_experiment('EXP-NOMINAL', expected_revision=3)
        else:
            api.delete_session(sid, expected_token=token)
    assert api.catalog.path.read_bytes() == before
    assert api.get_session(sid)['token'] == token
