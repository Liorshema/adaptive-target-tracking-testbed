"""Destructive backend checks use only isolated, disposable owned workspaces."""

import json
from pathlib import Path

import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.reporting import write_json

from test_backend import recorded_run, share


@pytest.fixture
def api(tmp_path):
    """Initialize a disposable Catalog and explicitly managed Run root."""
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    return ExperimentBackend(tmp_path / 'state', catalog=catalog)


def attach(api, run_id, title):
    """Create a real Session reference through its owner."""
    session = api.create_session({'title': title})
    api.add_run_to_session(session['session_id'], run_id)
    return session['session_id']


def owned(api, experiment=None):
    """Use Runner-shaped fixtures, with real Catalog identity when requested."""
    run_id, directory = recorded_run(api.directory / 'runs')
    if experiment:
        record = api.catalog.get(experiment)
        identity = {
            'id': experiment,
            'revision': record['revision'],
            'definition_sha256': record['sha256'],
        }
        for path in [directory / 'metadata.json', directory / 'status.json']:
            value = json.loads(path.read_text())
            value['experiment_identity'] = identity
            write_json(path, value)
        write_json(directory.parent.parent / 'experiment_identity.json', identity)
    return run_id, directory


def execute(api, kind, identifier):
    """Confirm the exact scope calculated by the authoritative preview."""
    impact = api.preview_deletion(kind, identifier)
    result = api.delete_resources(kind, identifier, expected_token=impact['token'])
    assert result['completed'] is True
    return impact


def test_scenario_purge_prunes_only_matching_pinned_experiment_references(api):
    """Explicit purge cannot leave surviving Sessions with dangling Catalog pins."""
    sid = api.create_session()['session_id']
    for name in ['EXP-NOMINAL', 'EXP-PREDICTION']:
        selected = api.get_session(sid)
        api.add_experiment_to_session(sid, name, 1, expected_token=selected['token'])
    execute(api, 'scenario', 'EXP-NOMINAL')
    assert [r['experiment_id'] for r in api.list_session_experiments(sid)] == ['EXP-PREDICTION']


def test_exclusive_session_removes_real_owned_run(api):
    """Successful deletion actually removes artifacts and its Session JSON."""
    rid, directory = owned(api)
    sid = attach(api, rid, 'Disposable')
    impact = execute(api, 'session', sid)
    assert impact['run_count'] == 1 and impact['shared_run_count'] == 0
    assert not directory.exists() and not api.list_sessions()
    assert not api.list_runs()


def test_shared_run_is_preserved_byte_identically(api):
    """A second Session protects shared evidence while exclusive evidence is deleted."""
    exclusive, path = owned(api)
    shared, keep = owned(api)
    sid = attach(api, exclusive, 'Delete this')
    api.add_run_to_session(sid, shared)
    other = attach(api, shared, 'Keep this')
    before = {p: p.read_bytes() for p in keep.rglob('*') if p.is_file()}
    impact = execute(api, 'session', sid)
    assert impact['run_count'] == 1 and impact['shared_run_count'] == 1
    assert not path.exists()
    assert all(p.read_bytes() == b for p, b in before.items())
    assert len(api.get_session(other)['session']['runs']) == 1


def test_scenario_cascade_updates_all_sessions_without_deleting_unrelated_runs(api):
    """Match immutable experiment identity and prune only matching references."""
    rid, directory = owned(api, 'EXP-NOMINAL')
    keep_id, keep = owned(api, 'EXP-PREDICTION')
    first = attach(api, rid, 'Mixed')
    api.add_run_to_session(first, keep_id)
    second = attach(api, rid, 'Only removed run')
    before = {p: p.read_bytes() for p in keep.rglob('*') if p.is_file()}
    impact = execute(api, 'scenario', 'EXP-NOMINAL')
    assert impact['run_count'] == 1 and impact['session_count'] == 2
    assert not directory.exists()
    assert api.get_session(first)['session']['runs'][0]['run_id'] == keep_id
    assert api.get_session(second)['session']['runs'] == []
    assert all(p.read_bytes() == b for p, b in before.items())
    with pytest.raises(KeyError):
        api.catalog.get('EXP-NOMINAL')
    assert not api.catalog.is_id_available('EXP-NOMINAL')


def test_readonly_registered_root_never_grants_deletion_authority(api, tmp_path):
    """Viewing historical evidence cannot turn it into writable managed data."""
    rid, directory = recorded_run(tmp_path / 'external')
    api.result_roots += (tmp_path / 'external',)
    sid = attach(api, rid, 'External evidence')
    with pytest.raises(ValueError, match='managed'):
        api.preview_deletion('session', sid)
    assert directory.is_dir()


@pytest.mark.parametrize('change', ['reference', 'file', 'catalog'])
def test_scope_change_rejects_stale_confirmation(api, change):
    """References, artifact mutations and Catalog updates invalidate pending deletion."""
    rid, directory = owned(api)
    sid = attach(api, rid, 'Stale')
    impact = api.preview_deletion('session', sid)
    if change == 'reference':
        attach(api, rid, 'New shared owner')
    elif change == 'file':
        (directory / 'new-artifact').write_text('Keep')
    else:
        api.catalog.rename_display(
            'EXP-NOMINAL', 'Changed', expected_revision=3, expected_name='Nominal IMM tracking'
        )
    with pytest.raises(ValueError, match='changed|stale'):
        api.delete_resources('session', sid, expected_token=impact['token'])
    assert directory.exists()


@pytest.mark.parametrize('target', ['root', 'file', 'reference'])
def test_symlink_and_ambiguous_paths_fail_closed(api, tmp_path, target):
    """Reject substituted managed roots, data files and Session reference paths."""
    rid, directory = owned(api)
    sid = attach(api, rid, 'Unsafe')
    if target == 'file':
        (directory / 'escape').symlink_to(tmp_path / 'outside')
    elif target == 'root':
        root = api.directory / 'runs'
        root.rename(api.directory / 'relocated')
        root.symlink_to(api.directory / 'relocated', target_is_directory=True)
    else:
        value = api.get_session(sid)['session']
        value['runs'][0]['directory'] += '/../' + directory.name
        write_json(api.directory / 'sessions' / (sid + '.json'), value)
    with pytest.raises(ValueError, match='symlink|path|owned|managed'):
        api.preview_deletion('session', sid)


def test_staging_failure_restores_all_artifacts_and_metadata(api, monkeypatch):
    """A failed stage rolls back without claiming partial work succeeded."""
    first, one = owned(api)
    second, two = owned(api)
    sid = attach(api, first, 'Rollback')
    api.add_run_to_session(sid, second)
    impact = api.preview_deletion('session', sid)
    before = api.get_session(sid)
    original = Path.rename
    calls = []

    def fail_second(path, destination):
        if '.deletion-' in Path(destination).name:
            calls.append(path)
            if len(calls) == 2:
                raise OSError('injected staging failure')
        return original(path, destination)

    monkeypatch.setattr(Path, 'rename', fail_second)
    with pytest.raises(ValueError, match='incomplete|failed'):
        api.delete_resources('session', sid, expected_token=impact['token'])
    assert one.is_dir() and two.is_dir() and api.get_session(sid) == before


def test_cleanup_failure_can_resume_exact_confirmed_transaction(api, monkeypatch):
    """Post-commit cleanup failure is explicit and retry completes the same scope."""
    import robot_experiments.deletion as deletion

    rid, directory = owned(api)
    sid = attach(api, rid, 'Retry')
    impact = api.preview_deletion('session', sid)
    original = deletion.shutil.rmtree
    monkeypatch.setattr(
        deletion.shutil,
        'rmtree',
        lambda p: (_ for _ in ()).throw(OSError('injected cleanup failure')),
    )
    with pytest.raises(ValueError, match='incomplete|cleanup'):
        api.delete_resources('session', sid, expected_token=impact['token'])
    assert not directory.exists()
    monkeypatch.setattr(deletion.shutil, 'rmtree', original)
    result = api.delete_resources('session', sid, expected_token=impact['token'])
    assert result['completed'] is True
    assert not list(api.directory.rglob('.deletion-*'))


@pytest.mark.parametrize(
    'state,clean,blocked',
    [
        ('running', True, True),
        ('worker_lost', True, True),
        ('failed', False, True),
        ('completed', True, False),
        ('completed', None, False),
        ('cancelled', True, False),
    ],
)
def test_legacy_cleanup_evidence_is_checked(api, state, clean, blocked):
    """Only verified terminal cleanup allows legacy association to be excluded."""
    import uuid

    rid, directory = owned(api)
    sid = attach(api, rid, 'Legacy')
    job = api.jobs.directory / str(uuid.uuid4())
    job.mkdir(parents=True)
    write_json(job / 'request.json', {'record': {'id': 'EXP-NOMINAL'}})
    status = {'state': state, 'study': str(directory.parent.parent)}
    if clean is not None:
        status['cleanup_complete'] = clean
    write_json(job / 'status.json', status)
    if blocked:
        with pytest.raises(ValueError, match='active|cleanup|Job|worker'):
            api.preview_deletion('session', sid)
    else:
        assert execute(api, 'session', sid)['run_count'] == 1


def test_completed_deletion_scrubs_catalog_preimages(api):
    """A completed Scenario purge must not retain deleted definitions in its journal."""
    execute(api, 'scenario', 'EXP-NOMINAL')
    receipts = list((api.directory / '.deletions').glob('*.json'))
    assert len(receipts) == 1
    receipt = json.loads(receipts[0].read_text())
    assert receipt['edits'] == [] and receipt['payloads'] == []


def test_managed_run_subroot_does_not_authorize_parent_study(api):
    """Explicit narrow ownership cannot sweep up study files above that boundary."""
    rid, directory = owned(api)
    study = directory.parent.parent
    api.managed_result_roots = (directory.parent,)
    sid = attach(api, rid, 'Narrow boundary')
    execute(api, 'session', sid)
    assert study.is_dir() and (study / 'experiment_identity.json').exists()
    assert not directory.exists()


def test_verified_legacy_job_stays_verified_after_its_artifacts_are_deleted(api):
    """A compact cleanup receipt prevents deletion itself creating a permanent legacy block."""
    import uuid

    rid, directory = owned(api)
    sid = attach(api, rid, 'Delete verified artifacts')
    job = api.jobs.directory / str(uuid.uuid4())
    job.mkdir(parents=True)
    write_json(job / 'request.json', {'record': {'id': 'EXP-NOMINAL'}})
    write_json(
        job / 'status.json', {'state': 'completed', 'study': str(directory.parent.parent)}
    )
    execute(api, 'session', sid)
    other = api.create_session({'title': 'Unrelated later Session'})['session_id']
    execute(api, 'session', other)


@pytest.mark.parametrize('missing', ['status.json', 'metadata.json'])
def test_scenario_never_omits_incompletely_recorded_run(api, missing):
    """Missing ownership documents block purge rather than hiding artifacts from inventory."""
    rid, directory = owned(api, 'EXP-NOMINAL')
    sid = attach(api, rid, 'Incomplete evidence')
    (directory / missing).unlink()
    with pytest.raises(ValueError, match='ownership|metadata|unreadable'):
        api.preview_deletion('scenario', 'EXP-NOMINAL')
    assert directory.exists() and api.get_session(sid)['session']['runs']
    assert api.catalog.get('EXP-NOMINAL')


def test_job_cleanup_flag_cannot_override_contradictory_run_evidence(api):
    """Terminal Job flags cannot certify a study with surviving physical processes."""
    _, directory = owned(api)
    value = json.loads((directory / 'status.json').read_text())
    value['cleanup']['survivors'] = [{'pid': 1234, 'created': 1.0}]
    write_json(directory / 'status.json', value)
    assert not api._job_cleanup_verified(
        {'state': 'failed', 'cleanup_complete': True, 'study': str(directory.parent.parent)}
    )


def test_concurrent_confirmations_delete_once_and_never_touch_unrelated_data(api):
    """Serialize two identical confirmations without a second destructive scope."""
    from concurrent.futures import ThreadPoolExecutor

    rid, path = owned(api)
    sid = attach(api, rid, 'Concurrent delete')
    impact = api.preview_deletion('session', sid)
    keep_id, keep = owned(api)
    # Add the unrelated Run before taking the actual confirmed inventory token.
    impact = api.preview_deletion('session', sid)
    before = {p: p.read_bytes() for p in keep.rglob('*') if p.is_file()}

    def confirm(_):
        try:
            return api.delete_resources('session', sid, expected_token=impact['token'])[
                'completed'
            ]
        except (KeyError, ValueError):
            return False

    with ThreadPoolExecutor(max_workers=2) as pool:
        assert sorted(pool.map(confirm, range(2))) == [False, True]
    assert not path.exists()
    assert all(p.read_bytes() == b for p, b in before.items())
    assert api.get_run_status(keep_id)


def test_metadata_commit_failure_rolls_back_runs_catalog_and_references(api, monkeypatch):
    """A mid-commit failure cannot leave pruned Sessions pointing to missing data."""
    from robot_experiments.deletion import ResourceDeletion

    rid, directory = owned(api, 'EXP-NOMINAL')
    sid = attach(api, rid, 'Rollback commit')
    impact = api.preview_deletion('scenario', 'EXP-NOMINAL')
    before_session = api.get_session(sid)
    before_catalog = api.catalog.path.read_bytes()
    original = ResourceDeletion._apply

    def fail_catalog(self, edit, field):
        if edit['kind'] == 'catalog' and field == 'after':
            raise OSError('injected metadata failure')
        return original(self, edit, field)

    monkeypatch.setattr(ResourceDeletion, '_apply', fail_catalog)
    with pytest.raises(ValueError, match='failed|incomplete'):
        api.delete_resources('scenario', 'EXP-NOMINAL', expected_token=impact['token'])
    assert directory.exists() and api.get_session(sid) == before_session
    assert api.catalog.path.read_bytes() == before_catalog


def test_scenario_blocks_unowned_study_files_instead_of_leaving_scientific_summaries(api):
    """A purge cannot report success while retaining selected study metrics unsafely."""
    _, directory = owned(api, 'EXP-NOMINAL')
    study = directory.parent.parent
    (study / 'summary.json').write_text('{"recorded_results": true}')
    (study / 'unowned.txt').write_text('Protected extra file')
    with pytest.raises(ValueError, match='unowned|shared|study'):
        api.preview_deletion('scenario', 'EXP-NOMINAL')
    assert (study / 'unowned.txt').read_text() == 'Protected extra file'
    assert directory.exists() and api.catalog.get('EXP-NOMINAL')


def test_scenario_deletes_owned_empty_study_and_its_stored_definition(api):
    """Even a study with no Run yet must not leave owned definitions behind on purge."""
    _, directory = owned(api, 'EXP-NOMINAL')
    import shutil

    study = directory.parent.parent
    shutil.rmtree(directory)
    (study / 'experiment.yaml').write_text('disposable study definition')
    impact = execute(api, 'scenario', 'EXP-NOMINAL')
    assert impact['run_count'] == 0
    assert not study.exists()


def test_empty_study_requires_explicit_immutable_revision(api):
    """A missing revision cannot silently resolve to the latest Catalog definition."""
    import shutil

    _, directory = owned(api, 'EXP-NOMINAL')
    study = directory.parent.parent
    shutil.rmtree(directory)
    value = json.loads((study / 'experiment_identity.json').read_text())
    value['revision'] = None
    write_json(study / 'experiment_identity.json', value)
    with pytest.raises(ValueError, match='identity|revision'):
        api.preview_deletion('scenario', 'EXP-NOMINAL')
    assert study.exists()


@pytest.mark.parametrize('phase', ['staging', 'cleanup'])
def test_abrupt_interruption_resumes_only_the_confirmed_scope(api, monkeypatch, phase):
    """Persisted transaction state protects data across pre/postcommit interruption."""
    from robot_experiments.deletion import ResourceDeletion

    rid, directory = owned(api)
    sid = attach(api, rid, 'Interrupted deletion')
    pending = api.preview_deletion('session', sid)
    if phase == 'staging':
        original = Path.rename

        def interrupt_after_move(path, destination):
            result = original(path, destination)
            if '.deletion-' in Path(destination).name:
                raise KeyboardInterrupt()
            return result

        monkeypatch.setattr(Path, 'rename', interrupt_after_move)
    else:
        original = ResourceDeletion._cleanup
        monkeypatch.setattr(
            ResourceDeletion,
            '_cleanup',
            lambda self, journal: (_ for _ in ()).throw(KeyboardInterrupt()),
        )
    with pytest.raises(KeyboardInterrupt):
        api.delete_resources('session', sid, expected_token=pending['token'])
    with pytest.raises(ValueError, match='incomplete'):
        api.create_session({'title': 'Must wait for recovery'})
    with pytest.raises(ValueError, match='incomplete'):
        api.save_experiment(api.new_draft('EXP-WAIT', 'Wait for recovery', 'EXP-NOMINAL'))
    assert api.preview_deletion('session', sid)['token'] == pending['token']
    if phase == 'staging':
        monkeypatch.setattr(Path, 'rename', original)
    else:
        monkeypatch.setattr(ResourceDeletion, '_cleanup', original)
    assert api.delete_resources('session', sid, expected_token=pending['token'])['completed']
    assert not directory.exists() and not list(api.directory.rglob('.deletion-*'))


@pytest.mark.parametrize('roots', ['/tmp/not-a-sequence', Path('/tmp'), ['/']])
def test_managed_root_configuration_cannot_accidentally_grant_filesystem_wide_access(
    api, roots
):
    """Reject scalar path iteration and filesystem-root deletion authority eagerly."""
    with pytest.raises(ValueError, match='managed|root|sequence'):
        ExperimentBackend(api.directory, catalog=api.catalog, managed_result_roots=roots)


def test_user_study_names_are_not_confused_with_transaction_staging(api):
    """A descriptive user name sharing the prefix remains discoverable and deletable."""
    rid, directory = owned(api)
    study = directory.parent.parent
    renamed = study.with_name('.deletion-user-study')
    study.rename(renamed)
    rid_path = renamed / 'runs' / directory.name
    sid = attach(api, rid, 'User named study')
    assert api.get_run_status(rid)
    assert execute(api, 'session', sid)['run_count'] == 1
    assert not rid_path.exists()


def test_orphaned_staging_blocks_purge_instead_of_hiding_owned_data(api):
    """Missing transaction provenance cannot make staged artifacts disappear from scope."""
    import uuid

    _, directory = owned(api, 'EXP-NOMINAL')
    study = directory.parent.parent
    staged = study.with_name('.deletion-' + str(uuid.uuid4()) + '-0')
    study.rename(staged)
    with pytest.raises(ValueError, match='staging|recovery|transaction'):
        api.preview_deletion('scenario', 'EXP-NOMINAL')
    assert staged.exists() and api.catalog.get('EXP-NOMINAL')


ROS_LOG_SESSION = '2026-11-01-06-30-20-857511-DESKTOP-VRFH3M9-143450'


def ros_log(directory, *, relative=False):
    """Mirror retained ROS2 logs without starting ROS, Jobs or a simulation."""
    log = directory / 'ros-log' / ROS_LOG_SESSION
    log.mkdir(parents=True)
    (log / 'launch.log').write_text('Disposable ROS2 launch log\n')
    link = log.parent / 'latest'
    link.symlink_to(log.name if relative else log, target_is_directory=True)
    return link, log


@pytest.mark.parametrize('relative', [False, True])
@pytest.mark.parametrize('payload', ['study', 'run'])
def test_owned_session_accepts_only_internal_ros_latest_and_preserves_shared_run(
    api, relative, payload
):
    """Normal ROS log aliases must not prevent an exclusive Run's deletion."""
    rid, directory = owned(api)
    link, _ = ros_log(directory, relative=relative)
    keep_id, keep = owned(api)
    keep_link, keep_log = ros_log(keep)
    sid = attach(api, rid, 'ROS logs')
    api.add_run_to_session(sid, keep_id)
    other = attach(api, keep_id, 'Shared retained logs')
    if payload == 'run':
        api.managed_result_roots = (directory.parent,)
    before = (keep_link.readlink(), (keep_log / 'launch.log').read_bytes())
    impact = execute(api, 'session', sid)
    assert impact['run_count'] == 1 and impact['shared_run_count'] == 1
    assert not directory.exists() and not link.is_symlink()
    assert keep_link.readlink() == before[0]
    assert (keep_log / 'launch.log').read_bytes() == before[1]
    assert api.get_session(other)['session']['runs'][0]['run_id'] == keep_id


@pytest.mark.parametrize('relative', [False, True])
def test_scenario_deletes_ros_log_study_and_prunes_sessions(api, relative):
    """The complete verified study remains deletable with Runner's log layout."""
    rid, directory = owned(api, 'EXP-NOMINAL')
    ros_log(directory, relative=relative)
    sid = attach(api, rid, 'Scenario logs')
    keep_id, keep = owned(api, 'EXP-PREDICTION')
    keep_link, keep_log = ros_log(keep)
    api.add_run_to_session(sid, keep_id)
    before = (keep_link.readlink(), (keep_log / 'launch.log').read_bytes())
    assert execute(api, 'scenario', 'EXP-NOMINAL')['run_count'] == 1
    assert not directory.parent.parent.exists()
    assert api.get_session(sid)['session']['runs'][0]['run_id'] == keep_id
    assert keep_link.readlink() == before[0]
    assert (keep_log / 'launch.log').read_bytes() == before[1]
    with pytest.raises(KeyError):
        api.catalog.get('EXP-NOMINAL')


@pytest.mark.parametrize('kind', ['session', 'scenario'])
@pytest.mark.parametrize('unsafe', [
    'external', 'other_run', 'dangling', 'file', 'chain', 'wrong_name',
    'misplaced', 'parent_alias', 'unexpected_destination', 'nested_escape',
])
def test_ros_log_symlink_exception_does_not_authorize_unsafe_links(api, tmp_path, kind, unsafe):
    """A familiar link name grants no authority over another path or symlink chain."""
    rid, directory = owned(api, 'EXP-NOMINAL')
    link, log = ros_log(directory)
    external = tmp_path / 'protected' / ROS_LOG_SESSION
    external.mkdir(parents=True)
    sentinel = external / 'keep.txt'
    sentinel.write_text('External evidence stays intact')
    link.unlink()
    target = log
    if unsafe == 'external':
        target = external
    elif unsafe == 'other_run':
        _, other = owned(api, 'EXP-PREDICTION')
        _, target = ros_log(other)
    elif unsafe == 'dangling':
        target = log.with_name(ROS_LOG_SESSION.replace('143450', '143451'))
    elif unsafe == 'file':
        (log / 'launch.log').rename(log.parent / 'log-sentinel')
        log.rmdir()
        log.write_text('A regular file is not a launch directory')
    elif unsafe == 'chain':
        (log / 'launch.log').unlink()
        log.rmdir()
        log.symlink_to(external, target_is_directory=True)
    elif unsafe == 'wrong_name':
        link = link.with_name('unexpected')
    elif unsafe == 'misplaced':
        link = directory / 'latest'
    elif unsafe == 'parent_alias':
        target = str(log.parent) + '/nested/../' + log.name
        (log.parent / 'nested').mkdir()
    elif unsafe == 'unexpected_destination':
        target = log.with_name('unrecognized-directory')
        log.rename(target)
    else:
        (log / 'escape').symlink_to(external, target_is_directory=True)
    link.symlink_to(target, target_is_directory=True)
    sid = attach(api, rid, 'Protected logs')
    before_catalog = api.catalog.path.read_bytes()
    before_session = api.get_session(sid)
    with pytest.raises(ValueError, match='symlink|Symlink|owned|log|path'):
        api.preview_deletion(kind, sid if kind == 'session' else 'EXP-NOMINAL')
    assert directory.is_dir() and sentinel.read_text() == 'External evidence stays intact'
    assert api.catalog.path.read_bytes() == before_catalog
    assert api.get_session(sid) == before_session
    assert not list((api.directory / '.deletions').glob('*.json'))


def test_retargeted_ros_log_link_invalidates_confirmation(api):
    """A new safe destination still requires a new destructive confirmation."""
    rid, directory = owned(api)
    link, log = ros_log(directory)
    other = log.with_name(ROS_LOG_SESSION.replace('143450', '143451'))
    other.mkdir()
    sid = attach(api, rid, 'Retargeted logs')
    impact = api.preview_deletion('session', sid)
    link.unlink()
    link.symlink_to(other)
    with pytest.raises(ValueError, match='changed|stale'):
        api.delete_resources('session', sid, expected_token=impact['token'])
    assert directory.is_dir() and api.get_session(sid)['session']['runs']


@pytest.mark.parametrize('phase', ['staging', 'cleanup'])
@pytest.mark.parametrize('payload', ['study', 'run'])
def test_absolute_ros_log_links_survive_interruption_and_exact_scope_recovery(
    api, monkeypatch, phase, payload
):
    """Staging must remap absolute log targets without following their old paths."""
    from robot_experiments.deletion import ResourceDeletion

    rid, directory = owned(api)
    link, log = ros_log(directory)
    sid = attach(api, rid, 'Interrupted ROS logs')
    if payload == 'run':
        api.managed_result_roots = (directory.parent,)
    impact = api.preview_deletion('session', sid)
    if phase == 'staging':
        original = Path.rename

        def interrupt_after_move(path, destination):
            result = original(path, destination)
            if '.deletion-' in Path(destination).name:
                raise KeyboardInterrupt()
            return result

        monkeypatch.setattr(Path, 'rename', interrupt_after_move)
    else:
        original = ResourceDeletion._cleanup
        monkeypatch.setattr(
            ResourceDeletion, '_cleanup',
            lambda self, journal: (_ for _ in ()).throw(KeyboardInterrupt()),
        )
    with pytest.raises(KeyboardInterrupt):
        api.delete_resources('session', sid, expected_token=impact['token'])
    assert not directory.exists()
    staged_links = list((api.directory / 'runs').rglob('latest'))
    assert len(staged_links) == 1 and staged_links[0].readlink() == log
    assert not staged_links[0].exists()  # Absolute link is dangling only because of staging.
    with pytest.raises(ValueError, match='incomplete'):
        api.create_session({'title': 'Wait for ROS log recovery'})
    with pytest.raises(ValueError, match='confirmation|incomplete'):
        api.delete_resources('session', sid, expected_token='wrong-token')
    if phase == 'staging':
        monkeypatch.setattr(Path, 'rename', original)
    else:
        monkeypatch.setattr(ResourceDeletion, '_cleanup', original)
    # A fresh service instance uses only the persisted journal and owner context.
    reopened = ExperimentBackend(
        api.directory, catalog=api.catalog, managed_result_roots=api.managed_result_roots,
    )
    assert reopened.preview_deletion('session', sid)['token'] == impact['token']
    assert reopened.delete_resources('session', sid, expected_token=impact['token'])['completed']
    assert not directory.exists() and not link.is_symlink()
    assert not list(api.directory.rglob('.deletion-*'))


def test_recovery_blocks_even_safe_log_retargeting_after_confirmation(api, monkeypatch):
    """Pending cleanup cannot silently broaden the confirmed alias destination."""
    from robot_experiments.deletion import ResourceDeletion

    rid, directory = owned(api)
    _, log = ros_log(directory)
    other = log.with_name(ROS_LOG_SESSION.replace('143450', '143451'))
    other.mkdir()
    sid = attach(api, rid, 'Protected recovery logs')
    impact = api.preview_deletion('session', sid)
    original = ResourceDeletion._cleanup
    monkeypatch.setattr(
        ResourceDeletion, '_cleanup',
        lambda self, journal: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    with pytest.raises(KeyboardInterrupt):
        api.delete_resources('session', sid, expected_token=impact['token'])
    monkeypatch.setattr(ResourceDeletion, '_cleanup', original)
    staged_link = next((api.directory / 'runs').rglob('latest'))
    staged_link.unlink()
    staged_link.symlink_to(other)
    with pytest.raises(ValueError, match='incomplete|cleanup'):
        api.delete_resources('session', sid, expected_token=impact['token'])
    assert staged_link.is_symlink()
    staged_link.unlink()
    staged_link.symlink_to(log)
    assert api.delete_resources('session', sid, expected_token=impact['token'])['completed']


def test_staged_absolute_log_alias_never_traverses_reoccupied_original_path(api, monkeypatch):
    """Cleanup uses staged directory entries even if an old absolute target exists again."""
    from robot_experiments.deletion import ResourceDeletion

    rid, directory = owned(api)
    _, log = ros_log(directory)
    sid = attach(api, rid, 'Staged alias')
    impact = api.preview_deletion('session', sid)
    original = ResourceDeletion._cleanup
    monkeypatch.setattr(
        ResourceDeletion, '_cleanup',
        lambda self, journal: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    with pytest.raises(KeyboardInterrupt):
        api.delete_resources('session', sid, expected_token=impact['token'])
    monkeypatch.setattr(ResourceDeletion, '_cleanup', original)
    log.mkdir(parents=True)
    sentinel = log / 'new-unrelated-evidence'
    sentinel.write_text('Preserve the reoccupied path')
    assert api.delete_resources('session', sid, expected_token=impact['token'])['completed']
    assert sentinel.read_text() == 'Preserve the reoccupied path'
    assert not list(api.directory.rglob('.deletion-*'))


@pytest.mark.parametrize('source', ['registration', 'job'])
def test_standard_ros_log_links_do_not_grant_external_root_authority(api, tmp_path, source):
    """Log compatibility cannot promote viewed or Job-discovered roots to managed owners."""
    import uuid

    rid, directory = recorded_run(tmp_path / 'external')
    link, log = ros_log(directory)
    if source == 'registration':
        api.result_roots += (tmp_path / 'external',)
    else:
        job = api.jobs.directory / str(uuid.uuid4())
        job.mkdir(parents=True)
        write_json(job / 'request.json', {
            'record': {'id': 'EXP-RECORDED'}, 'output': str(tmp_path / 'external'),
        })
        write_json(job / 'status.json', {
            'state': 'completed', 'study': str(directory.parent.parent),
        })
    sid = attach(api, rid, 'Read-only ROS logs')
    with pytest.raises(ValueError, match='managed'):
        api.preview_deletion('session', sid)
    assert directory.is_dir() and link.readlink() == log
    assert (log / 'launch.log').read_text() == 'Disposable ROS2 launch log\n'


@pytest.mark.parametrize('relative', [False, True])
@pytest.mark.parametrize('replacement', ['removed', 'file', 'symlink'])
def test_partial_ros_log_cleanup_resumes_only_missing_confirmed_target(
    api, tmp_path, monkeypatch, relative, replacement
):
    """Retry after partial rmtree tolerates removal, while refusing substituted targets."""
    import robot_experiments.deletion as deletion

    rid, directory = owned(api)
    _, log = ros_log(directory, relative=relative)
    sid = attach(api, rid, 'Partial ROS log cleanup')
    impact = api.preview_deletion('session', sid)
    original = deletion.shutil.rmtree
    protected = tmp_path / 'protected-log-target'
    protected.mkdir()
    sentinel = protected / 'keep.txt'
    sentinel.write_text('Keep external evidence')
    targets = []

    def partial_cleanup(staged):
        target = next(staged.rglob(log.name))
        original(target)
        targets.append(target)
        if replacement == 'file':
            target.write_text('Unexpected replacement file')
        elif replacement == 'symlink':
            target.symlink_to(protected, target_is_directory=True)
        raise OSError('injected interruption after deleting ROS log directory')

    monkeypatch.setattr(deletion.shutil, 'rmtree', partial_cleanup)
    with pytest.raises(ValueError, match='incomplete|cleanup'):
        api.delete_resources('session', sid, expected_token=impact['token'])
    monkeypatch.setattr(deletion.shutil, 'rmtree', original)
    if replacement != 'removed':
        with pytest.raises(ValueError, match='incomplete|cleanup'):
            api.delete_resources('session', sid, expected_token=impact['token'])
        assert targets[0].is_symlink() or targets[0].is_file()
        targets[0].unlink()
    result = api.delete_resources('session', sid, expected_token=impact['token'])
    assert result['completed']
    assert not list(api.directory.rglob('.deletion-*'))
    assert sentinel.read_text() == 'Keep external evidence'
