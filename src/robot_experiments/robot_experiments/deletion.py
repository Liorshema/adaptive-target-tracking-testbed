"""Confirmed multi-resource deletion inside the existing backend ownership boundary.

A durable undo journal covers staging and metadata commit. Once committed, only
cleanup is retried. No arbitrary path supplied by a client is ever a delete target.
"""

import hashlib
import json
import os
import re
import shutil
import stat
import uuid
from contextlib import ExitStack, contextmanager
from copy import deepcopy
from pathlib import Path

from .catalog import CatalogConflict
from .reporting import write_json


_STAGED_NAME = re.compile(r'\.deletion-[a-f0-9]{8}(?:-[a-f0-9]{4}){3}-[a-f0-9]{12}-\d+')
_ROS_LOG_SESSION = re.compile(
    r'\d{4}(?:-\d{2}){5}-\d{6}-[A-Za-z0-9_.-]+-[1-9]\d*'
)


def _staged_path(path):
    """Recognize only the reserved transaction format, preserving ordinary user names."""
    return any(_STAGED_NAME.fullmatch(part) for part in Path(path).parts)


def _token(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def _plain_path(path):
    """Reject symlinks and lexical aliases, including ancestors and dangling links."""
    path = Path(path)
    if not path.is_absolute() or path != Path(os.path.abspath(path)):
        raise ValueError('Unexpected artifact path; canonical absolute paths are required')
    for item in [path, *path.parents]:
        if item.is_symlink():
            raise ValueError('Symlink artifact paths are not owned deletion targets')
    return path


def _tree(path, *, runs=(), original=None, log_links=None):
    """Inventory without following links; only verified Run ROS2 aliases are allowed.

    Absolute ROS2 links still name the original tree after transaction staging.
    Validate their lexical destination against that name, then inspect the actual
    staged directory directly. Never resolve or traverse the symlink itself.
    """
    path = _plain_path(path)
    original = _plain_path(original) if original is not None else path
    allowed_links = {
        Path(run) / 'ros-log' / 'latest'
        for run in runs if Path(run).is_relative_to(original)
    }
    result = []
    for item in [path, *sorted(path.rglob('*'))]:
        info = item.lstat()
        target = None
        if stat.S_ISLNK(info.st_mode):
            logical = original / item.relative_to(path)
            target = os.readlink(item)
            if log_links is not None and log_links.get(str(item.relative_to(path))) != target:
                raise ValueError('ROS log symlink changed after deletion confirmation')
            destination = Path(target)
            expected = logical.parent / destination.name
            if (
                logical not in allowed_links
                or not _ROS_LOG_SESSION.fullmatch(destination.name)
                or target not in {str(expected), destination.name}
            ):
                raise ValueError('Unexpected or escaping ROS log symlink; artifacts protected')
            actual = _plain_path(path / expected.relative_to(original))
            try:
                target_info = actual.lstat()
            except FileNotFoundError:
                # A prior rmtree may remove this directory before reaching latest.
                # Only staged cleanup with its unchanged confirmed alias can retry
                # that partial removal; initial discovery still rejects dangling links.
                if log_links is None or original == path or not _staged_path(path):
                    raise ValueError('ROS log symlink target is missing') from None
            else:
                if not stat.S_ISDIR(target_info.st_mode):
                    raise ValueError('ROS log symlink target is not a directory')
                if target_info.st_dev != path.stat().st_dev:
                    raise ValueError('Cross-filesystem ROS log targets are not owned')
        elif not (stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode)):
            raise ValueError('Owned artifacts must be regular files/directories; no symlinks')
        if info.st_dev != path.stat().st_dev:
            raise ValueError('Cross-filesystem artifact mounts are not owned deletion targets')
        result.append(
            [
                str(item.relative_to(path)),
                info.st_dev,
                info.st_ino,
                info.st_size,
                info.st_mtime_ns,
                *([target] if target is not None else []),
            ]
        )
    return result


def _sync(directory):
    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class ResourceDeletion:
    """Compute and commit one exact Session/Scenario cascade under owner locks."""

    def __init__(self, backend):
        """Use existing owners and explicitly managed writable roots."""
        self.api = backend
        self.journals = backend.directory / '.deletions'

    @contextmanager
    def locked(self):
        """Use one lock order for start, preview, Catalog and all Session mutation."""
        _plain_path(self.journals)
        _plain_path(self.api.jobs.directory / 'simulation.lock')
        _plain_path(self.api.catalog.path.with_suffix(self.api.catalog.path.suffix + '.lock'))
        with self.api._management_lock(allow_pending=True), self.api.jobs._preview_lock():
            with self.api.catalog._locked(), ExitStack() as stack:
                sessions = self.api.directory / 'sessions'
                _plain_path(sessions)
                for path in sorted(sessions.glob('*.json')):
                    self.api._session_path(path.stem)
                    stack.enter_context(self.api._metadata_lock(path.with_suffix('.lock')))
                yield

    def _owned(self, path):
        path = _plain_path(path)
        if not self._within_managed(path):
            raise ValueError(
                'Run is outside managed writable result roots; '
                f'historical evidence preserved: {path}'
            )
        if path.parent.name != 'runs' or not path.is_dir():
            raise ValueError('Unexpected managed Run layout or missing artifact directory')
        return path

    def _within_managed(self, path):
        return any(
            path != root and path.is_relative_to(root)
            for root in [_plain_path(r) for r in self.api.managed_result_roots]
        )

    def _run(self, path):
        path = _plain_path(path)
        status_value = self.api._read(path / 'status.json')
        metadata = self.api._read(path / 'metadata.json')
        if not isinstance(status_value, dict) or not isinstance(metadata, dict):
            raise ValueError('Run ownership metadata is missing or unreadable')
        run_id = self.api._identifier(status_value.get('run_id'))
        identity = status_value.get('experiment_identity')
        if (
            metadata.get('run_id') != run_id
            or not isinstance(identity, dict)
            or metadata.get('experiment_identity') != identity
            or self.api._read(path.parent.parent / 'experiment_identity.json') != identity
        ):
            raise ValueError('Ambiguous immutable Run ownership; identity documents disagree')
        self.api.catalog._identifier(identity.get('id'))
        if (
            type(identity.get('revision')) is not int
            or identity['revision'] < 1
            or not isinstance(identity.get('definition_sha256'), str)
            or len(identity['definition_sha256']) != 64
        ):
            raise ValueError('Invalid immutable experiment identity')
        if status_value.get('execution_status') not in {'completed', 'failed', 'cancelled'}:
            raise ValueError('Run is active or completion is unknown')
        cleanup = status_value.get('cleanup')
        if not isinstance(cleanup, dict) or cleanup.get('survivors') != []:
            raise ValueError('Run cleanup is not verified; artifacts remain protected')
        return {'run_id': run_id, 'directory': str(path), 'identity': identity}

    def _scope(self, kind, identifier):
        if kind not in {'session', 'scenario'}:
            raise ValueError('Expected Session or Scenario deletion')
        api = self.api
        # Valid pending transactions are handled before scope discovery. Any
        # remaining reserved staging has unknown recovery provenance, so it must
        # not make owned evidence invisible to a Scenario purge.
        for root in api._roots():
            for path in root.rglob('.deletion-*'):
                if _staged_path(path):
                    raise ValueError(
                        f'Unresolved transaction staging needs recovery before deletion: {path}')
        catalog = api.catalog._read()
        sessions = {
            p.stem: api._session(p.stem)
            for p in sorted((api.directory / 'sessions').glob('*.json'))
        }
        selected = sessions.get(identifier) if kind == 'session' else None
        if kind == 'session' and selected is None:
            raise KeyError('Selected Session no longer exists')
        if kind == 'scenario':
            api.catalog._identifier(identifier)
            entry = catalog['experiments'][identifier]
            name = entry.get('display_name', entry['name'])
        else:
            name = selected['metadata'].get('title', 'Untitled Session')
        all_runs, paths = {}, {}
        candidates = set(api._run_paths())
        associated_studies = set()
        # A partial recording must not disappear from the purge inventory merely
        # because its status file is missing. Study identity and Run metadata are
        # independent discovery anchors; neither grants ownership on its own.
        if kind == 'scenario':
            for root in api._roots():
                candidates.update(
                    p.parent
                    for p in root.rglob('runs/*/metadata.json')
                    if not _staged_path(p)
                )
                for anchor in root.rglob('experiment_identity.json'):
                    if _staged_path(anchor):
                        continue
                    identity = api._read(anchor)
                    if isinstance(identity, dict) and identity.get('id') == identifier:
                        directory = anchor.parent / 'runs'
                        _plain_path(directory)
                        if not directory.is_dir():
                            raise ValueError(
                                'Study Run inventory is missing; ownership is incomplete'
                            )
                        associated_studies.add(anchor.parent)
                        candidates.update(directory.iterdir())
        for path in sorted(candidates):
            _plain_path(path)
            status_value = api._read(path / 'status.json')
            metadata = api._read(path / 'metadata.json')
            if not isinstance(status_value, dict) or not isinstance(metadata, dict):
                raise ValueError('Recorded Run ownership cannot be established')
            rid = status_value.get('run_id')
            paths.setdefault(rid, []).append(path)
            all_runs[str(path)] = (status_value, metadata)
        targets, shared = [], []
        if kind == 'session':
            for ref in selected['runs']:
                path = _plain_path(ref['directory'])
                if len(paths.get(ref['run_id'], [])) != 1 or paths[ref['run_id']][0] != path:
                    raise ValueError('Ambiguous or missing Run reference path')
                record = self._run(path)
                others = [
                    s
                    for key, s in sessions.items()
                    if key != identifier
                    and any(r['run_id'] == ref['run_id'] for r in s['runs'])
                ]
                (shared if others else targets).append(record)
        else:
            for path, (status_value, metadata) in all_runs.items():
                identities = [
                    status_value.get('experiment_identity'),
                    metadata.get('experiment_identity'),
                ]
                if not any(
                    isinstance(i, dict) and i.get('id') == identifier for i in identities
                ):
                    continue
                record = self._run(Path(path))
                if len(paths[record['run_id']]) != 1:
                    raise ValueError('Ambiguous duplicate Run identity')
                identity = record['identity']
                original = api.catalog.get(identifier, identity['revision'])
                if original['sha256'] != identity['definition_sha256']:
                    raise ValueError(
                        'Run experiment hash disagrees with the authoritative Catalog'
                    )
                targets.append(record)
        target_ids = {r['run_id'] for r in targets}
        known = {r['run_id']: r for r in [*targets, *shared]}
        for session in sessions.values():
            for ref in session['runs']:
                path = _plain_path(ref['directory'])
                if ref['run_id'] in known and str(path) != known[ref['run_id']]['directory']:
                    raise ValueError('Conflicting Session paths for the same Run identity')
        # All association and cleanup checks run under the existing Jobs lock.
        api._assert_no_associated_job(
            session=selected, experiment_id=identifier if kind == 'scenario' else None
        )
        for record in targets:
            self._owned(record['directory'])
        edits, affected = [], []
        for key, session in sessions.items():
            updated = deepcopy(session)
            updated['runs'] = [r for r in session['runs'] if r['run_id'] not in target_ids]
            if kind == 'scenario' and 'experiments' in updated:
                updated['experiments'] = [r for r in updated['experiments']
                                          if r['experiment_id'] != identifier]
            if kind == 'session' and key == identifier:
                updated = None
            if updated != session:
                affected.append(key)
                edits.append(
                    {
                        'kind': 'session',
                        'path': str(api._session_path(key)),
                        'before': session,
                        'after': updated,
                    }
                )
        if kind == 'scenario':
            after = deepcopy(catalog)
            del after['experiments'][identifier]
            after['retired_ids'].append(identifier)
            after['generation'] += 1
            edits.append(
                {
                    'kind': 'catalog',
                    'path': str(api.catalog.path),
                    'before': catalog,
                    'after': after,
                }
            )
        payloads, notes = [], []
        by_study = {}
        for record in targets:
            by_study.setdefault(Path(record['directory']).parent.parent, []).append(record)
        for study in associated_studies:
            by_study.setdefault(study, [])
        for study, records in by_study.items():
            children = list((study / 'runs').iterdir())
            selected_paths = {Path(r['directory']) for r in records}
            identity = self.api._read(study / 'experiment_identity.json')
            if not records:
                if type(identity.get('revision')) is not int or identity['revision'] < 1:
                    raise ValueError('Empty study identity requires an explicit positive revision')
                original = api.catalog.get(identifier, identity.get('revision'))
                if original['sha256'] != identity.get('definition_sha256'):
                    raise ValueError(
                        'Empty study identity disagrees with the authoritative Catalog'
                    )
            if set(children) == selected_paths and (
                not records or identity == records[0]['identity']
            ):
                # A wholly selected study is owned by the same immutable experiment.
                # Unknown extra files cannot be silently swept up in its deletion.
                allowed = {
                    'runs',
                    'experiment_identity.json',
                    'experiment.yaml',
                    'resolved_experiment.json',
                    'summary.json',
                    'comparison.csv',
                }
                if {p.name for p in study.iterdir()} <= allowed and self._within_managed(
                    study
                ):
                    _plain_path(study)
                    payloads.append(study)
                    continue
            if kind == 'scenario':
                raise ValueError(
                    'Scenario study contains shared, unowned or unmanaged artifacts; '
                    f'complete safe deletion is blocked: {study}'
                )
            payloads.extend(Path(r['directory']) for r in records)
            if (study / 'summary.json').exists() or (study / 'comparison.csv').exists():
                notes.append(
                    'Shared study-level summaries are preserved for '
                    'remaining or unowned artifacts.'
                )
        run_paths = [Path(record['directory']) for record in targets]
        manifests = {str(path): _tree(path, runs=run_paths) for path in payloads}
        impact = {
            'kind': kind,
            'id': identifier,
            'name': name,
            'run_count': len(targets),
            'shared_run_count': len(shared),
            'session_count': len(affected),
            'run_ids': sorted(target_ids),
            'notes': list(dict.fromkeys(notes)),
        }
        fingerprint = {
            'impact': impact,
            'sessions': sessions,
            'catalog': catalog,
            'manifests': manifests,
            'managed_roots': list(map(str, api.managed_result_roots)),
        }
        impact['token'] = _token(fingerprint)
        payloads = [
            {
                'original': str(path),
                'runs': [str(run) for run in run_paths if run.is_relative_to(path)],
                'log_links': {
                    row[0]: row[5] for row in manifests[str(path)] if len(row) == 6
                },
            }
            for path in payloads
        ]
        return impact, edits, payloads

    def _write(self, path, document):
        self.journals.mkdir(parents=True, exist_ok=True)
        write_json(path, document)
        _sync(path.parent)
        _sync(path.parent.parent)

    def _apply(self, edit, field):
        value = edit[field]
        path = _plain_path(edit['path'])
        if edit['kind'] == 'catalog':
            if path != self.api.catalog.path:
                raise ValueError('Deletion journal has unexpected Catalog owner')
            self.api.catalog._write(value)
        else:
            if path != self.api._session_path(path.stem):
                raise ValueError('Deletion journal has unexpected Session owner')
            if value is None:
                path.unlink(missing_ok=True)
            else:
                write_json(path, value)
            _sync(path.parent)

    def _rollback(self, journal):
        for edit in reversed(journal['edits']):
            current = (
                self.api.catalog._read()
                if edit['kind'] == 'catalog'
                else self.api._read(edit['path'])
            )
            if current not in (edit['before'], edit['after']):
                raise ValueError('Metadata changed outside deletion; manual recovery required')
            self._apply(edit, 'before')
        for item in reversed(journal['payloads']):
            original, staged = _plain_path(item['original']), _plain_path(item['staged'])
            if staged.exists():
                if original.exists():
                    raise ValueError(
                        'Deletion rollback path is occupied; manual recovery required'
                    )
                staged.rename(original)
                _sync(original.parent)
        journal['phase'] = 'rolled_back'

    def _cleanup(self, journal):
        for item in journal['payloads']:
            staged = _plain_path(item['staged'])
            if staged.exists():
                _tree(
                    staged, runs=item.get('runs', ()), original=item['original'],
                    log_links=item.get('log_links', {}),
                )
                shutil.rmtree(staged)
                _sync(staged.parent)
        journal['phase'] = 'completed'

    @staticmethod
    def _scrub(journal):
        # The durable receipt is not a second store of deleted definitions or
        # Session metadata. Undo images are necessary only before completion.
        if journal['phase'] in {'completed', 'rolled_back'}:
            journal['edits'] = []
            journal['payloads'] = []

    def _verified_jobs(self):
        verified = {}
        for path in self.api.jobs.directory.glob('*/request.json'):
            status_value = self.api.jobs.status(path.parent.name)
            if self.api._job_cleanup_verified(status_value):
                verified[path.parent.name] = {
                    'status': self.api._session_token(status_value),
                    'request': self.api._session_token(self.api._read(path)),
                }
        return verified

    def _validate_journal(self, path, journal):
        self.api._identifier(path.stem)
        for i, item in enumerate(journal['payloads']):
            original, staged = _plain_path(item['original']), _plain_path(item['staged'])
            expected = original.with_name('.deletion-' + path.stem + '-' + str(i))
            if staged != expected or not self._within_managed(original):
                raise ValueError(
                    'Deletion recovery path is outside its verified managed owner'
                )
            for run in item.get('runs', ()):
                run = _plain_path(run)
                if run != original and (
                    run.parent.parent != original or run.parent.name != 'runs'
                ):
                    raise ValueError('Deletion recovery has an unexpected owned Run path')
                if run.parent.name != 'runs':
                    raise ValueError('Deletion recovery has an unexpected owned Run layout')

    def _pending(self):
        for path in sorted(self.journals.glob('*.json')):
            _plain_path(path)
            journal = self.api._read(path)
            if journal['phase'] not in {'completed', 'rolled_back'}:
                self._validate_journal(path, journal)
                yield path, journal

    def preview(self, kind, identifier):
        """Return the current scope or the exact pending recovery scope."""
        with self.locked():
            pending = list(self._pending())
            if pending:
                path, journal = pending[0]
                if journal['impact']['kind'] == kind and journal['impact']['id'] == identifier:
                    return journal['impact']
                raise ValueError(
                    'An incomplete deletion needs recovery before another deletion'
                )
            return self._scope(kind, identifier)[0]

    def execute(self, kind, identifier, expected_token):
        """Revalidate and commit; report success only after complete cleanup."""
        with self.locked():
            pending = list(self._pending())
            if pending:
                path, journal = pending[0]
                if (
                    journal['impact']['kind'],
                    journal['impact']['id'],
                    journal['impact']['token'],
                ) != (kind, identifier, expected_token):
                    raise CatalogConflict(
                        'An incomplete deletion must be retried with its original confirmation'
                    )
                try:
                    if journal['phase'] in {'committed', 'cleanup_failed'}:
                        self._cleanup(journal)
                    else:
                        self._rollback(journal)
                    self._scrub(journal)
                    self._write(path, journal)
                except Exception as error:
                    raise ValueError(
                        'Deletion cleanup incomplete; retry the same operation. See Diagnostics.'
                    ) from error
                if journal['phase'] == 'completed':
                    return {**journal['impact'], 'completed': True}
            impact, edits, payloads = self._scope(kind, identifier)
            if not isinstance(expected_token, str) or impact['token'] != expected_token:
                raise CatalogConflict('Deletion scope changed; open its confirmation again')
            transaction_id = str(uuid.uuid4())
            path = self.journals / (transaction_id + '.json')
            journal = {
                'phase': 'prepared',
                'impact': impact,
                'edits': edits,
                'verified_jobs': self._verified_jobs(),
                'payloads': [
                    {
                        **item,
                        'staged': str(
                            Path(item['original']).with_name(
                                '.deletion-' + transaction_id + '-' + str(i)
                            )
                        ),
                    }
                    for i, item in enumerate(payloads)
                ],
            }
            self._write(path, journal)
            try:
                for item in journal['payloads']:
                    original, staged = Path(item['original']), Path(item['staged'])
                    if staged.exists():
                        raise ValueError('Deletion staging collision')
                    original.rename(staged)
                    _sync(original.parent)
                journal['phase'] = 'committing'
                self._write(path, journal)
                for edit in edits:
                    self._apply(edit, 'after')
                journal['phase'] = 'committed'
                self._write(path, journal)
            except Exception as error:
                try:
                    self._rollback(journal)
                    self._scrub(journal)
                    self._write(path, journal)
                except Exception:
                    journal['phase'] = 'rollback_failed'
                    self._write(path, journal)
                raise ValueError(
                    'Deletion failed or incomplete; protected artifacts require recovery. '
                    f'See Diagnostics. Cause: {error}'
                ) from error
            try:
                self._cleanup(journal)
                self._scrub(journal)
                self._write(path, journal)
            except Exception as error:
                journal['phase'] = 'cleanup_failed'
                self._write(path, journal)
                raise ValueError(
                    'Deletion cleanup incomplete; retry the same operation. See Diagnostics.'
                ) from error
            return {**impact, 'completed': True}
