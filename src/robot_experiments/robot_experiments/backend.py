"""Headless experiment client API; scientific ownership remains with existing services."""

from copy import deepcopy
from pathlib import Path

from .authoring import ExperimentAuthor, SLOTS
from .catalog import ExperimentCatalog
from .components import component_registries
from .jobs import ExperimentJobs
from .reporting import plain


class ExperimentBackend:
    """
    Compose authoring, execution and immutable result access for Python clients.

    ``directory`` owns only jobs and session metadata. ``result_roots`` registers
    existing result authorities; telemetry and scientific scores stay in Runs.
    No ROS node, GUI, process or file is created merely by constructing this API.
    """

    def __init__(self, directory, *, catalog=None, result_roots=()):
        self.directory = Path(directory).expanduser().resolve()
        self.catalog = catalog if catalog is not None else ExperimentCatalog()
        self.author = ExperimentAuthor(self.catalog)
        self.jobs = ExperimentJobs(self.directory / 'jobs')
        self.result_roots = tuple(dict.fromkeys(
            [self.directory, *(Path(p).expanduser().resolve() for p in result_roots)]))

    def list_experiments(self, *, include_deleted=False):
        """Return detached Catalog summaries, retaining immutable historical IDs."""
        result = self.catalog.list(include_deleted=include_deleted)
        for item in result:
            definition = self.catalog.get(item['id'], item['revision'])['definition']
            item['schema_version'] = definition['schema_version']
            item['one_file_authoring_supported'] = definition['schema_version'] == 1
            item['base_variants'] = [v['name'] for v in definition['experiment']['variants']]
        return result

    def list_revisions(self, experiment_id):
        """Return chronological immutable revision metadata, not Catalog internals."""
        latest = self.catalog.get(experiment_id)
        return [{key: record[key] for key in ('id', 'revision', 'created_utc', 'sha256')}
                for record in (self.catalog.get(experiment_id, number)
                               for number in range(1, latest['revision'] + 1))]

    def list_component_options(self, role, *, context=None):
        """
        Expose deployability separately from declared-context compatibility.

        No context means compatibility is unknown. Even a compatible declaration
        requires complete experiment preflight before it can be deployed.
        Historical aliases are grouped using owner metadata, never a second map.
        """
        registries = component_registries()
        family = SLOTS.get(role, role)
        if family not in registries:
            raise ValueError(f'unknown component role/family: {role!r}')
        registry = registries[family]
        descriptions = registry.describe_all()
        result = []
        for detail in descriptions:
            if detail['capabilities'].get('alias_of'):
                continue
            aliases = [x['id'] for x in descriptions
                       if x['capabilities'].get('alias_of') == detail['id']]
            compatible = (registry.compatibility(detail['id'], context)
                          if context is not None else None)
            result.append({**detail, 'aliases': aliases,
                           'deployable': detail['capabilities'].get('deployable', True),
                           'compatible': compatible['compatible'] if compatible else None,
                           'reasons': compatible['reasons'] if compatible else [],
                           'validation_scope': 'declared requirements; '
                                               'full draft preflight required'})
        return plain(result)

    def new_draft(self, experiment_id, name, base_case, *, base_revision=None, base_variant=None):
        """Create a detached one-file Author specification pinned to a real base revision."""
        self.catalog._identifier(experiment_id)
        if not isinstance(name, str) or not name.strip():
            raise ValueError('name: expected a nonempty display name')
        base = self.catalog.get(base_case, base_revision)
        if base['definition']['schema_version'] != 1:
            raise ValueError('one-file authoring requires a schema-1 base; '
                             'canonical comparisons remain runnable through the Catalog')
        draft = {'schema_version': 1, 'experiment': {
            'id': experiment_id, 'name': name, 'base_case': base['id'],
            'base_revision': base['revision']}}
        if base_variant is not None:
            draft['experiment']['base_variant'] = base_variant
        return draft

    def reset_draft(self, specification):
        """
        Discard local choices/overrides; retain the draft's base and identity.

        Restoring an earlier unsaved edit uses the client's detached draft value;
        neither operation revises the Catalog or changes any package default.
        """
        experiment, _ = self.author._read(specification)
        keys = ('id', 'name', 'base_case', 'base_revision', 'base_variant')
        return {'schema_version': 1,
                'experiment': {k: deepcopy(experiment[k]) for k in keys if k in experiment}}

    def preview_experiment(self, specification):
        """Project the existing Author/explain result, schemas and known provenance."""
        try:
            preview = self.author.preview(specification)
        except ValueError as error:
            raise type(error)(self._public_error(error)) from error
        registries = component_registries()
        for slot, detail in preview['components'].items():
            metadata = registries[SLOTS[slot]].describe(detail['id'])
            recorded = detail['id']
            detail['id'] = metadata['capabilities'].get('alias_of') or recorded
            if detail['id'] != recorded:
                detail['recorded_id'] = recorded
            detail.update({k: metadata[k] for k in ('schema', 'capabilities', 'requirements')})
            detail['parameter_sources'] = {}
            for key in detail.get('effective', {}):
                direct = (key in detail['overrides']
                          and not detail['advanced_owner_overrides_present'])
                detail['parameter_sources'][key] = {
                    'kind': 'experiment_override' if direct else 'resolved_owner_or_inherited',
                    'scope': 'direct Author parameters; full per-leaf ancestry unavailable'}
        self._canonical_selections(preview['effective'], registries)
        preview['summary'] = '\n'.join(
            [f"{preview['id']}: {preview['name']}",
             *(f"{slot}: {detail['id']}" for slot, detail in preview['components'].items())])
        preview['provenance_scope'] = (
            'Owner defaults, effective values, direct authored overrides and selection origin; '
            'resource ancestry is authoritative in effective.runs.sources. '
            'Advanced owner overrides and inherited values have no complete per-leaf ancestry.')
        return plain(preview)

    @staticmethod
    def _canonical_selections(effective, registries):
        for run in effective['runs']:
            selected = run['system']['selection']
            for slot in ('robot_estimator', 'target_estimator'):
                key = slot + '_type'
                metadata = registries[slot].describe(selected[key])
                canonical = metadata['capabilities'].get('alias_of') or selected[key]
                if selected[key] != canonical:
                    run['system'].setdefault('historical_selection', {})[key] = selected[key]
                    selected[key] = canonical

    def explain_experiment(self, experiment_id, revision=None):
        """Explain any saved revision/study through the existing seeded owner path."""
        try:
            effective = self.catalog.explain(experiment_id, revision)
        except ValueError as error:
            raise type(error)(self._public_error(error)) from error
        self._canonical_selections(effective, component_registries())
        return plain(effective)

    def validate_experiment(self, specification):
        """Return existing authoritative validation errors without launching or persisting."""
        try:
            compiled = self.author.compile(specification)
            spec = self.author.validate(compiled)
            return plain({'valid': True, 'errors': [],
                          'identities': spec['variants'][0]['plan']['identities']})
        except (ValueError, TypeError, KeyError, OSError) as error:
            return {'valid': False, 'errors': [self._public_error(error)]}

    def save_experiment(self, specification, *, expected_revision=None):
        """Delegate create/revise and optimistic concurrency to the Catalog owner."""
        try:
            return plain(self.author.save(specification, expected_revision=expected_revision))
        except ValueError as error:
            raise type(error)(self._public_error(error)) from error

    @staticmethod
    def _public_error(error):
        # Match only directories owned by the existing preparation services.
        # User-supplied resources and permanent Run paths remain real references.
        import re
        import tempfile
        pattern = (re.escape(tempfile.gettempdir())
                   + r'/(?:experiment_definition|robot_composition|'
                   r'robot_experiment_explain)_[^/\s:]+')
        return re.sub(pattern, '<materialized>', str(error))

    @staticmethod
    def gazebo_setup_capabilities():
        """Separate interactive setup from Runner-owned scored deployments."""
        from .simulation import display_capabilities
        display=display_capabilities()
        return {'interactive_setup_supported':True,'gui_available':display['available'],
                'scored_gui_supported':True,'live_reconfiguration_supported':False,
                'controlled_restart_required':True,'reason':display['reason']}

    def get_simulation_status(self, simulation_id=None):
        """Observe the owned unscored preview; construction/status never launch."""
        return self.jobs.preview_status(simulation_id)

    def open_simulation(self, experiment_id, revision, *, variant, seed=None):
        """Open one saved validated configuration with actuation unconditionally OFF."""
        if type(revision) is not int or revision<1:
            raise ValueError('Select an explicit positive saved revision')
        try:
            return self.jobs.start_simulation(self.catalog,experiment_id,revision,
                                              variant=variant,seed=seed)
        except ValueError as error:
            raise ValueError(self._public_error(error)) from error

    def close_simulation(self, simulation_id, *, timeout=25.):
        """Cooperate with the existing worker/OwnedLaunch cleanup; never kill arbitrary PIDs."""
        self.jobs.preview_status(simulation_id)
        return self.jobs.stop(simulation_id,timeout=timeout)


    @staticmethod
    def _read(path, default=None):
        import json
        path = Path(path)
        if not path.is_file():
            return deepcopy(default)
        if path.stat().st_size > 32 * 1024 * 1024:
            raise ValueError(f'recorded metadata exceeds 32 MiB; use artifact reference: {path}')
        return json.loads(path.read_text())

    @staticmethod
    def _identifier(identifier):
        import uuid
        if not isinstance(identifier, str) or str(uuid.UUID(identifier)) != identifier:
            raise ValueError('expected canonical UUID')
        return identifier

    @staticmethod
    def _page(offset, limit):
        if (type(offset) is not int or offset < 0
                or type(limit) is not int or not 0 <= limit <= 1000):
            raise ValueError('offset must be nonnegative; limit must be an integer in [0, 1000]')

    def _roots(self):
        # Existing immutable job requests retain registered output locations across
        # client restarts. This is discovery, not a second persistent results index.
        roots = list(self.result_roots)
        for path in sorted(self.jobs.directory.glob('*/request.json')):
            request = self._read(path)
            if request and request.get('output'):
                roots.append(Path(request['output']).resolve())
        return tuple(dict.fromkeys(roots))

    def _run_paths(self):
        return sorted({p.parent.resolve() for root in self._roots()
                       for p in root.rglob('runs/*/status.json')})

    def _find_run(self, run_id):
        self._identifier(run_id)
        paths = [p for p in self._run_paths()
                 if self._read(p / 'status.json', {}).get('run_id') == run_id]
        if not paths:
            raise KeyError(f'unknown Run ID: {run_id}')
        if len(paths) != 1:
            raise ValueError(f'ambiguous duplicate Run ID: {run_id}')
        return paths[0]

    def _run_row(self, directory):
        status = self._read(directory / 'status.json', {})
        metadata = self._read(directory / 'metadata.json', {})
        identity = status.get('experiment_identity') or metadata.get('experiment_identity') or {}
        return {'run_id': status.get('run_id'), 'experiment_id': identity.get('id'),
                'revision': identity.get('revision'), 'experiment_identity': identity,
                'seed': metadata.get('seed'), 'variant': metadata.get('variant'),
                'execution_status': status.get('execution_status'),
                'acceptance': status.get('acceptance'), 'started_utc': status.get('started_utc'),
                'directory': str(directory), 'study': str(directory.parent.parent)}

    def list_runs(self, filters=None, *, offset=0, limit=100):
        """
        Page headers only; never read telemetry, time series or scientific scores.

        Legacy artifacts without Run IDs remain listed with run_id=None. They are
        not assigned fabricated identities. New Runner attempts always have IDs.
        """
        self._page(offset, limit)
        filters = filters or {}
        allowed = {'run_id', 'experiment_id', 'revision', 'seed', 'variant',
                   'execution_status', 'acceptance', 'study'}
        if not isinstance(filters, dict) or set(filters) - allowed:
            raise ValueError('unsupported run filters')
        result, matched = [], 0
        for directory in self._run_paths():
            row = self._run_row(directory)
            if any(row[key] != value for key, value in filters.items()):
                continue
            if matched >= offset and len(result) < limit:
                result.append(row)
            matched += 1
            if len(result) == limit:
                break
        return result

    def get_run_status(self, run_id):
        """Read Runner's real state/error/acceptance/cleanup; no inferred percentage."""
        directory = self._find_run(run_id)
        return {**self._read(directory / 'status.json'), 'directory': str(directory)}

    def _results(self, directory, run_id):
        status = self._read(directory / 'status.json')
        if status is None or status.get('run_id') != run_id:
            raise KeyError(f'Run evidence missing or identity changed: {run_id}')
        metadata = self._read(directory / 'metadata.json', {})
        if metadata.get('run_id', run_id) != run_id:
            raise ValueError('Run metadata/status identity mismatch')
        resolved = self._read(directory / 'resolved_configuration.json', {})
        artifacts = {key: str(directory / name) if (directory / name).exists() else None
                     for key, name in {'metadata': 'metadata.json', 'metrics': 'metrics.json',
                                       'time_series': 'time_series.json', 'telemetry': 'telemetry',
                                       'frozen_inputs': 'inputs',
                                       'configuration': 'resolved_configuration.json',
                                       'events': 'events.json',
                                       'launch_log': 'launch.log'}.items()}
        return {'available': True, **self._run_row(directory), 'status': status,
                'metadata': metadata,
                'metrics': self._read(directory / 'metrics.json', {
                    'available': False, 'reason': 'metrics artifact not written'}),
                'configuration': resolved,
                'comparison': self._read(directory / 'comparison.json'),
                'artifacts': artifacts,
                'progress': {'phase': status.get('execution_status'),
                             'readiness': 'not separately published; see recorded errors/events',
                             'percentage': None}}

    def get_run_results(self, run_id):
        """Return recorded evidence and references, without recomputing metrics."""
        return self._results(self._find_run(run_id), run_id)

    def get_historical_results(self, directory):
        """Read a registered historical attempt even if it predates Runner UUIDs."""
        directory = Path(directory).expanduser().resolve()
        if directory not in self._run_paths():
            raise ValueError('historical directory is not a registered Run artifact location')
        run_id = self._read(directory / 'status.json', {}).get('run_id')
        return self._results(directory, run_id)

    def read_telemetry(self, run_id, stream, *, offset=0, limit=100):
        """Read a bounded page of existing JSONL records; no alignment or resampling."""
        import itertools
        import json
        import re
        self._page(offset, limit)
        if not isinstance(stream, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,79}', stream):
            raise ValueError('stream must be a recorded telemetry name')
        directory = self._find_run(run_id) / 'telemetry'
        path = (directory / (stream + '.jsonl')).resolve()
        if not path.is_relative_to(directory.resolve()):
            raise ValueError('telemetry reference escapes Run directory')
        with path.open() as source:
            records = [json.loads(line)
                       for line in itertools.islice(source, offset, offset + limit)]
        return {'run_id': run_id, 'stream': stream, 'offset': offset, 'records': records,
                'source': str(path)}

    def read_time_series(self, run_id, *, offset=0, limit=100):
        """Page scored, recorded curves unchanged; never calculate or resample values."""
        self._page(offset, limit)
        path = self._find_run(run_id) / 'time_series.json'
        document = self._read(path)
        if document is None:
            return {'run_id': run_id, 'available': False, 'offset': offset, 'series': {},
                    'reason': 'recorded time-series artifact not written', 'source': str(path)}
        if (not isinstance(document, dict) or document.get('schema_version') != 1
                or not isinstance(document.get('series'), dict)):
            raise ValueError('invalid recorded time-series document')
        series = {}
        for name, detail in document['series'].items():
            if not isinstance(detail, dict) or not isinstance(detail.get('samples'), list):
                raise ValueError(f'invalid recorded time-series samples: {name}')
            series[name] = {**detail, 'sample_count': len(detail['samples']),
                            'samples': detail['samples'][offset:offset + limit]}
        return {'run_id': run_id, 'available': True, 'offset': offset,
                'series': series, 'source': str(path)}

    def start_run(self, experiment_id, revision, *, output, enable_actuation=False, gui=False,
                  preview_id=None):
        """
        Start existing jobs, explicitly opting into physical commands for this call.

        The returned job_id is not a Run ID. Run IDs appear only when the Runner
        writes actual attempts; a revision may produce multiple seed/variant Runs.
        """
        if (type(revision) is not int or revision < 1 or type(enable_actuation) is not bool
                or type(gui) is not bool):
            raise ValueError(
                'an explicit positive revision and boolean actuation opt-in are required')
        try:
            job = self.jobs.start(self.catalog, experiment_id, revision, output=output,
                                  runtime_overrides={'enable_actuation': enable_actuation, 'gui': gui},
                                  **({'preview_id': preview_id} if preview_id is not None else {}))
        except ValueError as error:
            raise type(error)(self._public_error(error)) from error
        return self.get_job_status(job['job_id'])

    def get_job_status(self, job_id):
        """Project worker status plus actual Runner attempts, including live preparation."""
        status = self.jobs.status(job_id)
        if isinstance(status.get('error'), str):
            status['error'] = self._public_error(status['error'])
        study = status.get('study')
        status['runs'] = self.list_runs({'study': study}, limit=1000) if study else []
        status['run_ids'] = [row['run_id'] for row in status['runs'] if row['run_id']]
        status['progress_available'] = bool(study)
        return status

    def cancel_job(self, job_id, *, timeout=25.0):
        """Cooperatively cancel queued/running jobs via existing stop/finally cleanup."""
        self.jobs.stop(job_id, timeout=timeout)
        return self.get_job_status(job_id)

    def cancel_run(self, run_id, *, timeout=25.0):
        """Cancel the owning job (all its Runs); historical/unowned Runs cannot be signalled."""
        directory = self._find_run(run_id)
        study = str(directory.parent.parent)
        owners = [path.parent.name for path in self.jobs.directory.glob('*/status.json')
                  if self._read(path, {}).get('study') == study]
        if len(owners) != 1:
            raise ValueError('Run has no unique owned job; cancellation unsupported')
        return self.cancel_job(owners[0], timeout=timeout)

    def _session_path(self, session_id):
        return self.directory / 'sessions' / (self._identifier(session_id) + '.json')

    def _session(self, session_id):
        session = self._read(self._session_path(session_id))
        if session is None:
            raise KeyError(f'unknown Session ID: {session_id}')
        expected = {'schema_version', 'session_id', 'created_utc', 'metadata', 'runs'}
        if (not isinstance(session, dict) or set(session) != expected
                or type(session['schema_version']) is not int or session['schema_version'] != 1
                or session['session_id'] != session_id or not isinstance(session['runs'], list)
                or not isinstance(session['created_utc'], str)
                or not isinstance(session['metadata'], dict)
                or set(session['metadata']) - {'title', 'notes'}
                or any(not isinstance(v, str) for v in session['metadata'].values())):
            raise ValueError('invalid Session metadata')
        seen = set()
        for reference in session['runs']:
            if not isinstance(reference, dict) or set(reference) != {'run_id', 'directory'}:
                raise ValueError('invalid Session Run reference')
            self._identifier(reference['run_id'])
            path = Path(reference['directory'])
            if (not path.is_absolute()
                    or not any(path.resolve().is_relative_to(r) for r in self._roots())):
                raise ValueError('Session references an unregistered result root')
            if reference['run_id'] in seen:
                raise ValueError('duplicate Session Run reference')
            seen.add(reference['run_id'])
        return session

    def create_session(self, metadata=None):
        """Persist one small UUID record; no Run evidence or metric copies."""
        from datetime import datetime, timezone
        import uuid
        from .reporting import write_json
        metadata = {} if metadata is None else deepcopy(metadata)
        if (not isinstance(metadata, dict) or set(metadata) - {'title', 'notes'}
                or any(not isinstance(v, str) for v in metadata.values())):
            raise ValueError('Session metadata supports string title/notes only')
        session = {'schema_version': 1, 'session_id': str(uuid.uuid4()),
                   'created_utc': datetime.now(timezone.utc).isoformat(),
                   'metadata': metadata, 'runs': []}
        path = self._session_path(session['session_id'])
        path.parent.mkdir(parents=True, exist_ok=True)
        write_json(path, session)
        return session

    def add_run_to_session(self, session_id, run_id):
        """Append a proven reference under a lock; retries are idempotent."""
        import fcntl
        from .reporting import write_json
        directory = self._find_run(run_id)
        path = self._session_path(session_id)
        if not path.is_file():
            raise KeyError(f'unknown Session ID: {session_id}')
        with path.with_suffix('.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                session = self._session(session_id)
                if not any(r['run_id'] == run_id for r in session['runs']):
                    session['runs'].append({'run_id': run_id, 'directory': str(directory)})
                    write_json(path, session)
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)
        return session

    def list_sessions(self, *, title=None, offset=0, limit=100):
        """Page Session summaries in stable creation order; no duplicate results store."""
        self._page(offset, limit)
        sessions = [self._session(path.stem)
                    for path in (self.directory / 'sessions').glob('*.json')]
        summaries = [{k: value for k, value in session.items() if k != 'runs'} |
                     {'run_count': len(session['runs'])} for session in sessions
                     if title is None or session['metadata'].get('title') == title]
        ordered = sorted(summaries, key=lambda x: (x['created_utc'], x['session_id']))
        return ordered[offset:offset + limit]

    def get_session_comparison(self, session_id):
        """Present real stored metrics with existing scientific guards; never rank."""
        from .comparison import (
            comparison_reasons, comparison_views, different_paths, metric_comparison_reasons,
            _metric_evidence,
        )
        session = self._session(session_id)
        rows, checks, reasons, differences = [], [], [], []
        for reference in session['runs']:
            try:
                result = self._results(Path(reference['directory']), reference['run_id'])
            except (KeyError, ValueError, OSError) as error:
                result = {**reference, 'available': False, 'metrics': None, 'error': str(error)}
                reasons.append(f"{reference['run_id']}: evidence unavailable: {error}")
            # Compact table keeps original metric objects/availability, never synthesized scores.
            columns = ('run_id', 'available', 'experiment_id', 'revision', 'seed', 'variant',
                       'execution_status', 'acceptance', 'status', 'metrics', 'artifacts', 'error')
            rows.append({k: result[k] for k in columns if k in result})
            if not result['available']:
                continue
            view = result['comparison']
            if not view:
                reasons.append(f"{result['run_id']}: comparison evidence unavailable")
                continue
            check = {'variant': result['run_id'], 'seed': result['seed'],
                     'execution_status': result['status'].get('execution_status'),
                     'comparison_contract': view.get('original_hash'),
                     'comparison': view, 'metrics': result['metrics']}
            try:
                expected = comparison_views(view['original_contract'], view.get('policy'),
                                            view['protected_conditions'])
                if view != expected:
                    raise ValueError('comparison hash/metadata differs from original evidence')
                checks.append(check)
                reasons.extend(f"{result['run_id']}: {reason}"
                               for reason in _metric_evidence(check, view))
            except (ValueError, KeyError, TypeError) as error:
                reasons.append(f"{result['run_id']}: invalid comparison evidence: {error}")
        if len(checks) < 2:
            reasons.append('at least two Runs with comparison evidence are required')
        if checks:
            first = checks[0]['comparison']
            policy = first.get('policy')
            if any(c['comparison'].get('policy') != policy for c in checks):
                reasons.append('comparison policies differ')
                policy = None
            reasons.extend(comparison_reasons(
                checks, [c['variant'] for c in checks],
                sorted({c['seed'] for c in checks}, key=str),
                policy))
            for check in checks[1:]:
                reasons.extend(f"{check['variant']}: {reason}"
                               for reason in metric_comparison_reasons(checks[0], check))
                paths = {field: different_paths(first[field], check['comparison'][field])
                         for field in ('original_contract', 'protected_conditions')}
                differences.append({'first_run': checks[0]['variant'],
                                    'second_run': check['variant'], **paths})
                if paths['protected_conditions']:
                    reasons.append(f"{check['variant']}: protected condition differences: "
                                   + ', '.join(paths['protected_conditions']))
        return {'session': session, 'runs': rows, 'ranking': None,
                'comparability': {'comparable': not reasons,
                                  'reasons': list(dict.fromkeys(reasons)),
                                  'differences': differences,
                                  'scope': 'existing recorded comparison policies, metric/window/'
                                           'coverage evidence and protected conditions; '
                                           'no automatic ranking'}}
