"""Headless experiment client API; scientific ownership remains with existing services."""

from contextlib import contextmanager, nullcontext
from copy import deepcopy
from pathlib import Path

from .authoring import ExperimentAuthor, SLOTS
from .catalog import CatalogConflict, ExperimentCatalog
from .components import component_registries
from .jobs import ExperimentJobs
from .reporting import plain


class ExperimentBackend:
    """
    Compose authoring, execution and immutable result access for Python clients.

    ``directory`` owns jobs, session metadata and the user Dynamics Library.
    ``result_roots`` registers existing result authorities; telemetry and scores stay in Runs.
    No ROS node, GUI, process or file is created merely by constructing this API.
    """

    def __init__(self, directory, *, catalog=None, result_roots=(), managed_result_roots=None,
                 dynamics_library_path=None):
        """Separate registered read access from explicitly managed deletion boundaries."""
        self.directory = Path(directory).expanduser().resolve()
        self.catalog = catalog if catalog is not None else ExperimentCatalog()
        self.author = ExperimentAuthor(self.catalog)
        from .dynamics_library import DynamicsLibrary
        self.dynamics = DynamicsLibrary(
            dynamics_library_path or self.directory / 'dynamics_library.yaml')
        if self.dynamics.path == self.catalog.path:
            raise ValueError('Dynamics Library and Experiment Catalog must have distinct paths')
        self.jobs = ExperimentJobs(self.directory / 'jobs')
        self.result_roots = tuple(dict.fromkeys(
            [self.directory, *(Path(p).expanduser().resolve() for p in result_roots)]))
        if (isinstance(managed_result_roots, (str, bytes))
                or hasattr(managed_result_roots, '__fspath__')):
            raise ValueError('managed_result_roots must be a sequence of root paths')
        try:
            self.managed_result_roots = tuple(dict.fromkeys(
                Path(p).expanduser().absolute() for p in (
                    (self.directory / 'runs',) if managed_result_roots is None
                    else managed_result_roots)))
        except (TypeError, ValueError) as error:
            raise ValueError('managed_result_roots must be a sequence of root paths') from error
        if any(root == root.parent for root in self.managed_result_roots):
            raise ValueError('A filesystem root cannot be a managed deletion root')
        self.result_roots = tuple(dict.fromkeys([*self.result_roots, *self.managed_result_roots]))

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

    def list_scenarios(self, *, include_deleted=False):
        """Project legacy variants as independent Scenarios without rewriting history."""
        import hashlib
        state = self.catalog._read()
        result = []
        for row in self.list_experiments(include_deleted=include_deleted):
            entry = state['experiments'][row['id']]
            if len(row['base_variants']) == 1:
                result.append({**row, 'variant': row['base_variants'][0],
                               'origin': deepcopy(entry.get('scenario_origin'))})
                continue
            for variant in row['base_variants']:
                digest = hashlib.sha256((row['id'] + '\0' + variant).encode()).hexdigest()[:24]
                identifier = 'EXP-SCENARIO-' + digest.upper()
                expected = {'id': row['id'], 'variant': variant}
                if identifier in state['experiments']:
                    origin = state['experiments'][identifier].get('scenario_origin') or {}
                    if any(origin.get(k) != v for k, v in expected.items()):
                        raise CatalogConflict('Scenario projection ID conflicts with an existing record')
                    continue  # Actual child appears independently, or remains explicitly deleted.
                if identifier in state['retired_ids']:
                    continue
                suffix = variant.upper() if len(variant) <= 4 else variant.replace('_', ' ')
                result.append({**row, 'id': identifier, 'name': row['name'] + ' · ' + suffix,
                               'source_id': row['id'], 'variant': variant, 'origin': None})
        return result

    def ensure_scenario(self, scenario_id):
        """Lazily freeze one selected legacy variant through real Catalog preflight."""
        choice = next((r for r in self.list_scenarios() if r['id'] == scenario_id), None)
        if choice is None:
            raise ValueError('Select an available saved Scenario')
        if 'source_id' not in choice:
            return self.catalog.get(scenario_id)
        source = self.catalog.get(choice['source_id'], choice['revision'])
        definition = deepcopy(source['definition'])
        definition['experiment']['variants'] = [
            v for v in definition['experiment']['variants'] if v['name'] == choice['variant']]
        origin = {'id': source['id'], 'revision': source['revision'],
                  'sha256': source['sha256'], 'variant': choice['variant']}
        try:
            with self._management_lock():
                return self.catalog.create(scenario_id, choice['name'], definition,
                                           source['overrides'], scenario_origin=origin)
        except CatalogConflict:
            # Another client may have frozen the identical source while preflight ran.
            existing = self.catalog._read()['experiments'].get(scenario_id, {})
            if existing.get('scenario_origin') != origin or existing.get('deleted'):
                raise
            return self.catalog.get(scenario_id)

    @staticmethod
    def unique_display_name(name, existing):
        """Generate short collision-safe labels independently of immutable technical IDs."""
        name = name.strip()
        if not name:
            raise ValueError('A nonempty display name is required')
        candidate, number = name, 2
        while candidate in existing:
            candidate = f'{name} {number}'
            number += 1
        return candidate

    def list_environment_options(self):
        """Discover named environment resources from their resolved owner documents."""
        from .composition import load_resource
        result = []
        for identifier in self.author.discover()['resources']:
            document = load_resource(self.author._resource(identifier),
                                     base=self.catalog.builtin.parent, share=self.catalog.share)
            if 'environment' not in document:
                continue
            fields = document['environment'].get('acceleration_fields', [])
            active = [f['type'] for f in fields if f.get('enabled', True)]
            result.append({'id': identifier, 'active_fields': active,
                           'label': identifier.replace('_', ' ').capitalize()
                           if active else 'No external acceleration'})
        return result

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
        if base['definition']['experiment'].get('comparison'):
            draft['experiment']['comparison'] = deepcopy(base['definition']['experiment']['comparison'])
        if base_variant is not None:
            draft['experiment']['base_variant'] = base_variant
        return draft

    def list_comparison_variables(self):
        """Discover exact declaration paths from registered component owner schemas."""
        from .comparison_variables import comparison_options
        return comparison_options()

    def draft_with_dynamics(self, specification, library_id, *, revision):
        """Pin a library revision into a detached ordinary Author draft; never launch."""
        experiment, _ = self.author._read(specification)
        experiment = deepcopy(experiment)
        resolved = self.dynamics.resolve(library_id, revision)
        experiment.setdefault('components', {})['target_dynamics'] = 'equation_state_space'
        experiment.setdefault('parameters', {})['target_dynamics'] = {'model': resolved['model']}
        experiment['dynamics_source'] = resolved['source']
        return {'schema_version': 1, 'experiment': experiment}

    @staticmethod
    def describe_robot_response(model, influences=None):
        """Expose owner-validated capability and detached frozen data to clients."""
        from robot_simulation.dynamics.derivative_influences import prepare_robot_response
        from .dynamics_library import content_hash
        try:
            profiles = prepare_robot_response(model, influences)
            return {'available': True, 'enabled': profiles[0]['enabled'],
                    'influences': profiles, 'sha256': content_hash(profiles), 'reason': ''}
        except (ValueError, TypeError, KeyError) as error:
            return {'available': False, 'enabled': False, 'reason': str(error)}

    def draft_with_robot_response(self, specification, enabled):
        """Freeze an experiment-only influence through existing Author parameters."""
        if type(enabled) is not bool:
            raise ValueError('Robot Response requires explicit boolean opt-in')
        from robot_simulation.dynamics.derivative_influences import prepare_robot_response
        experiment, _ = self.author._read(specification)
        experiment = deepcopy(experiment)
        local = experiment.get('parameters', {}).get('target_dynamics', {})
        if not enabled and 'model' in local and local.get('influences'):
            # Allow a rejected ON draft to recover without bypassing OFF preflight.
            local['influences'] = prepare_robot_response(
                local['model'], local['influences'], enabled=False)
        preview = self.preview_experiment({'schema_version': 1, 'experiment': experiment})
        effective = preview['components']['target_dynamics']
        if effective['id'] != 'equation_state_space':
            raise ValueError('Robot Response requires explicitly bound equation dynamics')
        values = effective['effective']
        profiles = prepare_robot_response(values['model'], values.get('influences'), enabled=enabled)
        experiment = deepcopy(experiment)
        experiment.setdefault('parameters', {})['target_dynamics'] = {
            'model': deepcopy(values['model']), 'influences': profiles}
        source = preview.get('provenance', {}).get('dynamics_library')
        if source is not None:
            experiment['dynamics_source'] = deepcopy(source)
        return {'schema_version': 1, 'experiment': experiment}

    def validate_experiment_id(self, identifier):
        """Reject malformed and reserved identities; saving rechecks atomically."""
        if not self.catalog.is_id_available(identifier):
            raise CatalogConflict('experiment ID already exists or was permanently retired')
        return identifier

    def propose_experiment_id(self, display_name):
        """Use a Latin slug, or a stable UTF-8 hash when no ASCII identifier remains."""
        import hashlib
        import re
        import unicodedata
        if not isinstance(display_name, str) or not display_name.strip():
            raise ValueError('name: expected a nonempty display name')
        name = unicodedata.normalize('NFKC', display_name.strip())
        ascii_name = unicodedata.normalize('NFKD', name).encode('ascii', 'ignore').decode()
        slug = re.sub(r'[^A-Z0-9]+', '-', ascii_name.upper()).strip('-')
        fingerprint = hashlib.sha256(name.encode('utf-8')).hexdigest()[:12].upper()
        if not slug:
            slug = fingerprint
        elif len(slug) > 71:
            slug = slug[:58].rstrip('-') + '-' + fingerprint
        return self.validate_experiment_id('EXP-' + slug)

    @staticmethod
    @contextmanager
    def _metadata_lock(path):
        """Never follow a substituted metadata lock into another owner's files."""
        import fcntl
        import os
        import stat
        try:
            descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        except OSError as error:
            raise ValueError(
                'owned metadata lock unavailable; symlinks are unsupported') from error
        with os.fdopen(descriptor, 'a') as lock:
            if not stat.S_ISREG(os.fstat(lock.fileno()).st_mode):
                raise ValueError('owned metadata lock must be a regular file')
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)

    @contextmanager
    def _management_lock(self, *, allow_pending=False):
        # Serialize dashboard metadata mutations with accepted Job creation.
        # Execution still belongs entirely to ExperimentJobs and the Runner.
        self.directory.mkdir(parents=True, exist_ok=True)
        with self._metadata_lock(self.directory / '.management.lock'):
            if not allow_pending:
                for path in (self.directory / '.deletions').glob('*.json'):
                    if self._read(path).get('phase') not in {'completed', 'rolled_back'}:
                        raise ValueError('An incomplete deletion needs recovery; see Diagnostics')
            yield

    def preview_deletion(self, kind, identifier):
        """Calculate the exact owned cascade; read access never grants delete authority."""
        from .deletion import ResourceDeletion
        return ResourceDeletion(self).preview(kind, identifier)

    def delete_resources(self, kind, identifier, *, expected_token):
        """Commit only a confirmed revalidated scope, or resume that same transaction."""
        from .deletion import ResourceDeletion
        return ResourceDeletion(self).execute(kind, identifier, expected_token)

    def rename_experiment(self, experiment_id, name, *, expected_revision, expected_name):
        """Change Catalog-owned display metadata, preserving recorded scientific identity."""
        with self._management_lock():
            return self.catalog.rename_display(
                experiment_id, name, expected_revision=expected_revision,
                expected_name=expected_name)

    def _job_cleanup_verified(self, status):
        """Read authoritative cleanup evidence for legacy terminal workers."""
        if status.get('state') not in {'completed', 'cancelled', 'failed', 'closed', 'error'}:
            return False
        if status.get('cleanup_complete') is False:
            return False
        study = status.get('study')
        if study is None:
            return status.get('cleanup_complete') is True
        # A completed deletion can remove the original cleanup artifacts. Its
        # small receipt proves only the exact unchanged terminal Job documents.
        directory = status.get('artifacts')
        if directory and isinstance(study, str) and not Path(study).exists():
            directory = Path(directory)
            if directory.parent == self.jobs.directory:
                request = self._read(directory / 'request.json')
                proof = {'status': self._session_token(status),
                         'request': self._session_token(request)}
                for path in (self.directory / '.deletions').glob('*.json'):
                    receipt = self._read(path)
                    if (receipt.get('phase') == 'completed'
                            and receipt.get('verified_jobs', {}).get(directory.name) == proof):
                        return True
        if not isinstance(study, str):
            return False
        from .deletion import _plain_path
        try:
            study = _plain_path(study)
            if not any(study.is_relative_to(root) for root in self._roots()):
                return False
            children = list((study / 'runs').iterdir())
            if not children:
                return False
            for child in children:
                _plain_path(child / 'status.json')
                run = self._read(child / 'status.json')
                if (not isinstance(run, dict)
                        or run.get('execution_status') not in {'completed', 'failed', 'cancelled'}
                        or not isinstance(run.get('cleanup'), dict)
                        or run['cleanup'].get('survivors') != []):
                    return False
            return True
        except (ValueError, TypeError, OSError):
            return False

    def _assert_no_associated_job(self, *, session=None, experiment_id=None):
        studies = ({str(Path(r['directory']).parent.parent.resolve())
                    for r in session['runs']} if session else set())
        directories = {p.parent for pattern in ('*/request.json', '*/status.json')
                       for p in self.jobs.directory.glob(pattern)}
        for directory in sorted(directories):
            try:
                request = self._read(directory / 'request.json')
                if (not isinstance(request, dict)
                        or not isinstance(request.get('record'), dict)):
                    raise ValueError('Job request association unavailable')
                self.catalog._identifier(request['record']['id'])
                if request.get('session_id') is not None:
                    self._identifier(request['session_id'])
                associated = (request.get('record', {}).get('id') == experiment_id
                              if experiment_id else
                              request.get('session_id') == session['session_id'])
                status = self.jobs.status(directory.name)
                if session and status.get('study'):
                    associated = associated or str(Path(status['study']).resolve()) in studies
            except (ValueError, KeyError, TypeError, OSError) as error:
                # Corrupt owner records cannot establish that cleanup is complete.
                raise ValueError(
                    'Job association or cleanup is unknown; inspect Diagnostics') from error
            safe = self._job_cleanup_verified(status)
            if session and not request.get('session_id') and not safe:
                raise ValueError('Legacy Job Session association is unknown; '
                                 'wait for completion/cleanup')
            if not associated:
                continue
            if not safe:
                raise ValueError('Associated Job is active or cleanup is unresolved')

    def archive_experiment(self, experiment_id, *, expected_revision):
        """Soft-delete through the Catalog; every immutable revision remains readable."""
        self.catalog._identifier(experiment_id)
        if type(expected_revision) is not int or expected_revision < 1:
            raise ValueError('expected_revision must be a positive integer')
        with self._management_lock(), self.jobs._preview_lock():
            self._assert_no_associated_job(experiment_id=experiment_id)
            self.catalog.delete(experiment_id, expected_revision=expected_revision)

    def reset_draft(self, specification):
        """
        Discard local choices/overrides; retain base, identity and comparison scope.

        Restoring an earlier unsaved edit uses the client's detached draft value;
        neither operation revises the Catalog or changes any package default.
        """
        experiment, _ = self.author._read(specification)
        keys = ('id', 'name', 'base_case', 'base_revision', 'base_variant', 'comparison')
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
            with self._management_lock():
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
        from .deletion import _staged_path
        return sorted({p.parent.resolve() for root in self._roots()
                       for p in root.rglob('runs/*/status.json')
                       if not _staged_path(p)})

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

    def _telemetry_path(self, run_id, stream):
        """Resolve an existing recorded stream within the registered Run boundary."""
        import re
        if not isinstance(stream, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,79}', stream):
            raise ValueError('stream must be a recorded telemetry name')
        directory = self._find_run(run_id) / 'telemetry'
        path = (directory / (stream + '.jsonl')).resolve()
        if not path.is_relative_to(directory.resolve()):
            raise ValueError('telemetry reference escapes Run directory')
        return path

    def iter_telemetry(self, run_id, stream, *, offset=0, limit=1000):
        """Scan original JSONL once in bounded pages, preserving order and provenance."""
        import itertools
        import json
        self._page(offset, limit)
        if limit == 0:
            raise ValueError('telemetry iteration requires a positive page limit')
        path = self._telemetry_path(run_id, stream)
        with path.open() as source:
            lines = itertools.islice(source, offset, None)
            while True:
                records = [json.loads(line) for line in itertools.islice(lines, limit)]
                if not records:
                    return
                yield {'run_id': run_id, 'stream': stream, 'offset': offset,
                       'records': records, 'source': str(path)}
                offset += len(records)

    def read_telemetry(self, run_id, stream, *, offset=0, limit=100):
        """Read a bounded page of existing JSONL records; no alignment or resampling."""
        self._page(offset, limit)
        if limit:
            page = next(self.iter_telemetry(run_id, stream, offset=offset, limit=limit), None)
            if page is not None:
                return page
        path = self._telemetry_path(run_id, stream)
        # Preserve missing-stream errors and the established empty-page contract.
        with path.open():
            pass
        return {'run_id': run_id, 'stream': stream, 'offset': offset, 'records': [],
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
                  preview_id=None, session_id=None):
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
            with self._management_lock():
                path = self._session_path(session_id) if session_id is not None else None
                if path is not None and not path.is_file():
                    raise KeyError(f'unknown Session ID: {session_id}')
                with self._metadata_lock(path.with_suffix('.lock')) if path else nullcontext():
                    if session_id is not None:
                        self._session(session_id)
                    job = self.jobs.start(
                        self.catalog, experiment_id, revision, output=output,
                        runtime_overrides={'enable_actuation': enable_actuation, 'gui': gui},
                        **({'preview_id': preview_id} if preview_id is not None else {}),
                        **({'session_id': session_id} if session_id is not None else {}))
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
        path = self.directory / 'sessions' / (self._identifier(session_id) + '.json')
        if path.parent.is_symlink() or path.is_symlink():
            raise ValueError('Session metadata must be an owned regular document, not a symlink')
        if path.exists() and not path.is_file():
            raise ValueError('Session metadata must be an owned regular document')
        return path

    @staticmethod
    def _session_token(session):
        import hashlib
        import json
        return hashlib.sha256(json.dumps(
            session, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()).hexdigest()

    def get_session(self, session_id):
        """Return detached metadata and a token covering title, notes and Run references."""
        session = self._session(session_id)
        return {'session': session, 'token': self._session_token(session)}

    def _check_session_token(self, session, expected_token):
        if not isinstance(expected_token, str) or self._session_token(session) != expected_token:
            raise CatalogConflict('Session changed; reload before managing it')

    def rename_session(self, session_id, title, *, expected_token):
        """Change only the display title, with compare-and-swap protection."""
        from .reporting import write_json
        if not isinstance(title, str) or not title.strip():
            raise ValueError('Session title must be nonempty')
        path = self._session_path(session_id)
        if not path.is_file():
            raise KeyError(f'unknown Session ID: {session_id}')
        with self._management_lock(), self._metadata_lock(path.with_suffix('.lock')):
            session = self._session(session_id)
            self._check_session_token(session, expected_token)
            session['metadata']['title'] = title.strip()
            write_json(path, session)
            return {'session': session, 'token': self._session_token(session)}

    def delete_session(self, session_id, *, expected_token):
        """Unlink only the selected Session document; never recurse into Run evidence."""
        with self._management_lock():
            path = self._session_path(session_id)
            if not path.is_file():
                return False
            with self._metadata_lock(path.with_suffix('.lock')):
                if not self._session_path(session_id).is_file():
                    return False
                session = self._session(session_id)
                self._check_session_token(session, expected_token)
                self._assert_no_associated_job(session=session)
                path.unlink()
                return True

    def _session(self, session_id):
        session = self._read(self._session_path(session_id))
        if session is None:
            raise KeyError(f'unknown Session ID: {session_id}')
        expected = {'schema_version', 'session_id', 'created_utc', 'metadata', 'runs'}
        if (not isinstance(session, dict) or not expected <= set(session)
                or set(session) - expected - {'experiments'}
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
        references = session.get('experiments', [])
        if not isinstance(references, list):
            raise ValueError('invalid Session Experiment references')
        seen = set()
        for reference in references:
            if (not isinstance(reference, dict)
                    or set(reference) != {'experiment_id', 'revision', 'sha256'}
                    or type(reference['revision']) is not int or reference['revision'] < 1):
                raise ValueError('invalid Session Experiment reference')
            key = (reference['experiment_id'], reference['revision'])
            try:
                record = self.catalog.get(*key)
            except (KeyError, ValueError, TypeError) as error:
                raise ValueError('Session references an unknown Experiment revision') from error
            if key in seen or record['sha256'] != reference['sha256']:
                raise ValueError('Duplicate or mismatched Session Experiment revision')
            seen.add(key)
        return session

    def add_experiment_to_session(self, session_id, experiment_id, revision, *, expected_token):
        """Append an immutable Catalog reference, without executing or copying a Run."""
        from .reporting import write_json
        if type(revision) is not int or revision < 1:
            raise ValueError('An explicit positive Experiment revision is required')
        path = self._session_path(session_id)
        with self._management_lock(), self._metadata_lock(path.with_suffix('.lock')):
            session = self._session(session_id)
            self._check_session_token(session, expected_token)
            record = self.catalog.get(experiment_id, revision)
            reference = {'experiment_id': experiment_id, 'revision': record['revision'],
                         'sha256': record['sha256']}
            references = session.setdefault('experiments', [])
            if reference not in references:
                references.append(reference)
                write_json(path, session)
            return {'session': session, 'token': self._session_token(session)}

    def list_session_experiments(self, session_id):
        """Resolve pinned names from the Catalog; configurations remain in that owner."""
        result = []
        for reference in self._session(session_id).get('experiments', []):
            record = self.catalog.get(reference['experiment_id'], reference['revision'])
            result.append({**reference, 'name': record['name']})
        return result

    def create_session(self, metadata=None):
        """Persist one small UUID record; no Run evidence or metric copies."""
        from datetime import datetime, timezone
        import uuid
        from .reporting import write_json
        automatic_title = metadata is None
        metadata = {} if metadata is None else deepcopy(metadata)
        if (not isinstance(metadata, dict) or set(metadata) - {'title', 'notes'}
                or any(not isinstance(v, str) for v in metadata.values())):
            raise ValueError('Session metadata supports string title/notes only')
        session = {'schema_version': 1, 'session_id': str(uuid.uuid4()),
                   'created_utc': datetime.now(timezone.utc).isoformat(),
                   'metadata': metadata, 'runs': []}
        path = self._session_path(session['session_id'])
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._management_lock():
            if automatic_title:
                session['metadata']['title'] = self.unique_display_name(
                    'session', [s['metadata'].get('title') for s in self.list_sessions(limit=1000)])
            write_json(path, session)
        return session

    def add_run_to_session(self, session_id, run_id):
        """Append a proven reference under a lock; retries are idempotent."""
        from .reporting import write_json
        path = self._session_path(session_id)
        if not path.is_file():
            raise KeyError(f'unknown Session ID: {session_id}')
        with self._management_lock(), self._metadata_lock(path.with_suffix('.lock')):
            directory = self._find_run(run_id)
            session = self._session(session_id)
            if not any(r['run_id'] == run_id for r in session['runs']):
                session['runs'].append({'run_id': run_id, 'directory': str(directory)})
                write_json(path, session)
        return session

    def list_sessions(self, *, title=None, offset=0, limit=100):
        """Page Session summaries in stable creation order; no duplicate results store."""
        self._page(offset, limit)
        sessions = []
        for path in (self.directory / 'sessions').glob('*.json'):
            try:
                sessions.append(self._session(path.stem))
            except (KeyError, FileNotFoundError):
                # A concurrent metadata deletion may follow the directory snapshot.
                continue
        summaries = [{k: value for k, value in session.items() if k != 'runs'} |
                     {'run_count': len(session['runs'])} for session in sessions
                     if title is None or session['metadata'].get('title') == title]
        ordered = sorted(summaries, key=lambda x: (x['created_utc'], x['session_id']))
        return ordered[offset:offset + limit]

    def get_session_comparison(self, session_id, *, descriptive_metric=None, scope=None):
        """Verify frozen evidence, pair authoritative Conditions and return backend ranks."""
        from .benchmarking import rank_conditions
        from .comparison import (
            comparison_reasons, comparison_views, contract_hash, different_paths,
            metric_comparison_reasons, _metric_evidence,
        )
        session = self._session(session_id)
        rows, checks, reasons, differences, labels = [], [], [], [], {}
        expected_seeds, expected_variants, observed_variants = set(), set(), set()
        modern = True
        for reference in session['runs']:
            try:
                result = self._results(Path(reference['directory']), reference['run_id'])
            except (KeyError, ValueError, OSError) as error:
                result = {**reference, 'available': False, 'metrics': None, 'error': str(error)}
                reasons.append(f"{reference['run_id']}: evidence unavailable: {error}")
            columns = ('run_id', 'available', 'experiment_id', 'revision', 'seed', 'variant',
                       'execution_status', 'acceptance', 'status', 'metrics', 'artifacts', 'error')
            row = {k: result[k] for k in columns if k in result}
            rows.append(row)
            if not result['available']:
                continue
            view = result['comparison']
            try:
                if not view:
                    raise ValueError('comparison evidence unavailable')
                expected = comparison_views(view['original_contract'], view.get('policy'),
                                            view['protected_conditions'])
                if view != expected:
                    raise ValueError('comparison hash/metadata differs from original evidence')
                metadata = result['metadata']
                setup = view['protected_conditions'].get('experiment_setup', {})
                identity = metadata.get('experiment_identity', {})
                declared = bool(identity.get('id') and result.get('variant') and setup.get('seeds'))
                modern = modern and declared
                if view['schema_version'] == 2:
                    if not declared:
                        raise ValueError('frozen Experiment/Variant/seed declaration unavailable')
                    frozen = result['configuration']
                    if (frozen.get('comparison') != view
                            or frozen.get('comparison_contract') != view['original_contract']):
                        raise ValueError('comparison differs from independent frozen metadata/hash evidence')
                    consumers = view['protected_conditions'].get('seed_consumers')
                    if (not isinstance(consumers, dict)
                            or consumers != metadata.get('seed_consumers')
                            or consumers != frozen.get('owner_snapshot', {}).get('seed_consumers')
                            or consumers.get('measurements.target_position.random_seed') != result['seed']):
                        raise ValueError('Run seed does not match frozen owner seed-consumer evidence')
                    if (view['protected_conditions'].get('component_selection')
                            != frozen.get('system', {}).get('selection')):
                        raise ValueError('component identities differ from frozen resolved system')
                    if (metadata.get('experiment_identity') != result['status'].get('experiment_identity')
                            or not identity.get('definition_sha256')):
                        raise ValueError('frozen Experiment identity mismatch')
                variables = view['variables'] if view.get('policy') else {'revision': result.get('revision')}
                condition = contract_hash({'experiment': identity.get('id'),
                                           'variant': result.get('variant'), 'variables': variables})
                # Undeclared historical evidence retains its existing strict comparison;
                # it cannot establish modern matched Condition identity for ranking.
                if not declared:
                    condition = result['run_id']
                row['condition_id'] = condition
                labels[condition] = {'experiment_id': result.get('experiment_id'),
                                     'variant': result.get('variant'), 'variables': view['variables']}
                expected_seeds.update(setup.get('seeds', [result['seed']]))
                expected_variants.update(setup.get('variants', []))
                observed_variants.add(result.get('variant'))
                check = {'variant': condition, 'run_id': result['run_id'], 'seed': result['seed'],
                         'execution_status': result['status'].get('execution_status'),
                         'comparison_contract': view['original_hash'], 'comparison': view,
                         'metrics': result['metrics'], 'acceptance': result.get('acceptance'),
                         'status': result['status']}
                frozen = result['configuration']
                check['frozen_consistent'] = (
                    frozen.get('comparison') == view
                    and frozen.get('comparison_contract') == view['original_contract'])
                checks.append(check)
                reasons.extend(f"{result['run_id']}: {reason}"
                               for reason in _metric_evidence(check, view))
            except (ValueError, KeyError, TypeError) as error:
                reasons.append(f"{result['run_id']}: invalid comparison evidence: {error}")
        variants = list(dict.fromkeys(c['variant'] for c in checks))
        if len(checks) < 2:
            reasons.append('at least two Runs with comparison evidence are required')
        if expected_variants - observed_variants:
            reasons.append('missing declared Variants: ' + ', '.join(sorted(expected_variants - observed_variants)))
        policy = checks[0]['comparison'].get('policy') if checks else None
        if any(c['comparison'].get('policy') != policy for c in checks):
            reasons.append('comparison policies differ')
            policy = None
        seeds = sorted(expected_seeds)
        reasons.extend(comparison_reasons(checks, variants, seeds, policy))
        if checks:
            for check in checks[1:]:
                first = next((c for c in checks if c['seed'] == check['seed']), checks[0])
                reasons.extend(f"{check['run_id']}: {reason}"
                               for reason in metric_comparison_reasons(first, check))
                paths = {field: different_paths(first['comparison'][field], check['comparison'][field])
                         for field in ('original_contract', 'protected_conditions')}
                differences.append({'first_run': first['run_id'], 'second_run': check['run_id'], **paths})
                field = 'invariant_conditions' if check['comparison']['schema_version'] == 2 else 'protected_conditions'
                # New declarations project exact mirrors; raw differences remain visible above.
                protected = different_paths(first['comparison'].get(field), check['comparison'].get(field))
                if protected:
                    reasons.append(f"{check['run_id']}: protected condition differences: " + ', '.join(protected))
        reasons = list(dict.fromkeys(reasons))
        ranking_reasons = list(reasons)
        if not modern:
            ranking_reasons.append('authoritative frozen Condition/seed declaration unavailable for historical Runs')
        if policy and policy.get('schema_version') == 2 and len({
                contract_hash(c['comparison']['variables']) for c in checks}) < 2:
            ranking_reasons.append('at least two distinct declared experimental variable values are required')
        ranking = rank_conditions(checks, variants, seeds, policy, reasons=ranking_reasons)
        for condition in ranking['conditions']:
            condition.update(labels[condition['condition_id']])
        result = {'session': session, 'runs': rows,
                'ranking': ranking if ranking['eligible'] else None,
                'ranking_diagnostics': ranking,
                'comparability': {'comparable': not reasons, 'reasons': reasons,
                                  'differences': differences,
                                  'scope': 'Frozen protected conditions and matched Seeds; execution, quality and physical safety are separate. Matched Seeds do not imply identical physical trajectories.'}}
        if descriptive_metric is not None:
            from .benchmarking import rank_descriptive_runs
            result['descriptive_ranking'] = rank_descriptive_runs(checks, descriptive_metric, scope=scope)
            for row in rows:
                if row['run_id'] not in {c['run_id'] for c in checks}:
                    result['descriptive_ranking']['excluded'][row['run_id']] = [
                        'verified frozen comparison evidence unavailable']
        return result
