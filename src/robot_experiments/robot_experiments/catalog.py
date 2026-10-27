"""
Experiment definitions, immutable revisions and detached M08 preparation.

The catalog owns identity and editing. Existing M07/M08 code owns configuration
interpretation, mathematical validation, execution and scientific comparison.
"""
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime
from datetime import timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile

from robot_configuration.yaml_loader import load_yaml
import yaml

from .configuration import load_experiment
from .configuration import materialize_run
from .configuration import resolve_system_resource


class CatalogConflict(ValueError):
    """A writer attempted to replace a revision it did not read."""


def definition_hash(definition, overrides):
    value = {'definition': definition, 'overrides': overrides}
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def package_share(package):
    """Resolve installed resources lazily; offline users inject their resolver."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


class ExperimentCatalog:
    """Locked atomic edits with compare-and-swap revision protection."""

    def __init__(self, path=None, share=None):
        self.share = share or package_share
        self.builtin = Path(self.share('robot_experiments'))/'config/experiments.yaml'
        self.path = Path(path or self.builtin).resolve()

    def _read(self):
        state = load_yaml(self.path)
        required = {'schema_version', 'generation', 'experiments', 'retired_ids'}
        optional = {'compositions', 'resources', 'direct_launch', 'legacy_resources'}
        if (set(state) - (required | optional) or not required <= set(state)
                or type(state['schema_version']) is not int or state['schema_version'] != 1):
            raise ValueError('catalog: expected schema_version 1, generation, experiments')
        for section in ('compositions', 'resources', 'direct_launch', 'legacy_resources'):
            if section in state and not isinstance(state[section], dict):
                raise ValueError(f'catalog: {section} must be a mapping')
        if (type(state['generation']) is not int or state['generation'] < 0
                or not isinstance(state['experiments'], dict)):
            raise ValueError('catalog: invalid generation/experiments')
        if (not isinstance(state['retired_ids'], list)
                or len(state['retired_ids']) != len(set(state['retired_ids']))):
            raise ValueError('catalog: invalid retired IDs')
        for identifier in state['retired_ids']:
            self._identifier(identifier)
        for identifier, entry in state['experiments'].items():
            self._identifier(identifier)
            if (set(entry) != {'name', 'legacy_entry', 'deleted', 'revisions'}
                    or not isinstance(entry['name'], str) or not entry['name'].strip()):
                raise ValueError(f'{identifier}: invalid catalog entry')
            if (type(entry['deleted']) is not bool
                    or not isinstance(entry['revisions'], list) or not entry['revisions']):
                raise ValueError(f'{identifier}: invalid revision history')
            for number, revision in enumerate(entry['revisions'], 1):
                if (set(revision) != {
                        'revision', 'created_utc', 'definition', 'overrides', 'sha256'}
                        or type(revision['revision']) is not int
                        or revision['revision'] != number):
                    raise ValueError(f'{identifier}: invalid revision sequence')
                if revision['sha256'] != definition_hash(
                        revision['definition'], revision['overrides']):
                    raise ValueError(f'{identifier}: revision fingerprint mismatch')
        return state

    @staticmethod
    def _identifier(identifier):
        if (not isinstance(identifier, str)
                or not re.fullmatch(r'EXP-[A-Z0-9][A-Z0-9_-]{0,70}', identifier)):
            raise ValueError(
                'experiment ID must be EXP- followed by a stable uppercase identifier')

    @contextmanager
    def _locked(self):
        if self.path == self.builtin.resolve():
            raise ValueError('installed catalog is read-only; initialize a writable catalog first')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.with_suffix(self.path.suffix+'.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)

    def _write(self, state):
        encoded = yaml.safe_dump(state, sort_keys=False)
        fd, name = tempfile.mkstemp(prefix='.'+self.path.name, dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w') as stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self.path)
            directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if Path(name).exists():
                Path(name).unlink()

    def initialize(self):
        """Copy immutable shipped definitions without modifying package resources."""
        with self._locked():
            if self.path.exists():
                raise CatalogConflict('catalog already exists')
            self._write(load_yaml(self.builtin))
        return self.list()

    def list(self, *, include_deleted=False):  # noqa: A003 - preserve the public catalog API.
        return [{'id': key, 'name': value['name'], 'revision': len(value['revisions']),
                 'deleted': value['deleted']} for key, value in self._read()['experiments'].items()
                if include_deleted or not value['deleted']]

    def get(self, identifier, revision=None):
        self._identifier(identifier)
        entry = self._read()['experiments'][identifier]
        number = len(entry['revisions']) if revision is None else revision
        if type(number) is not int or not 1 <= number <= len(entry['revisions']):
            raise ValueError('revision: expected existing positive integer')
        return deepcopy({'id': identifier, 'name': entry['name'], **entry['revisions'][number-1]})

    def entry_id(self, name):
        """Explicit migration map for former installed M08 entry filenames."""
        matches = [key for key, value in self._read()['experiments'].items()
                   if value['legacy_entry'] == name]
        if len(matches) != 1:
            raise ValueError(f'unknown historical experiment entry: {name}')
        return matches[0]

    def document(self, name):
        """Return an independent imported definition for migration/tests/editors."""
        return self.get(name if name.startswith('EXP-') else self.entry_id(name))['definition']

    def _prepare(self, record, output=None, runtime_overrides=None):
        definition, overrides = deepcopy(record['definition']), deepcopy(record['overrides'])
        if not isinstance(definition, dict) or set(definition) != {'schema_version', 'experiment'}:
            raise ValueError(
                'catalog definition: expected self-contained M08 schema_version/experiment')
        if not isinstance(overrides, dict):
            raise ValueError('overrides: expected owner mapping')
        # Declaring origin stays the catalog, not the temporary validation file.
        ex = definition['experiment']
        if not isinstance(ex, dict):
            raise ValueError('experiment: expected mapping')
        for node in [ex, *ex.get('variants', [])]:
            if isinstance(node, dict) and 'system_config' in node:
                node['system_config'] = str(resolve_system_resource(
                    node['system_config'], self.path.parent, self.share))
        runtime = {} if runtime_overrides is None else runtime_overrides
        if not isinstance(runtime, dict) or set(runtime)-{'enable_actuation'}:
            raise ValueError('runtime overrides: supported enable_actuation only')
        ex['runtime'] = {**ex.get('runtime', {}), **runtime}
        with tempfile.TemporaryDirectory(prefix='experiment_definition_') as temporary:
            path = Path(temporary)/'experiment.yaml'
            path.write_text(yaml.safe_dump(definition))
            spec = load_experiment(path, self.share, output)
            for variant in spec['variants']:
                variant['owner_overrides'] = deepcopy(overrides)
                for origin in variant.get('parameter_sources', {}).values():
                    origin['document'] = str(self.path)
                # Invoke actual seeded materialization/preflight, never a second validator.
                directory = Path(temporary)/variant['name']
                directory.mkdir()
                materialize_run(variant, spec['seeds'][0], directory, self.share,
                                spec['runtime'], spec['execution']['record_actuator_trace'])
        spec['source'] = self.path
        spec['source_ancestry'] = [str(self.path)]
        spec['source_document'] = deepcopy(record['definition'])
        spec['experiment_identity'] = {'id': record['id'], 'revision': record['revision'],
                                       'definition_sha256': record['sha256']}
        spec['experiment_definition'] = deepcopy(record)
        return spec

    def prepare(self, identifier, revision=None, *, output=None, runtime_overrides=None):
        entry = self._read()['experiments'][identifier]
        if entry['deleted']:
            raise ValueError(f'{identifier}: experiment is deleted')
        return self._prepare(self.get(identifier, revision), output, runtime_overrides)

    def draft(self, identifier=None, revision=None):
        if identifier is None:
            # Fresh sessions never inherit another user's unsaved or saved overrides.
            return {'definition': None, 'overrides': {}}
        record = self.get(identifier, revision)
        return {'definition': record['definition'], 'overrides': record['overrides']}

    @staticmethod
    def reset(draft):
        value = deepcopy(draft)
        value['overrides'] = {}
        return value

    def _revision(self, identifier, name, number, definition, overrides):
        record = {'id': identifier, 'name': name, 'revision': number,
                  'created_utc': datetime.now(timezone.utc).isoformat(),
                  'definition': deepcopy(definition), 'overrides': deepcopy(overrides),
                  'sha256': definition_hash(definition, overrides)}
        self._prepare(record)
        return {key: value for key, value in record.items() if key not in {'id', 'name'}}

    def create(self, identifier, name, definition, overrides=None):
        self._identifier(identifier)
        if not isinstance(name, str) or not name.strip():
            raise ValueError('name must be nonempty')
        revision = self._revision(
            identifier, name, 1, definition, {} if overrides is None else overrides)
        with self._locked():
            state = self._read()
            if identifier in state['experiments'] or identifier in state['retired_ids']:
                raise CatalogConflict('experiment ID already exists or was permanently retired')
            state['experiments'][identifier] = {'name': name, 'legacy_entry': None,
                                                'deleted': False, 'revisions': [revision]}
            state['generation'] += 1
            self._write(state)
        return deepcopy({'id': identifier, 'name': name, **revision})

    def revise(self, identifier, definition, overrides=None, *, expected_revision):
        old = self.get(identifier)
        if type(expected_revision) is not int or expected_revision < 1:
            raise ValueError('expected_revision must be a positive integer')
        revision = self._revision(identifier, old['name'], expected_revision+1, definition,
                                  {} if overrides is None else overrides)
        with self._locked():
            state = self._read()
            entry = state['experiments'][identifier]
            if len(entry['revisions']) != expected_revision or entry['deleted']:
                raise CatalogConflict('experiment changed or was deleted; reload before revising')
            entry['revisions'].append(revision)
            state['generation'] += 1
            self._write(state)
        return deepcopy({'id': identifier, 'name': old['name'], **revision})

    def delete(self, identifier, *, expected_revision):
        with self._locked():
            state = self._read()
            entry = state['experiments'][identifier]
            if len(entry['revisions']) != expected_revision:
                raise CatalogConflict('experiment changed')
            entry['deleted'] = True
            state['generation'] += 1
            self._write(state)

    def purge(self, identifier, *, expected_revision):
        """Explicit permanent definition deletion; historical result files stay untouched."""
        with self._locked():
            state = self._read()
            entry = state['experiments'][identifier]
            if not entry['deleted'] or len(entry['revisions']) != expected_revision:
                raise CatalogConflict(
                    'soft-delete the expected revision before permanent deletion')
            del state['experiments'][identifier]
            state['retired_ids'].append(identifier)
            state['generation'] += 1
            self._write(state)

    def explain(self, identifier, revision=None, **kwargs):
        from .explain import explain_experiment
        return explain_experiment(self.prepare(identifier, revision, **kwargs), self.share)

    def run(self, identifier, revision=None, **kwargs):
        from .runner import run_experiment
        return run_experiment(self.prepare(identifier, revision, **kwargs), self.share)
