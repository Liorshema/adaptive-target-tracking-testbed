"""Workspace mathematical definitions; revisions freeze source, D3 model and mapping."""

import hashlib
import json
import re
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

from robot_configuration.resources import package_config_path
from robot_configuration.yaml_loader import load_yaml
from robot_configuration.yaml_store import atomic_write_yaml, locked_yaml

from robot_simulation.dynamics import equation_model
from robot_simulation.dynamics.equation_model import EquationDynamics, _mapping
from robot_simulation.dynamics.motion_composition import (
    compile_definition,
    definition_data,
    dynamics_templates,
)

from .catalog import CatalogConflict


def content_hash(value):
    """Hash deterministic finite data, independently of display aliases or timestamps."""
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    ).hexdigest()


def _identifier(identifier):
    if not isinstance(identifier, str) or not re.fullmatch(
        r'DYN-[A-Z0-9][A-Z0-9_-]{0,70}', identifier
    ):
        raise ValueError('Dynamics ID must be DYN- followed by a stable uppercase identifier')


def _name(name):
    if not isinstance(name, str) or not name.strip() or len(name) > 256:
        raise ValueError('Display/creator name must be nonempty and at most 256 characters')
    return name.strip()


def _revision(number):
    if type(number) is not int or number < 1:
        raise ValueError('An explicit positive integer revision is required')


def validate_dynamics_source(source, model):
    """Check a frozen reference's internal consistency without resolving mutable storage."""
    _mapping(
        source,
        {'library_id', 'revision', 'sha256', 'model_sha256', 'symbol_map'},
        {'library_id', 'revision', 'sha256', 'model_sha256', 'symbol_map'},
        'dynamics_source',
    )
    _identifier(source['library_id'])
    _revision(source['revision'])
    for key in ('sha256', 'model_sha256'):
        if not isinstance(source[key], str) or not re.fullmatch(r'[0-9a-f]{64}', source[key]):
            raise ValueError('Dynamics source hashes must be SHA-256 hex strings')
    if not isinstance(source['symbol_map'], dict) or source['model_sha256'] != content_hash(model):
        raise ValueError('Pinned dynamics source differs from the authored canonical model')
    return deepcopy(source)


class DynamicsLibrary:
    """A single versioned YAML authority; reading or constructing never creates files."""

    def __init__(self, path):
        """Locate one user authority while refusing installed owner data paths."""
        self.path = Path(path).expanduser().resolve()
        protected = package_config_path(
            equation_model.__file__, 'robot_simulation', 'dynamics.yaml'
        )
        templates = package_config_path(
            equation_model.__file__, 'robot_simulation', 'dynamics_templates.yaml'
        )
        if self.path in (protected.resolve(), templates.resolve()):
            raise ValueError('Installed dynamics defaults/templates cannot be a user library')

    def _read(self):
        if not self.path.exists():
            return {'schema_version': 1, 'generation': 0, 'models': {}}

        def plain(data, path):
            _mapping(
                data,
                {'schema_version', 'generation', 'models'},
                {'schema_version', 'generation', 'models'},
                'library',
            )
            return data

        state = load_yaml(self.path, transform=plain)
        if (
            type(state['schema_version']) is not int
            or state['schema_version'] != 1
            or type(state['generation']) is not int
            or state['generation'] < 0
            or not isinstance(state['models'], dict)
        ):
            raise ValueError('Invalid dynamics library schema/generation')
        for identifier, entry in state['models'].items():
            _identifier(identifier)
            _mapping(entry, {'name', 'revisions'}, {'name', 'revisions'}, 'library model')
            _name(entry['name'])
            if not isinstance(entry['revisions'], list) or not entry['revisions']:
                raise ValueError('Library model must contain immutable revisions')
            for number, record in enumerate(entry['revisions'], 1):
                _mapping(
                    record,
                    {
                        'revision',
                        'created_utc',
                        'created_by',
                        'compiler_version',
                        'definition',
                        'model',
                        'symbol_map',
                        'sha256',
                    },
                    {
                        'revision',
                        'created_utc',
                        'created_by',
                        'compiler_version',
                        'definition',
                        'model',
                        'symbol_map',
                        'sha256',
                    },
                    'library revision',
                )
                if (
                    type(record['revision']) is not int
                    or record['revision'] != number
                    or type(record['compiler_version']) is not int
                    or record['compiler_version'] != 1
                ):
                    raise ValueError('Invalid dynamics revision/compiler version')
                _name(record['created_by'])
                if (
                    not isinstance(record['created_utc'], str)
                    or datetime.fromisoformat(record['created_utc']).utcoffset() is None
                ):
                    raise ValueError('Revision creation must have an explicit timezone')
                payload = {k: record[k] for k in ('definition', 'model', 'symbol_map')}
                if not isinstance(record['definition'], dict) or not isinstance(
                    record['symbol_map'], dict
                ):
                    raise ValueError('Revision source and symbol mapping must be mappings')
                if record['sha256'] != content_hash(payload):
                    raise ValueError('Dynamics revision content hash mismatch')
                # Validate frozen canonical data, never reinterpret historical
                # composition source through a later compiler implementation.
                EquationDynamics(model=record['model'])
        return state

    @staticmethod
    def validate(definition):
        """Validate either authoring mode through the same D3 owner without persistence."""
        try:
            compiled = compile_definition(definition_data(definition))
            return {
                'valid': True,
                'errors': [],
                **compiled,
                'capabilities': EquationDynamics(model=compiled['model']).capabilities,
            }
        except (ValueError, TypeError, KeyError) as error:
            return {'valid': False, 'errors': [str(error)]}

    @staticmethod
    def templates():
        """Return owner-provided mathematical starting points, never runtime categories."""
        return dynamics_templates()

    @staticmethod
    def preview(definition, *, duration, dt, context=None, control=None):
        """Inspect bounded owner numerics without creating a Run or environment."""
        from robot_simulation.dynamics.motion_preview import preview_motion

        return preview_motion(
            definition, duration=duration, dt=dt, context=context, control=control
        )

    def list(self):  # noqa: A003 - library API.
        """List mutable labels with immutable latest revision identity for inspection."""
        return [
            {
                'id': key,
                'name': entry['name'],
                'revision': len(entry['revisions']),
                'sha256': entry['revisions'][-1]['sha256'],
            }
            for key, entry in sorted(self._read()['models'].items())
        ]

    def get(self, identifier, revision=None):
        """Read a detached revision; only inspection may request the latest implicitly."""
        _identifier(identifier)
        state = self._read()
        if identifier not in state['models']:
            raise ValueError(f'Unknown dynamics model: {identifier}')
        records = state['models'][identifier]['revisions']
        number = len(records) if revision is None else revision
        _revision(number)
        if number > len(records):
            raise ValueError('Unknown dynamics revision')
        return definition_data({'id': identifier, **records[number - 1]})

    def revisions(self, identifier):
        """Return chronological immutable content and creation metadata."""
        latest = self.get(identifier)
        return [
            {k: r[k] for k in ('id', 'revision', 'sha256', 'created_utc', 'created_by')}
            for r in (self.get(identifier, i) for i in range(1, latest['revision'] + 1))
        ]

    def resolve(self, identifier, revision):
        """Resolve only a pinned canonical revision; it does not depend on mutable latest."""
        _revision(revision)
        record = self.get(identifier, revision)
        return {
            'model': record['model'],
            'source': {
                'library_id': identifier,
                'revision': revision,
                'sha256': record['sha256'],
                'model_sha256': content_hash(record['model']),
                'symbol_map': record['symbol_map'],
            },
        }

    def capabilities(self, identifier, revision):
        """Keep mathematical, numerical-contract and deployment support distinct."""
        return EquationDynamics(model=self.resolve(identifier, revision)['model']).capabilities

    @staticmethod
    def _record(definition, number, created_by):
        definition = definition_data(definition)
        compiled = compile_definition(definition)
        payload = {'definition': deepcopy(definition), **compiled}
        return {
            'revision': number,
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'created_by': _name(created_by),
            'compiler_version': 1,
            **payload,
            'sha256': content_hash(payload),
        }

    def create(self, identifier, name, definition, *, created_by='local'):
        """Create an independent stable identity; never overwrite an existing entry."""
        _identifier(identifier)
        name = _name(name)
        record = self._record(definition, 1, created_by)
        with locked_yaml(self.path):
            state = self._read()
            if identifier in state['models']:
                raise CatalogConflict('Dynamics ID already exists')
            state['models'][identifier] = {'name': name, 'revisions': [record]}
            state['generation'] += 1
            atomic_write_yaml(self.path, state)
        return deepcopy({'id': identifier, **record})

    def revise(self, identifier, definition, *, expected_revision, created_by='local',
               name=None, expected_name=None):
        """Append after compare-and-swap; validation failure does not alter the library."""
        _identifier(identifier)
        _revision(expected_revision)
        name = _name(name) if name is not None else None
        record = self._record(definition, expected_revision + 1, created_by)
        with locked_yaml(self.path):
            state = self._read()
            entry = state['models'].get(identifier)
            if entry is None or len(entry['revisions']) != expected_revision:
                raise CatalogConflict('Dynamics changed or disappeared; reload before revising')
            if expected_name is not None and entry['name'] != expected_name:
                raise CatalogConflict('Dynamics label changed; reload before revising')
            if name is not None:
                entry['name'] = name
            entry['revisions'].append(record)
            state['generation'] += 1
            atomic_write_yaml(self.path, state)
        return deepcopy({'id': identifier, **record})

    def rename(self, identifier, name, *, expected_revision, expected_name):
        """Change a display alias only, guarding both the revision and selected label."""
        _identifier(identifier)
        _revision(expected_revision)
        name = _name(name)
        with locked_yaml(self.path):
            state = self._read()
            entry = state['models'].get(identifier)
            if (
                entry is None
                or len(entry['revisions']) != expected_revision
                or entry['name'] != expected_name
            ):
                raise CatalogConflict('Dynamics label/revision changed; reload before renaming')
            entry['name'] = name
            state['generation'] += 1
            atomic_write_yaml(self.path, state)
        return {'id': identifier, 'name': name, 'revision': expected_revision}
