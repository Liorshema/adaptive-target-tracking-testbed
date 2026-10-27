"""
Materialize explicit catalog groups into plain YAML for existing owner readers.

Generated files are content addressed within one lazily created process-lifetime
temporary directory. Its retained TemporaryDirectory removes files on process
exit; callers persisting runs must copy effective documents into run inputs.
No source file is modified and ordinary references retain their original path.
"""

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from robot_configuration.parameter_validation import override_parameters
from robot_configuration.resources import resolve_resource
from robot_configuration.yaml_loader import _merge_mappings
from robot_configuration.yaml_loader import load_yaml
from robot_configuration.yaml_loader import load_yaml_group
import yaml

_temporary = None
_sources = {}
_labels = {}
_fingerprints = {}


@lru_cache(maxsize=128)
def _cached_group(path, group, fingerprint):
    return load_yaml_group(path, list(group) if isinstance(group, tuple) else group)


def load_resource_group(path, group=None):
    """Read detached groups; cache only self-contained files by their exact bytes."""
    path = Path(path).resolve()
    contents = path.read_bytes()
    # Inherited documents must reread their parents; bypass caching conservatively.
    if b'extends' in contents:
        return load_yaml_group(path, group)
    selected = tuple(group) if isinstance(group, list) else group
    return deepcopy(_cached_group(path, selected, hashlib.sha256(contents).digest()))


def resource_sources(path):
    """Return detached JSON-compatible original path/group provenance records."""
    path = Path(path).resolve()
    for original, expected in _fingerprints.get(path, {}).items():
        if not original.is_file() or hashlib.sha256(original.read_bytes()).hexdigest() != expected:
            raise ValueError(f'{original}: source changed since preparation; prepare again')
    return deepcopy(_sources.get(path, [{'path': str(path)}]))


def materialize_resource(reference, base, share):
    """
    Resolve a plain reference or materialize an explicit package/path/group.

    A selected mapping is either an ordinary document or an exact
    ``{base: resource, overrides: mapping, selection?: mapping}`` recipe. Selection
    replaces existing typed component slots atomically before strict existing
    parameter-leaf overrides; owner factories retain physical validation.
    Explicit nested resource mappings become filenames. Schema-2 system owners
    retain their package/group mappings, except experiment catalog references
    which are expanded recursively. Group lists use load_yaml_group semantics.
    """
    return _materialize(reference, Path(base), share, ())


def _reference(reference, base, share):
    if isinstance(reference, Path):
        reference = str(reference)
    group = None
    if isinstance(reference, dict):
        if set(reference) not in ({'package', 'path'}, {'package', 'path', 'group'}):
            raise ValueError(f'Invalid resource reference: {reference!r}')
        group = reference.get('group')
        if 'group' in reference and group is None:
            raise ValueError('Resource group must be an explicit name or group list')
        reference = {key: reference[key] for key in ['package', 'path']}
    return resolve_resource(reference, base, share), group


def _append(records, additions):
    for record in additions:
        if record not in records:
            records.append(deepcopy(record))


def _materialize(reference, base, share, ancestors):
    global _temporary
    if isinstance(reference, dict) and 'base' in reference:
        identity = ('inline', id(reference))
        if identity in ancestors:
            raise ValueError('Inline composition recipe cycle')
        records = []
        value = _apply_recipe(
            reference, Path(base) / '<inline>', None, share, (*ancestors, identity), records
        )
        value = _expand(
            value,
            base,
            share,
            (*ancestors, identity),
            records,
            preserve_owners=value.get('schema_version') == 2,
        )
        return _store_document(value, records)
    path, group = _reference(reference, base, share)
    if group is None:
        return path
    # Let the existing loader validate group names and list structure first.
    value = load_resource_group(path, group)
    identity = (path, tuple(group) if isinstance(group, list) else group)
    if identity in ancestors:
        raise ValueError(f'Catalog resource recipe cycle: {path}:{group!r}')
    ancestors = (*ancestors, identity)
    records = [{'path': str(path), 'group': deepcopy(group)}]
    if b'extends' in path.read_bytes():
        groups = group if isinstance(group, list) else [group]
        system_recipes = set()

        def normalize(document, origin):
            _append(records, [{'path': str(origin)}])
            for selected in groups:
                parent = document
                keys = selected.split('.')
                for key in keys[:-1]:
                    parent = parent.get(key, {}) if isinstance(parent, dict) else {}
                if not isinstance(parent, dict) or not isinstance(parent.get(keys[-1]), dict):
                    continue
                declared = parent[keys[-1]]
                if 'base' in declared:
                    owner = _materialize(declared['base'], origin.parent, share, ancestors)
                    _append(records, resource_sources(owner))
                    declared['base'] = str(owner)
                if selected in system_recipes and isinstance(declared.get('overrides'), dict):
                    settings = _expand(declared['overrides'], origin.parent, share, ancestors,
                                       records, preserve_owners=True)
                    declared['overrides'] = _system_references(
                        settings, origin, share, replace_resources=True)
                if value.get('schema_version') == 2:
                    expanded = _expand(declared, origin.parent, share, ancestors,
                                       records, preserve_owners=True)
                    parent[keys[-1]] = _system_references(expanded, origin, share)
            return document

        inherited = load_yaml(path, transform=normalize)
        # A recipe's base determines whether its overrides declare system owners.
        # Resolve those declarations before inheritance, just as M07 does for files.
        for selected in groups:
            selected_value = inherited
            for key in selected.split('.'):
                selected_value = selected_value[key]
            if isinstance(selected_value, dict) and 'base' in selected_value:
                owner = _materialize(selected_value['base'], path.parent, share, ancestors)
                if load_yaml(owner).get('schema_version') == 2:
                    system_recipes.add(selected)
        if system_recipes:
            inherited = load_yaml(path, transform=normalize)
        value = {}
        for selected in groups:
            selected_value = inherited
            for key in selected.split('.'):
                selected_value = selected_value[key]
            value = _merge_mappings(value, selected_value)
    if {'base', 'overrides', 'selection', 'omit'} & value.keys():
        value = _apply_recipe(value, path, group, share, ancestors, records)
    value = _expand(
        value,
        path.parent,
        share,
        ancestors,
        records,
        preserve_owners=value.get('schema_version') == 2,
    )
    if value.get('schema_version') == 2:
        value = _system_references(value, path, share)
    return _store_document(value, records)


def _apply_recipe(value, path, group, share, ancestors, records):
    if (
        not {'base', 'overrides'} <= value.keys()
        or set(value) - {'base', 'overrides', 'selection', 'omit'}
        or not isinstance(value['overrides'], dict)
    ):
        raise ValueError(f'{path}:{group!r}: recipe requires base, overrides, optional selection')
    value_recipe_omit = value.get('omit', [])
    owner = _materialize(value['base'], path.parent, share, ancestors)
    _append(records, resource_sources(owner))

    def capture(document, origin):
        _append(records, resource_sources(origin))
        return document

    defaults = load_yaml(owner, transform=capture)
    if defaults.get('schema_version') == 2:

        def normalize(document, origin):
            _append(records, resource_sources(origin))
            expanded = _expand(
                document, origin.parent, share, ancestors, records, preserve_owners=True
            )
            return _system_references(expanded, origin, share)

        defaults = load_yaml(owner, transform=normalize)
    settings = _expand(
        value['overrides'],
        path.parent,
        share,
        ancestors,
        records,
        preserve_owners=defaults.get('schema_version') == 2,
    )
    if defaults.get('schema_version') == 2:
        settings = _system_references(settings, path, share, replace_resources=True)
    defaults = _select(defaults, value.get('selection', {}), f'{path}:{group!r}.selection')
    value = (
        _merge_mappings(defaults, settings)
        if defaults.get('schema_version') == 2
        else override_parameters(defaults, settings, path=f'{path}:{group!r}.overrides')
    )
    omitted = value_recipe_omit
    if (
        not isinstance(omitted, list)
        or any(not isinstance(k, str) or not k for k in omitted)
        or len(set(omitted)) != len(omitted)
    ):
        raise ValueError('Recipe omit must name unique existing fields')
    for dotted in omitted:
        parent = value
        keys = dotted.split('.')
        for key in keys[:-1]:
            if not isinstance(parent, dict) or key not in parent:
                raise ValueError('Recipe omit must name existing fields')
            parent = parent[key]
        if not isinstance(parent, dict) or keys[-1] not in parent:
            raise ValueError('Recipe omit must name existing fields')
        del parent[keys[-1]]
    return value


def _system_references(value, origin, share, *, replace_resources=False):
    # Reuse M07's declaration-origin handling, including inherited group clearing.
    from robot_bringup.factory.system_configuration import EXPLICIT_REFERENCES
    from robot_bringup.factory.system_configuration import REFERENCES
    from robot_bringup.factory.system_configuration import _resolve_references
    from robot_experiments.reporting import plain

    # Explicit package references are already independent of the declaring file.
    # Preserve their public shape and group selection for the owner readers.
    relative = {}
    for dotted in (*REFERENCES, *EXPLICIT_REFERENCES):
        keys = dotted.split('.')
        source, target = value, relative
        for key in keys[:-1]:
            if not isinstance(source, dict) or key not in source:
                break
            source = source[key]
            target = target.setdefault(key, {})
        else:
            candidate = source.get(keys[-1]) if isinstance(source, dict) else None
            if (isinstance(candidate, (str, Path))
                    or (replace_resources and isinstance(candidate, dict)
                        and {'package', 'path'} <= candidate.keys())):
                target[keys[-1]] = deepcopy(candidate)
                group = keys[-1] + '_group'
                if group in source:
                    target[group] = source[group]
    resolved = plain(_resolve_references(relative, origin, share))
    return _merge_mappings(value, resolved)


def resource_label(value):
    """Return a stable display origin for generated documents, never a runtime path."""
    return _labels.get(str(value))


def _logical(value):
    if isinstance(value, dict):
        return {k: _logical(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_logical(v) for v in value]
    return _labels.get(value, value) if isinstance(value, str) else value


def _store_document(value, records):
    global _temporary
    encoded = yaml.safe_dump(value, sort_keys=False).encode()
    if _temporary is None:
        _temporary = TemporaryDirectory(prefix='robot_composition_')
    fingerprints = {
        Path(r['path']).resolve(): hashlib.sha256(Path(r['path']).read_bytes()).hexdigest()
        for r in records
    }
    identity = json.dumps(
        {
            'value': _logical(value),
            'sources': records,
            'fingerprints': {str(k): v for k, v in fingerprints.items()},
        },
        sort_keys=True,
    ).encode()
    digest = hashlib.sha256(identity).hexdigest()
    generated = Path(_temporary.name) / (digest + '.yaml')
    if not generated.exists():
        generated.write_bytes(encoded)
    _sources[generated] = deepcopy(records)
    _fingerprints[generated] = fingerprints
    first = records[0] if records else {'path': '<inline>'}
    _labels[str(generated)] = (
        first['path'] + '#' + str(first.get('group', 'resolved')) + '@' + digest[:12]
    )
    return generated


def _select(document, selections, label):
    if not isinstance(selections, dict):
        raise ValueError(f'{label}: expected mapping')
    for dotted, selected in selections.items():
        if (
            not isinstance(dotted, str)
            or not dotted
            or any(not key for key in dotted.split('.'))
            or not isinstance(selected, dict)
            or set(selected) != {'type', 'parameters'}
            or not isinstance(selected['type'], str)
            or not selected['type'].strip()
            or not isinstance(selected['parameters'], dict)
        ):
            raise ValueError(f'{label}: invalid typed component selection {dotted!r}')
        parent = document
        keys = dotted.split('.')
        for key in keys[:-1]:
            if not isinstance(parent, dict) or key not in parent:
                raise ValueError(f'{label}: unknown selection path {dotted!r}')
            parent = parent[key]
        if not isinstance(parent, dict) or keys[-1] not in parent:
            raise ValueError(f'{label}: unknown selection path {dotted!r}')
        current = parent[keys[-1]]
        if not isinstance(current, dict) or (
            current
            and (
                set(current) != {'type', 'parameters'}
                or not isinstance(current['type'], str)
                or not isinstance(current['parameters'], dict)
            )
        ):
            raise ValueError(
                f'{label}: selection path must name a typed component slot {dotted!r}'
            )
        parent[keys[-1]] = deepcopy(selected)
    return document


def _expand(value, base, share, ancestors, records, *, preserve_owners):
    if isinstance(value, dict):
        if 'base' in value and 'overrides' in value:
            target = _materialize(value, base, share, ancestors)
            _append(records, resource_sources(target))
            return str(target)
        if 'package' in value and 'path' in value:
            # Validate exact reference structure even for preserved owner refs.
            if set(value) not in ({'package', 'path'}, {'package', 'path', 'group'}):
                raise ValueError(f'Invalid resource reference: {value!r}')
            if preserve_owners and value['package'] != 'robot_experiments':
                return deepcopy(value)
            target = _materialize(value, base, share, ancestors)
            _append(records, resource_sources(target))
            return str(target)
        return {
            key: _expand(child, base, share, ancestors, records, preserve_owners=preserve_owners)
            for key, child in value.items()
        }
    if isinstance(value, list):
        return [
            _expand(child, base, share, ancestors, records, preserve_owners=preserve_owners)
            for child in value
        ]
    return deepcopy(value)


def load_resource(reference, *, base=None, share=None):
    """Read current or explicitly migrated experiment resources with owner provenance."""
    if share is None:
        from robot_experiments.catalog import package_share

        share = package_share
    from robot_experiments.configuration import resolve_system_resource

    return load_yaml(resolve_system_resource(reference, base or Path.cwd(), share))
