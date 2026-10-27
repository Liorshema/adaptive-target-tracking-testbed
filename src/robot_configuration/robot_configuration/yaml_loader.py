
"""Utilities for loading external simulation configuration."""

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path, *, transform=None) -> dict[str, Any]:
    """
    Load a mapping, optionally inheriting one file with ``extends``.

    Relative parent paths are resolved from the child file's directory.
    Nested mappings merge recursively; lists, scalars, and null replace the
    inherited value. The top-level ``extends`` key is consumed, and ordinary
    documents retain their existing loading behavior. Cycles and non-mapping
    documents are rejected.

    A caller may transform each document with ``transform(mapping, source_path)``
    before inheritance merging, for example to preserve relative resource origins.
    The loader itself has no knowledge of resource schemas or package policies.
    """
    return _load_yaml(Path(path).resolve(), (), transform)


def _load_yaml(
    config_path: Path,
    ancestors: tuple[Path, ...],
    transform=None,
) -> dict[str, Any]:
    """Resolve one inheritance chain while detecting canonical-path cycles."""
    if config_path in ancestors:
        chain = ' -> '.join(str(path) for path in (*ancestors, config_path))
        raise ValueError(f'Configuration inheritance cycle: {chain}')

    with config_path.open(
        mode='r',
        encoding='utf-8',
    ) as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError(
                f'Invalid YAML configuration: {config_path}'
            ) from error

    if not isinstance(data, dict):
        raise ValueError(
            f'Configuration must be a mapping: {config_path}'
        )

    _reject_recursive_aliases(data, set())
    has_parent = 'extends' in data
    parent = data.pop('extends', None)
    if transform is not None:
        data = transform(data, config_path)
        if not isinstance(data, dict):
            raise ValueError(f'Document transform must return a mapping: {config_path}')
    if not has_parent:
        return data
    if not isinstance(parent, str) or not parent.strip():
        raise ValueError(
            f'Configuration extends must name one file: {config_path}'
        )

    parent_path = (config_path.parent / parent).resolve()
    inherited = _load_yaml(parent_path, (*ancestors, config_path), transform)
    return _merge_mappings(inherited, data)


def _merge_mappings(
    base: dict[str, Any],
    override: dict[str, Any],
) -> dict[str, Any]:
    """Merge a child mapping without appending lists or mutating the base."""
    merged = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_mappings(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def _reject_recursive_aliases(value: Any, ancestors: set[int]) -> None:
    """Reject recursive YAML containers while permitting ordinary aliases."""
    if not isinstance(value, (dict, list)):
        return
    identity = id(value)
    if identity in ancestors:
        raise ValueError('Configuration contains a recursive YAML alias')
    ancestors.add(identity)
    children = value.values() if isinstance(value, dict) else value
    for child in children:
        _reject_recursive_aliases(child, ancestors)
    ancestors.remove(identity)


def load_yaml_group(path: str | Path, group: str | list[str] | None = None) -> dict[str, Any]:
    """Load an explicitly named mapping group without selecting a default group."""
    data = load_yaml(path)
    if group is None:
        return data
    if isinstance(group, list):
        if (
            not group
            or any(not isinstance(name, str) for name in group)
            or len(set(group)) != len(group)
        ):
            raise ValueError(f'{path}: group list must contain unique nonempty names')
        merged = {}
        for name in group:
            merged = _merge_mappings(merged, load_yaml_group(path, name))
        return merged
    if not isinstance(group, str) or not group or any(not key for key in group.split('.')):
        raise ValueError(f'{path}: group must be a nonempty dotted name')
    for key in group.split('.'):
        if not isinstance(data, dict) or key not in data:
            raise ValueError(f'{path}: unknown parameter group {group!r}')
        data = data[key]
    if not isinstance(data, dict):
        raise ValueError(f'{path}: parameter group {group!r} must be a mapping')
    return deepcopy(data)
