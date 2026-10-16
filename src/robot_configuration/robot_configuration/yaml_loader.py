
"""Utilities for loading external simulation configuration."""

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    """
    Load a mapping, optionally inheriting one file with ``extends``.

    Relative parent paths are resolved from the child file's directory.
    Nested mappings merge recursively; lists, scalars, and null replace the
    inherited value. The top-level ``extends`` key is consumed, and ordinary
    documents retain their existing loading behavior. Cycles and non-mapping
    documents are rejected.
    """
    return _load_yaml(Path(path).resolve(), ())


def _load_yaml(
    config_path: Path,
    ancestors: tuple[Path, ...],
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
    if 'extends' not in data:
        return data

    parent = data.pop('extends')
    if not isinstance(parent, str) or not parent.strip():
        raise ValueError(
            f'Configuration extends must name one file: {config_path}'
        )

    parent_path = (config_path.parent / parent).resolve()
    inherited = _load_yaml(parent_path, (*ancestors, config_path))
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
