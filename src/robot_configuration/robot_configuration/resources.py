"""Resolve explicit package resources or paths with injected package lookup."""

from collections.abc import Mapping
import os
from pathlib import Path
import re


def resolve_resource(reference, base_directory, package_share):
    """Resolve a file; forbid lexical package traversal, allowing installed symlinks."""
    if isinstance(reference, str) and reference.strip():
        path = (Path(base_directory) / reference).resolve()
    elif isinstance(reference, Mapping) and set(reference) == {'package', 'path'}:
        package, relative = reference['package'], reference['path']
        if (
            not isinstance(package, str)
            or not re.fullmatch(r'[a-z][a-z0-9_]*', package)
            or not isinstance(relative, str)
            or not relative.strip()
            or Path(relative).is_absolute()
        ):
            raise ValueError(f'Invalid package resource: {reference!r}')
        try:
            share = Path(package_share(package)).absolute()
        except Exception as error:
            raise ValueError(f'Package resource unavailable: {reference!r}: {error}') from error
        path = Path(os.path.normpath(share / relative))
        if not path.is_relative_to(share):
            raise ValueError(f'Package resource escapes share: {reference!r}')
        path = path.resolve()
    else:
        raise ValueError(f'Resource must be a path or {{package, path}} mapping: {reference!r}')
    if not path.is_file():
        raise ValueError(f'Resource file does not exist: {reference!r} -> {path}')
    return path


def package_config_path(module_file, package, filename):
    """Locate owner data beside source or under an ancestor's installed share."""
    if Path(filename).name != filename or not filename:
        raise ValueError('Configuration filename must be a simple basename')
    for parent in Path(module_file).resolve().parents:
        candidates = [parent / 'share' / package / 'config' / filename]
        if parent.name == package:
            candidates.append(parent / 'config' / filename)
        for candidate in candidates:
            if candidate.is_file():
                return candidate
    raise ValueError(f'{package}: packaged configuration unavailable: {filename}')
