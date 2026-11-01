"""Canonical robot resources preserve explicit legacy meaning without installed copies."""

from copy import deepcopy
import hashlib
from pathlib import Path

import pytest
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.composition import resource_sources
from robot_experiments.configuration import resolve_system_resource
import yaml

SRC = Path(__file__).resolve().parents[2]
OLD = 'config/robot/robot.yaml'
NEW = 'config/robots/mobile_manipulator.yaml'


def share(package):
    return SRC / package


def test_canonical_physical_resource_and_owner_default_have_one_installed_location(monkeypatch):
    from ament_index_python import packages
    from robot_description import model_parameters

    canonical = share('robot_description') / NEW
    assert canonical.is_file()
    assert not (share('robot_description') / OLD).exists()
    monkeypatch.setattr(packages, 'get_package_share_directory', lambda name: str(share(name)))
    assert model_parameters.default_robot_description_path() == canonical
    actual = model_parameters.load_robot_model_parameters()
    expected = model_parameters.load_robot_model_parameters(canonical)
    assert actual.base == expected.base
    assert actual.manipulator.joint_axes.tolist() == expected.manipulator.joint_axes.tolist()


@pytest.mark.parametrize('kind', ['package', 'relative', 'absolute'])
def test_exact_historical_physical_path_resolves_to_identical_canonical_bytes(kind):
    canonical = share('robot_description') / NEW
    reference = ({'package': 'robot_description', 'path': OLD} if kind == 'package'
                 else OLD if kind == 'relative' else share('robot_description') / OLD)
    before = deepcopy(reference)
    actual = resolve_system_resource(reference, share('robot_description'), share)
    assert actual == canonical
    alias = load_yaml(share('robot_experiments') / 'config/experiments.yaml')[
        'legacy_resources']['robot_description:' + OLD]
    assert alias['sha256'] == hashlib.sha256(canonical.read_bytes()).hexdigest()
    assert reference == before


def isolated_share(tmp_path):
    physical = tmp_path / 'robot_description' / NEW
    physical.parent.mkdir(parents=True)
    physical.write_bytes((share('robot_description') / NEW).read_bytes())
    return (lambda package: tmp_path / package if package == 'robot_description'
            else share(package)), physical


@pytest.mark.parametrize('conflict', ['canonical', 'old'])
def test_historical_alias_rejects_changed_physical_meaning(tmp_path, conflict):
    lookup, physical = isolated_share(tmp_path)
    changed = physical if conflict == 'canonical' else lookup('robot_description') / OLD
    changed.parent.mkdir(parents=True, exist_ok=True)
    changed.write_text('robot: {different: true}\n')
    with pytest.raises(ValueError, match='SHA|sha|checksum|fingerprint'):
        resolve_system_resource({'package': 'robot_description', 'path': OLD}, tmp_path, lookup)


@pytest.mark.parametrize('kind', ['package', 'absolute', 'relative'])
def test_external_manifest_migrates_old_description_and_keeps_real_source_fingerprints(
    tmp_path, kind,
):
    lookup, physical = isolated_share(tmp_path)
    base = lookup('robot_description')
    reference = ({'package': 'robot_description', 'path': OLD} if kind == 'package'
                 else str(base / OLD) if kind == 'absolute' else OLD)
    source = base / 'external_system.yaml'
    source.write_text(yaml.safe_dump({
        'schema_version': 2, 'robot': {'description': reference},
    }))
    original = source.read_bytes()
    resolved = resolve_system_resource(source, tmp_path, lookup)
    assert load_yaml(resolved)['robot']['description'] == str(physical)
    records = resource_sources(resolved)
    assert str(source) in {record['path'] for record in records}
    assert str(physical) in {record['path'] for record in records}
    assert source.read_bytes() == original
    physical.write_text('robot: {changed_after_preparation: true}\n')
    with pytest.raises(ValueError, match='source changed'):
        resource_sources(resolved)


def test_alias_does_not_guess_from_unrelated_filenames_or_missing_robots(tmp_path):
    outside = tmp_path / OLD
    outside.parent.mkdir(parents=True)
    outside.write_text('robot: {unrelated: true}\n')
    assert resolve_system_resource(outside, tmp_path, share) == outside
    with pytest.raises(ValueError):
        resolve_system_resource(
            {'package': 'robot_description', 'path': 'config/robots/unrelated.yaml'},
            tmp_path, share,
        )


def test_existing_unguarded_alias_supports_plain_string_resource(tmp_path):
    physical = tmp_path / 'selected.yaml'
    physical.write_text('value: 1\n')
    catalog = tmp_path / 'robot_experiments/config/experiments.yaml'
    catalog.parent.mkdir(parents=True)
    catalog.write_text(yaml.safe_dump({'legacy_resources': {
        'robot_simulation:config/old_fixture.yaml': str(physical),
    }}))

    def lookup(package):
        return tmp_path / package
    assert resolve_system_resource(
        {'package': 'robot_simulation', 'path': 'config/old_fixture.yaml'}, tmp_path, lookup,
    ) == physical


@pytest.mark.parametrize('guard', [None, True, 'invalid'])
def test_declared_checksum_guard_must_be_an_explicit_sha256(tmp_path, guard):
    lookup, physical = isolated_share(tmp_path)
    catalog = tmp_path / 'robot_experiments/config/experiments.yaml'
    catalog.parent.mkdir(parents=True)
    catalog.write_text(yaml.safe_dump({'legacy_resources': {
        'robot_description:' + OLD: {
            'package': 'robot_description', 'path': NEW, 'sha256': guard,
        },
    }}))

    def local_lookup(package):
        return tmp_path / package
    with pytest.raises(ValueError, match='SHA256 guard'):
        resolve_system_resource({'package': 'robot_description', 'path': OLD},
                                tmp_path, local_lookup)


def test_inherited_relative_legacy_description_keeps_declaring_parent_and_fingerprint(tmp_path):
    lookup, physical = isolated_share(tmp_path)
    parent = lookup('robot_description') / 'parent.yaml'
    parent.write_text(yaml.safe_dump({
        'schema_version': 2, 'robot': {'description': OLD},
    }))
    child = tmp_path / 'external/child.yaml'
    child.parent.mkdir()
    control = child.parent / 'control.yaml'
    control.write_text('control_node: {ros__parameters: {}}\n')
    child.write_text(
        'extends: ../robot_description/parent.yaml\ncontrol: {config: control.yaml}\n')
    resolved = resolve_system_resource(child, tmp_path, lookup)
    actual = load_yaml(resolved)
    assert actual['robot']['description'] == str(physical)
    assert actual['control']['config'] == str(control)
    sources = {record['path'] for record in resource_sources(resolved)}
    assert {str(parent), str(child), str(physical)} <= sources
    parent.write_text(parent.read_text() + '# changed parent\n')
    with pytest.raises(ValueError, match='source changed'):
        resource_sources(resolved)


@pytest.mark.parametrize('kind', ['relative', 'absolute'])
@pytest.mark.parametrize('symlink_root', [False, True])
def test_legacy_leaf_symlink_cannot_bypass_physical_checksum(tmp_path, kind, symlink_root):
    lookup, physical = isolated_share(tmp_path)
    original_lookup = lookup
    root = lookup('robot_description')
    if symlink_root:
        alias_root = tmp_path / 'installed_share'
        alias_root.symlink_to(root, target_is_directory=True)
        root = alias_root

        def lookup(name):
            return root if name == 'robot_description' else original_lookup(name)
    unrelated = tmp_path / 'different.yaml'
    unrelated.write_text('robot: {different: true}\n')
    old = root / OLD
    old.parent.mkdir(parents=True)
    old.symlink_to(unrelated)
    reference = OLD if kind == 'relative' else old
    with pytest.raises(ValueError, match='SHA256 mismatch'):
        resolve_system_resource(reference, root, lookup)


def test_proven_legacy_leaf_and_package_root_symlinks_remain_supported(tmp_path):
    lookup, physical = isolated_share(tmp_path)
    original_lookup = lookup
    root = tmp_path / 'installed_share'
    root.symlink_to(lookup('robot_description'), target_is_directory=True)

    def lookup(name):
        return root if name == 'robot_description' else original_lookup(name)
    old = root / OLD
    old.parent.mkdir(parents=True)
    old.symlink_to(physical)
    assert resolve_system_resource(old, root, lookup) == physical
    assert resolve_system_resource(OLD, root, lookup) == physical
