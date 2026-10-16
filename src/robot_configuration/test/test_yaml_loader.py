"""Portable tests for the single mapping/inheritance owner."""

import pytest

from robot_configuration.yaml_loader import load_yaml


def test_yaml_inheritance_merges_mappings_and_replaces_lists(tmp_path):
    """Child overrides must preserve siblings and replace list values."""
    base = tmp_path / 'base.yaml'
    base.write_text(
        'world: {output: world.sdf, physics: {step: 0.001, rate: 1000}}\n'
        'plugins: [physics, scene]\n'
        'tracking: {source: static}\n',
        encoding='utf-8',
    )
    scenario_dir = tmp_path / 'scenarios'
    scenario_dir.mkdir()
    child = scenario_dir / 'moving.yaml'
    child.write_text(
        'extends: ../base.yaml\n'
        'world: {physics: {step: 0.002}}\n'
        'plugins: [physics]\n'
        'tracking: null\n',
        encoding='utf-8',
    )

    assert load_yaml(child) == {
        'world': {
            'output': 'world.sdf',
            'physics': {'step': 0.002, 'rate': 1000},
        },
        'plugins': ['physics'],
        'tracking': None,
    }
    assert load_yaml(base)['plugins'] == ['physics', 'scene']


def test_yaml_inheritance_resolves_each_parent_relative_to_its_child(tmp_path):
    """A nested parent reference must use its own directory."""
    (tmp_path / 'base.yaml').write_text('gain: 2\n', encoding='utf-8')
    parent_dir = tmp_path / 'parents'
    parent_dir.mkdir()
    (parent_dir / 'parent.yaml').write_text(
        'extends: ../base.yaml\nrate: 20\n', encoding='utf-8'
    )
    child = tmp_path / 'child.yaml'
    child.write_text(
        'extends: parents/parent.yaml\ngain: 3\n', encoding='utf-8'
    )

    assert load_yaml(child) == {'gain': 3, 'rate': 20}


@pytest.mark.parametrize('extension', ['null', '[]', '{}', '3', "''"])
def test_yaml_inheritance_rejects_invalid_parent_values(tmp_path, extension):
    """Inheritance must explicitly name one parent file."""
    child = tmp_path / 'child.yaml'
    child.write_text(f'extends: {extension}\n', encoding='utf-8')

    with pytest.raises(ValueError, match='extends'):
        load_yaml(child)


def test_yaml_inheritance_rejects_cycles(tmp_path):
    """Canonical paths must detect a cycle before recursive loading."""
    first = tmp_path / 'first.yaml'
    first.write_text('extends: second.yaml\n', encoding='utf-8')
    (tmp_path / 'second.yaml').write_text(
        'extends: ./first.yaml\n', encoding='utf-8'
    )

    with pytest.raises(ValueError, match='cycle'):
        load_yaml(first)


@pytest.mark.parametrize('contents', ['[]\n', 'null\n', '42\n'])
def test_yaml_inheritance_rejects_non_mapping_parent(tmp_path, contents):
    """A malformed parent must never silently become empty defaults."""
    (tmp_path / 'base.yaml').write_text(contents, encoding='utf-8')
    child = tmp_path / 'child.yaml'
    child.write_text('extends: base.yaml\n', encoding='utf-8')

    with pytest.raises(ValueError, match='mapping'):
        load_yaml(child)


def test_yaml_loading_rejects_recursive_aliases(tmp_path):
    """Configuration trees must remain finite for recursive merging."""
    child = tmp_path / 'recursive.yaml'
    child.write_text('robot: &robot {nested: *robot}\n', encoding='utf-8')

    with pytest.raises(ValueError, match='recursive'):
        load_yaml(child)
