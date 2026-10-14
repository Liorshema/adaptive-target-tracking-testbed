
"""Tests for component registration and configuration loading."""

from pathlib import Path

import pytest

from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.factory.registry import ComponentRegistry


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'


class ExampleComponent:
    """Simple configurable component for testing."""

    def __init__(self, gain: float) -> None:
        self.gain = gain


def test_component_registration():
    """Registered components should be constructible from config."""
    registry = ComponentRegistry()
    registry.register('example', ExampleComponent)

    component = registry.create({
        'type': 'example',
        'parameters': {'gain': 2.5},
    })

    assert isinstance(component, ExampleComponent)
    assert component.gain == 2.5


def test_multiple_component_types():
    """Different types should produce different components."""
    registry = ComponentRegistry()
    registry.register('first', lambda: 'first_model')
    registry.register('second', lambda: 'second_model')

    assert registry.create({'type': 'first'}) == 'first_model'
    assert registry.create({'type': 'second'}) == 'second_model'


def test_unknown_component():
    """Unknown component types must raise an error."""
    registry = ComponentRegistry()

    with pytest.raises(ValueError, match='Unknown component'):
        registry.create({'type': 'unknown'})


def test_duplicate_registration():
    """Duplicate type names must be rejected."""
    registry = ComponentRegistry()
    registry.register('example', ExampleComponent)

    with pytest.raises(ValueError, match='already registered'):
        registry.register('example', ExampleComponent)


@pytest.mark.parametrize(
    'filename,root',
    [
        ('target.yaml', 'target'),
        ('environment.yaml', 'environment'),
        ('simulation.yaml', 'simulation'),
    ],
)
def test_yaml_loading(filename, root):
    """Each simulation configuration must expose its expected root."""
    config = load_yaml(CONFIG_DIR / filename)
    assert root in config


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


def test_moving_target_scenario_preserves_independent_dynamics():
    """Composition must preserve the independent trajectory conditions."""
    config = load_yaml(CONFIG_DIR / 'scenarios' / 'moving_target.yaml')

    assert config['paths']['base'] == 'package'
    assert config['world']['output_name'] == 'world.sdf'
    assert config['target']['initial_position'] == [2.7, 0.0, 0.85]
    assert config['target']['initial_velocity'] == [0.0, 0.35, 0.0]
    assert config['tracking']['source'] == 'static'
    target = load_yaml(CONFIG_DIR.parent / config['target']['config'])
    assert target['target']['dynamics']['parameters'] == {
        'alpha': 0.0,
        'preferred_distance': 1.0,
        'damping': 0.05,
        'distance_epsilon': 1.0e-6,
    }


def test_reactive_target_scenario_uses_real_tracking_frame():
    """The optional robot-reactive scenario must enable TF feedback."""
    config = load_yaml(CONFIG_DIR / 'scenarios' / 'reactive_target.yaml')

    assert config['tracking']['source'] == 'tf'
    assert config['tracking']['reference_frame'] == 'world'
    assert config['tracking']['tracking_frame'] == 'tracking_frame'
    assert config['target']['initial_position'] == [2.0, 0.0, 1.0]
    assert config['target']['initial_velocity'] == [0.0, 0.0, 0.0]
    target = load_yaml(CONFIG_DIR.parent / config['target']['config'])
    assert target['target']['dynamics']['parameters']['alpha'] == 1.0
