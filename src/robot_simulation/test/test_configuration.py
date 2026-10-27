
"""Tests for component registration and configuration loading."""

from pathlib import Path

import pytest

from functools import partial
from robot_experiments.composition import load_resource
load_yaml = partial(load_resource, share=lambda package: Path(__file__).resolve().parents[2] / package)
from robot_simulation.factory.registry import ComponentRegistry


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'


def effective_target(reference):
    """Resolve selected parameters through the same canonical runtime registry."""
    from robot_simulation.factory.simulation_factory import SimulationFactory
    target = load_yaml(reference)
    dynamics = target['target']['dynamics']
    dynamics['parameters'] = SimulationFactory.with_defaults().dynamics_registry.validate(
        dynamics['type'], dynamics.get('parameters', {}))
    return target


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
    config = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_moving_target'})

    assert config['paths']['base'] == 'package'
    assert config['world']['output_name'] == 'world.sdf'
    assert config['target']['initial_position'] == [2.7, 0.0, 0.85]
    assert config['target']['initial_velocity'] == [0.0, 0.35, 0.0]
    assert config['tracking']['source'] == 'static'
    target = effective_target(CONFIG_DIR.parent / config['target']['config'])
    assert target['target']['dynamics']['parameters'] == {
        'alpha': 0.0,
        'preferred_distance': 1.0,
        'damping': 0.05,
        'distance_epsilon': 1.0e-6,
    }


def test_reactive_target_scenario_uses_real_tracking_frame():
    """The optional robot-reactive scenario must enable TF feedback."""
    config = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_reactive_target'})

    assert config['tracking']['source'] == 'tf'
    assert config['tracking']['reference_frame'] == 'world'
    assert config['tracking']['tracking_frame'] == 'tracking_frame'
    assert config['target']['initial_position'] == [2.0, 0.0, 1.0]
    assert config['target']['initial_velocity'] == [0.0, 0.0, 0.0]
    target = effective_target(CONFIG_DIR.parent / config['target']['config'])
    assert target['target']['dynamics']['parameters']['alpha'] == 1.0


def test_low_damping_experiment_preserves_reactive_tf_and_environment():
    """The experiment changes target damping while retaining baseline resources."""
    baseline = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_reactive_target'})
    experiment = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_reactive_target_low_damping'})
    assert experiment['tracking'] == baseline['tracking']
    assert experiment['tracking']['source'] == 'tf'
    assert experiment['simulation'] == baseline['simulation']
    assert experiment['target']['initial_position'] == baseline['target']['initial_position']
    assert experiment['target']['initial_velocity'] == baseline['target']['initial_velocity']
    base_target = effective_target(CONFIG_DIR.parent / baseline['target']['config'])
    target = effective_target(CONFIG_DIR.parent / experiment['target']['config'])
    assert target['target']['dynamics']['parameters']['damping'] == 0.15
    target['target']['dynamics']['parameters']['damping'] = 0.5
    assert target == base_target


def test_saturated_scenario_selects_shared_engine_without_inherited_alpha():
    """A new interaction must retain visuals, external fields and live TF feedback."""
    from robot_simulation.factory.simulation_factory import SimulationFactory
    baseline = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_reactive_target'})
    scenario = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_reactive_target_saturated'})
    target = load_yaml(CONFIG_DIR.parent / scenario['target']['config'])
    original = load_yaml(CONFIG_DIR / 'target.yaml')
    assert scenario['tracking'] == baseline['tracking']
    assert scenario['simulation'] == baseline['simulation']
    assert target['target']['geometry'] == original['target']['geometry']
    assert target['target']['physics'] == original['target']['physics']
    assert 'alpha' not in target['target']['dynamics']['parameters']
    engine, _ = SimulationFactory.with_defaults().create(
        target, load_yaml(CONFIG_DIR.parent / scenario['simulation']['environment_config']),
        load_yaml(CONFIG_DIR.parent / scenario['simulation']['config']))
    assert engine.dynamics.parameters.a_max > 0.


def test_launch_without_physics_override_uses_effective_world_step(tmp_path, monkeypatch):
    """An optional override must not become a new required launch YAML key."""
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    import yaml
    root=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('simulation_without_physics_override',root/'robot_simulation/launch/simulation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:str(root/name))
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros'))
    cfg=load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_standalone'});del cfg['world']['physics']
    path=tmp_path/'launch.yaml';path.write_text(yaml.safe_dump(cfg))
    context=LaunchContext();context.launch_configurations.update(
        config=str(path),gui='false',enable_actuation='false',publish_ground_truth='')
    nodes=module.configure(context)
    simulation=next(n for n in nodes if isinstance(n,Node) and n.node_executable=='simulation_node')
    params=evaluate_parameters(context,simulation._Node__parameters)[0]
    assert params['context_history_capacity'] == 10402
