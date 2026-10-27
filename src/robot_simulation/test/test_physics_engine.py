"""Explicit backend selection must survive rendered and copied deployments."""
import xml.etree.ElementTree as ET
from pathlib import Path
from subprocess import CompletedProcess
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from functools import partial
from robot_experiments.composition import load_resource
load_yaml = partial(load_resource, share=lambda package: Path(__file__).resolve().parents[2] / package)

from robot_simulation.factory.deployment_configuration import validate_deployment_configuration
from robot_simulation.gazebo.world_builder import build_world


def test_selected_motor_backend_is_written_inside_physics_system(tmp_path):
    """Write the discovered library into Gazebo's supported Physics-system field."""
    base = Path(__file__).parents[1]
    library = tmp_path / 'installed_motor_backend.so'
    library.touch()
    rendered = CompletedProcess(
        ['xacro'], 0, stdout='<sdf><world name="test">'
        '<physics name="physics" type="ode"/></world></sdf>')
    with (patch('robot_simulation.gazebo.world_builder.subprocess.run', return_value=rendered),
          patch('importlib.util.find_spec', return_value=SimpleNamespace(origin=str(library)))):
        output, _ = build_world(
            base / 'worlds/tracking_testbed.sdf.xacro', base / 'config/terrain/flat.yaml',
            base / 'config/target.yaml', tmp_path / 'world.sdf', entity_name='target',
            system_plugins=[{'filename': 'gz-sim-physics-system',
                             'name': 'gz::sim::systems::Physics'}],
            physics={'engine': 'dartsim_recovery', 'max_step_size': .002})
    world = ET.parse(output).getroot().find('world')
    plugin = world.find("plugin[@name='gz::sim::systems::Physics']")
    assert plugin.findtext('engine/filename') == str(library)
    assert world.findtext('physics/max_step_size') == '0.002'
    assert world.find('physics').get('type') == 'ode'


@pytest.mark.parametrize('engine', ['dartsim', 'dartsim_recovery'])
def test_preflight_accepts_only_supported_physics_engines(engine):
    """Allow explicit stock and corrected engines without requiring a build."""
    root = Path(__file__).parents[1]
    d = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_oscillatory_reactive_tracking'})
    d['world']['physics']['engine'] = engine
    validate_deployment_configuration(
        d, load_yaml(root / d['target']['config']),
        load_yaml(root / d['simulation']['environment_config']),
        load_yaml(root / d['simulation']['config']),
        load_yaml(root / d['simulation']['measurement_config']))


@pytest.mark.parametrize('engine', ['typo', '/arbitrary.so', True, None])
def test_preflight_rejects_invalid_physics_engine(engine):
    """Do not silently fall back from an unsupported or malformed selector."""
    root = Path(__file__).parents[1]
    d = load_yaml({'package': 'robot_experiments', 'path': 'config/experiments.yaml', 'group': 'resources.deployment_oscillatory_reactive_tracking'})
    d['world']['physics']['engine'] = engine
    with pytest.raises(ValueError):
        validate_deployment_configuration(
            d, load_yaml(root / d['target']['config']),
            load_yaml(root / d['simulation']['environment_config']),
            load_yaml(root / d['simulation']['config']),
            load_yaml(root / d['simulation']['measurement_config']))
