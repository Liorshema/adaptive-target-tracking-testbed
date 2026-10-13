
"""Check generic world construction without Gazebo or ROS dependencies."""

from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest

from robot_simulation.gazebo.world_builder import build_world


def test_world_builder_reads_name_and_plugins_from_inputs(tmp_path):
    base = Path(__file__).resolve().parents[1]

    xml = '<sdf version="1.9"><world name="user_defined_world"/></sdf>'
    plugins = [
        {
            'filename': 'example-plugin',
            'name': 'ExampleSystem',
        }
    ]

    with patch(
        'robot_simulation.gazebo.world_builder.subprocess.run',
        return_value=CompletedProcess(
            ['xacro'],
            0,
            stdout=xml,
        ),
    ):
        output, name = build_world(
            base / 'worlds/tracking_testbed.sdf.xacro',
            base / 'config/terrain/flat.yaml',
            base / 'config/target.yaml',
            tmp_path / 'world.sdf',
            entity_name='my_object',
            system_plugins=plugins,
        )

    world = ET.parse(output).getroot().find('world')

    assert name == 'user_defined_world'
    assert world.find("model[@name='my_object']") is not None
    assert len(world.findall('plugin')) == 1
    assert world.find('plugin').get('name') == 'ExampleSystem'


def test_world_includes_robot_in_initial_scene(tmp_path):
    """The normal scene owns the robot so a world reset can recreate it."""
    base = Path(__file__).resolve().parents[1]
    robot = ET.fromstring('<model name="robot"><link name="base"/></model>')
    with patch('robot_simulation.gazebo.world_builder.subprocess.run',
               return_value=CompletedProcess(['xacro'], 0,
                                            stdout='<sdf version="1.9"><world name="scene"/></sdf>')):
        output, name = build_world(
            base / 'worlds/tracking_testbed.sdf.xacro',
            base / 'config/terrain/flat.yaml', base / 'config/target.yaml',
            tmp_path / 'world.sdf', entity_name='target', system_plugins=[],
            models=[robot],
        )
    world = ET.parse(output).getroot().find('world')
    assert world.find("model[@name='robot']") is not None
    assert world.find("model[@name='target']") is not None


@pytest.mark.parametrize('model_xml', ['<model name="target"/>', '<model/>'])
def test_world_rejects_duplicate_or_unnamed_robot(tmp_path, model_xml):
    """Reject invalid scene identity before invoking the simulator."""
    base = Path(__file__).resolve().parents[1]
    with patch('robot_simulation.gazebo.world_builder.subprocess.run',
               return_value=CompletedProcess(['xacro'], 0,
                                            stdout='<sdf version="1.9"><world name="scene"/></sdf>')):
        with pytest.raises(ValueError):
            build_world(
                base / 'worlds/tracking_testbed.sdf.xacro',
                base / 'config/terrain/flat.yaml', base / 'config/target.yaml',
                tmp_path / 'world.sdf', entity_name='target', system_plugins=[],
                models=[ET.fromstring(model_xml)],
            )
