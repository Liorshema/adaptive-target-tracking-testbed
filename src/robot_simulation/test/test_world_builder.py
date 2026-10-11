
"""Check generic world construction without Gazebo or ROS dependencies."""

from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch
import xml.etree.ElementTree as ET

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
