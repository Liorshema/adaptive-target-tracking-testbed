
"""Build a Gazebo SDF world from a chosen template and model configuration."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.geometry.sdf_generator import SDFGenerator


def build_world(
    world_xacro,
    terrain_config,
    target_config,
    output_path,
    *,
    entity_name,
    system_plugins,
):
    """Render the configured world and add a target and selected system plugins.

    Returns the output path and the world name read from SDF (not hard-coded).
    """
    completed = subprocess.run(
        ['xacro', str(world_xacro), f'terrain_config:={terrain_config}'],
        check=True,
        capture_output=True,
        text=True,
    )

    root = ET.fromstring(completed.stdout)
    world = root.find('world')

    if world is None or not world.get('name'):
        raise ValueError('World SDF requires a named <world> element.')

    for plugin in system_plugins:
        filename, name = plugin['filename'], plugin['name']
        if not any(p.get('name') == name for p in world.findall('plugin')):
            ET.SubElement(
                world,
                'plugin',
                filename=filename,
                name=name,
            )

    target_xml = SDFGenerator.with_defaults().generate(
        load_yaml(target_config),
        model_name=entity_name,
    )

    model = ET.fromstring(target_xml).find('model')

    if model is None:
        raise ValueError('Generated target SDF has no model.')

    if world.find(f"model[@name='{entity_name}']") is not None:
        raise ValueError(f'Duplicate target model: {entity_name}')

    world.append(model)

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)

    ET.indent(root, space='  ')
    ET.ElementTree(root).write(
        destination,
        encoding='unicode',
        xml_declaration=True,
    )

    return destination, world.get('name')
