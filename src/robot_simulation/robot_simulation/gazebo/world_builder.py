
"""Build a Gazebo SDF world from a chosen template and model configuration."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import math

from robot_configuration.yaml_loader import load_yaml
from robot_simulation.geometry.sdf_generator import SDFGenerator


def build_world(
    world_xacro,
    terrain_config,
    target_config,
    output_path,
    *,
    entity_name,
    system_plugins,
    models=(),
    physics=None,
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
    if physics is not None:
        if set(physics).difference({'max_step_size', 'real_time_factor'}):
            raise ValueError('Unsupported physics parameters')
        element = world.find('physics')
        if element is None:
            raise ValueError('Configured world must provide a physics element')
        for name, value in physics.items():
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f'Physics {name} must be finite and positive')
            parameter = element.find(name)
            if parameter is None:
                parameter = ET.SubElement(element, name)
            parameter.text = str(value)

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

    for extra in models:
        name = extra.get('name')
        if extra.tag != 'model' or not name:
            raise ValueError('Extra world models require a name')
        if any(existing.get('name') == name for existing in world.findall('model')):
            raise ValueError(f'Duplicate world model: {name}')
        world.append(extra)

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)

    ET.indent(root, space='  ')
    ET.ElementTree(root).write(
        destination,
        encoding='unicode',
        xml_declaration=True,
    )

    return destination, world.get('name')
