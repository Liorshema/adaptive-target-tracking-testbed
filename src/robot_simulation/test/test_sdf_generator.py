
"""Tests for configuration-driven Gazebo SDF generation."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.geometry.sdf_generator import SDFGenerator


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "target.yaml"
)


def make_config():
    """Read the configured target model."""
    return load_yaml(CONFIG_PATH)


def test_sphere_from_yaml():
    """Sphere radius must match the YAML configuration."""
    config = make_config()

    sdf = SDFGenerator.with_defaults().generate(config)
    root = ET.fromstring(sdf)

    radius = float(
        root.findtext(".//sphere/radius")
    )

    assert radius == pytest.approx(
        config["target"]["geometry"]["parameters"]["radius"]
    )


def test_box_from_yaml():
    """Changing geometry configuration must produce a box."""
    config = deepcopy(make_config())

    config["target"]["geometry"] = {
        "type": "box",
        "parameters": {
            "size": [0.2, 0.3, 0.4],
        },
    }

    sdf = SDFGenerator.with_defaults().generate(config)
    root = ET.fromstring(sdf)

    size = [
        float(value)
        for value in root.findtext(".//box/size").split()
    ]

    assert size == pytest.approx([0.2, 0.3, 0.4])
    assert root.find(".//sphere") is None


def test_color_from_yaml():
    """The visual material must use the configured RGBA values."""
    config = make_config()

    sdf = SDFGenerator.with_defaults().generate(config)
    root = ET.fromstring(sdf)

    color = [
        float(value)
        for value in root.findtext(".//material/diffuse").split()
    ]

    assert color == pytest.approx(
        config["target"]["appearance"]["color"]
    )


def test_collision_disabled():
    """Disabled collision must not create a collision element."""
    root = ET.fromstring(
        SDFGenerator.with_defaults().generate(make_config())
    )

    assert root.find(".//collision") is None


def test_collision_enabled():
    """Collision geometry must follow the configured shape."""
    config = make_config()
    config["target"]["physics"]["collision_enabled"] = True

    root = ET.fromstring(
        SDFGenerator.with_defaults().generate(config)
    )

    assert root.find(".//collision/geometry/sphere") is not None


def test_invalid_radius():
    """Non-positive geometry dimensions must be rejected."""
    config = make_config()
    config["target"]["geometry"]["parameters"]["radius"] = -1.0

    with pytest.raises(ValueError):
        SDFGenerator.with_defaults().generate(config)


def test_unknown_geometry():
    """Unsupported geometry must raise a clear error."""
    config = make_config()
    config["target"]["geometry"]["type"] = "unknown_shape"

    with pytest.raises(ValueError, match="Unknown geometry"):
        SDFGenerator.with_defaults().generate(config)
