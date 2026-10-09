
"""Tests for SDF model export."""

import xml.etree.ElementTree as ET
from pathlib import Path

from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.geometry.model_exporter import ModelExporter
from robot_simulation.geometry.sdf_generator import SDFGenerator


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def test_export_model(tmp_path):
    """Export a configured model to an SDF file."""
    config = load_yaml(CONFIG_DIR / "target.yaml")

    exporter = ModelExporter(
        generator=SDFGenerator.with_defaults()
    )

    output_path = tmp_path / "models" / "target.sdf"

    result = exporter.export(
        target_config=config,
        output_path=output_path,
    )

    assert result == output_path
    assert output_path.is_file()

    root = ET.parse(output_path).getroot()

    assert root.tag == "sdf"
    assert root.find(".//model[@name='target']") is not None

    radius = float(root.findtext(".//sphere/radius"))

    assert radius == config["target"]["geometry"]["parameters"]["radius"]
