
"""Generate Gazebo SDF models from external geometry configuration."""

from collections.abc import Callable, Mapping
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np


def _positive_values(values: Any, count: int, name: str) -> np.ndarray:
    """Validate positive, finite geometry dimensions."""
    result = np.asarray(values, dtype=float)

    if result.shape != (count,):
        raise ValueError(f"{name} must have shape ({count},).")

    if not np.all(np.isfinite(result)) or np.any(result <= 0.0):
        raise ValueError(f"{name} must contain positive finite values.")

    return result


def _sphere(parameters: Mapping[str, Any]) -> ET.Element:
    """Create sphere geometry."""
    radius = _positive_values(
        [parameters["radius"]], 1, "radius"
    )[0]

    geometry = ET.Element("geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = str(float(radius))

    return geometry


def _box(parameters: Mapping[str, Any]) -> ET.Element:
    """Create box geometry."""
    size = _positive_values(parameters["size"], 3, "size")

    geometry = ET.Element("geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = " ".join(map(str, size))

    return geometry


class SDFGenerator:
    """Generate SDF models using registered geometry builders."""

    def __init__(self) -> None:
        self._geometry_builders: dict[
            str, Callable[[Mapping[str, Any]], ET.Element]
        ] = {}

    def register_geometry(
        self,
        name: str,
        builder: Callable[[Mapping[str, Any]], ET.Element],
    ) -> None:
        """Register a geometry implementation."""
        if not name or not callable(builder):
            raise ValueError("Invalid geometry registration.")

        if name in self._geometry_builders:
            raise ValueError(f"Geometry already registered: {name}")

        self._geometry_builders[name] = builder

    @classmethod
    def with_defaults(cls) -> "SDFGenerator":
        """Register the built-in geometry implementations."""
        generator = cls()
        generator.register_geometry("sphere", _sphere)
        generator.register_geometry("box", _box)
        return generator

    def generate(
        self,
        target_config: Mapping[str, Any],
        model_name: str = "target",
    ) -> str:
        """Generate a complete SDF model from target configuration."""
        if not model_name or not model_name.strip():
            raise ValueError("Model name must be non-empty.")

        target = target_config["target"]
        geometry_config = target["geometry"]
        geometry_type = geometry_config["type"]

        if geometry_type not in self._geometry_builders:
            raise ValueError(
                f"Unknown geometry type: {geometry_type}"
            )

        parameters = geometry_config.get("parameters", {})
        geometry = self._geometry_builders[geometry_type](parameters)

        color = np.asarray(
            target["appearance"]["color"], dtype=float
        )

        if (
            color.shape != (4,)
            or not np.all(np.isfinite(color))
            or np.any(color < 0.0)
            or np.any(color > 1.0)
        ):
            raise ValueError("Color must be an RGBA vector in [0, 1].")

        physics = target["physics"]

        for key in ("kinematic", "gravity", "collision_enabled"):
            if not isinstance(physics[key], bool):
                raise ValueError(f"{key} must be a boolean.")

        sdf = ET.Element("sdf", {"version": "1.9"})
        model = ET.SubElement(sdf, "model", {"name": model_name})

        link = ET.SubElement(model, "link", {"name": "target_link"})

        ET.SubElement(link, "kinematic").text = str(
            physics["kinematic"]
        ).lower()
        ET.SubElement(link, "gravity").text = str(
            physics["gravity"]
        ).lower()

        visual = ET.SubElement(
            link, "visual", {"name": "target_visual"}
        )
        visual.append(geometry)

        material = ET.SubElement(visual, "material")
        color_text = " ".join(map(str, color))
        ET.SubElement(material, "ambient").text = color_text
        ET.SubElement(material, "diffuse").text = color_text

        if physics["collision_enabled"]:
            collision = ET.SubElement(
                link, "collision", {"name": "target_collision"}
            )
            collision.append(
                self._geometry_builders[geometry_type](parameters)
            )

        ET.indent(sdf, space="  ")

        return ET.tostring(
            sdf, encoding="unicode", xml_declaration=True
        )
