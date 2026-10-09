
"""Export generated SDF models to disk."""

from pathlib import Path
from typing import Any, Mapping

from robot_simulation.geometry.sdf_generator import SDFGenerator


class ModelExporter:
    """Write a configured target model to an SDF file."""

    def __init__(self, generator: SDFGenerator) -> None:
        self.generator = generator

    def export(
        self,
        target_config: Mapping[str, Any],
        output_path: str | Path,
        model_name: str = "target",
    ) -> Path:
        """Generate and save an SDF model."""
        destination = Path(output_path)
        sdf_content = self.generator.generate(
            target_config=target_config,
            model_name=model_name,
        )

        destination.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        destination.write_text(
            sdf_content,
            encoding="utf-8",
        )

        return destination
