
"""Utilities for loading external simulation configuration."""

from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a YAML configuration mapping from disk."""
    config_path = Path(path)

    with config_path.open(
        mode="r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError(
            f"Configuration must be a mapping: {config_path}"
        )

    return data
