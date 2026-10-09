
"""Tests for component registration and configuration loading."""

from pathlib import Path

import pytest

from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.factory.registry import ComponentRegistry


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class ExampleComponent:
    """Simple configurable component for testing."""

    def __init__(self, gain: float) -> None:
        self.gain = gain


def test_component_registration():
    """Registered components should be constructible from config."""
    registry = ComponentRegistry()
    registry.register("example", ExampleComponent)

    component = registry.create({
        "type": "example",
        "parameters": {"gain": 2.5},
    })

    assert isinstance(component, ExampleComponent)
    assert component.gain == 2.5


def test_multiple_component_types():
    """Different types should produce different components."""
    registry = ComponentRegistry()
    registry.register("first", lambda: "first_model")
    registry.register("second", lambda: "second_model")

    assert registry.create({"type": "first"}) == "first_model"
    assert registry.create({"type": "second"}) == "second_model"


def test_unknown_component():
    """Unknown component types must raise an error."""
    registry = ComponentRegistry()

    with pytest.raises(ValueError, match="Unknown component"):
        registry.create({"type": "unknown"})


def test_duplicate_registration():
    """Duplicate type names must be rejected."""
    registry = ComponentRegistry()
    registry.register("example", ExampleComponent)

    with pytest.raises(ValueError, match="already registered"):
        registry.register("example", ExampleComponent)


@pytest.mark.parametrize(
    "filename,root",
    [
        ("target.yaml", "target"),
        ("environment.yaml", "environment"),
        ("simulation.yaml", "simulation"),
    ],
)
def test_yaml_loading(filename, root):
    """Each simulation configuration must expose its expected root."""
    config = load_yaml(CONFIG_DIR / filename)
    assert root in config
