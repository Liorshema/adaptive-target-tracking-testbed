
"""Generic registry for configurable simulation components."""

from collections.abc import Callable, Mapping
from typing import Any


class ComponentRegistry:
    """Register and construct components by their configured type."""

    def __init__(self) -> None:
        self._factories: dict[str, Callable[..., Any]] = {}

    def register(
        self,
        name: str,
        factory: Callable[..., Any],
    ) -> None:
        """Register a component constructor."""
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Component name must be non-empty.")

        if not callable(factory):
            raise TypeError("Component factory must be callable.")

        if name in self._factories:
            raise ValueError(f"Component already registered: {name}")

        self._factories[name] = factory

    def create(
        self,
        config: Mapping[str, Any],
        **dependencies: Any,
    ) -> Any:
        """Construct a component from its type and parameters."""
        component_type = config["type"]

        if component_type not in self._factories:
            raise ValueError(
                f"Unknown component type: {component_type}"
            )

        parameters = config.get("parameters", {})

        if not isinstance(parameters, Mapping):
            raise ValueError("Component parameters must be a mapping.")

        return self._factories[component_type](
            **dict(parameters),
            **dependencies,
        )

    @property
    def registered_types(self) -> tuple[str, ...]:
        """Return the names of registered component types."""
        return tuple(self._factories)
