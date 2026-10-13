
"""Generic registry for constructing configurable control components."""

from collections.abc import Callable, Mapping
from typing import Any


class ComponentRegistry:
    """Map component names to externally registered constructors.

    Registration is explicit. No dynamic imports or assumptions about
    concrete controllers, robots, or reference models are required.
    """

    def __init__(self) -> None:
        """Initialize an empty component registry."""
        self._constructors: dict[str, Callable[..., Any]] = {}

    def register(
        self,
        name: str,
        constructor: Callable[..., Any],
    ) -> None:
        """Register a component constructor under a unique name."""
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Component name must be a nonempty string")

        if not callable(constructor):
            raise TypeError("Component constructor must be callable")

        if name in self._constructors:
            raise ValueError(
                f"Component '{name}' is already registered"
            )

        self._constructors[name] = constructor

    def create(
        self,
        name: str,
        parameters: Mapping[str, Any] | None = None,
    ) -> Any:
        """Create a registered component using named parameters.

        Args:
            name: Registered component identifier.
            parameters: Keyword arguments passed to its constructor.

        Returns:
            Constructed component.

        Raises:
            KeyError: If the requested component is not registered.
            TypeError: If parameters are not a mapping.
        """
        if name not in self._constructors:
            available = ", ".join(sorted(self._constructors))
            raise KeyError(
                f"Unknown component '{name}'. "
                f"Available components: {available}"
            )

        if parameters is None:
            parameters = {}

        if not isinstance(parameters, Mapping):
            raise TypeError(
                "Component parameters must be a mapping"
            )

        return self._constructors[name](**dict(parameters))

    def names(self) -> tuple[str, ...]:
        """Return registered component names in sorted order."""
        return tuple(sorted(self._constructors))
