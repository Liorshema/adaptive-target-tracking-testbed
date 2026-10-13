
"""Unit tests for the generic component registry."""

import pytest

from robot_control.factory.component_registry import ComponentRegistry


class ExampleController:
    """Simple controller stub for registry tests."""

    def __init__(self, gain=1.0, mode="velocity"):
        """Store injected configuration."""
        self.gain = gain
        self.mode = mode


class AlternativeController:
    """Alternative implementation sharing the same registry."""

    def __init__(self, horizon=10):
        """Store the configured horizon."""
        self.horizon = horizon


@pytest.fixture
def registry():
    """Provide an empty registry for each test."""
    return ComponentRegistry()


def test_registry_starts_empty(registry):
    """A new registry must have no registered components."""
    assert registry.names() == ()


def test_register_component(registry):
    """Register a constructor under a configured identifier."""
    registry.register("jacobian", ExampleController)

    assert registry.names() == ("jacobian",)


def test_create_component_without_parameters(registry):
    """Create an instance using constructor defaults."""
    registry.register("jacobian", ExampleController)

    component = registry.create("jacobian")

    assert isinstance(component, ExampleController)
    assert component.gain == 1.0
    assert component.mode == "velocity"


def test_create_component_with_parameters(registry):
    """Forward named parameters to the constructor."""
    registry.register("jacobian", ExampleController)

    component = registry.create(
        "jacobian",
        parameters={
            "gain": 2.5,
            "mode": "effort",
        },
    )

    assert component.gain == pytest.approx(2.5)
    assert component.mode == "effort"


def test_create_distinct_instances(registry):
    """Each construction call must create a separate instance."""
    registry.register("jacobian", ExampleController)

    first = registry.create("jacobian")
    second = registry.create("jacobian")

    assert first is not second


def test_register_multiple_implementations(registry):
    """Allow independently selectable component implementations."""
    registry.register("jacobian", ExampleController)
    registry.register("alternative", AlternativeController)

    first = registry.create("jacobian")
    second = registry.create(
        "alternative",
        {"horizon": 20},
    )

    assert isinstance(first, ExampleController)
    assert isinstance(second, AlternativeController)
    assert second.horizon == 20


def test_names_are_sorted(registry):
    """Expose deterministic registry ordering."""
    registry.register("zeta", ExampleController)
    registry.register("alpha", AlternativeController)

    assert registry.names() == ("alpha", "zeta")


def test_reject_duplicate_registration(registry):
    """Prevent silent replacement of registered implementations."""
    registry.register("jacobian", ExampleController)

    with pytest.raises(ValueError, match="already registered"):
        registry.register("jacobian", AlternativeController)


@pytest.mark.parametrize("name", ["", " ", "\t", None, 123])
def test_reject_invalid_registration_name(registry, name):
    """Reject empty and non-string component identifiers."""
    with pytest.raises(ValueError, match="nonempty string"):
        registry.register(name, ExampleController)


def test_reject_noncallable_constructor(registry):
    """Require a valid constructor callable."""
    with pytest.raises(TypeError, match="callable"):
        registry.register("invalid", object())


def test_unknown_component(registry):
    """Fail clearly when a component has not been registered."""
    registry.register("jacobian", ExampleController)

    with pytest.raises(KeyError, match="Unknown component"):
        registry.create("mpc")


def test_unknown_component_lists_available_names(registry):
    """Provide useful diagnostics for configuration mistakes."""
    registry.register("jacobian", ExampleController)

    with pytest.raises(KeyError, match="jacobian"):
        registry.create("missing")


@pytest.mark.parametrize("parameters", [42, "gain=2", [1, 2]])
def test_reject_invalid_parameters(registry, parameters):
    """Accept mappings rather than positional or scalar parameters."""
    registry.register("jacobian", ExampleController)

    with pytest.raises(TypeError, match="mapping"):
        registry.create("jacobian", parameters)


def test_constructor_errors_are_not_suppressed(registry):
    """Expose errors from incorrectly configured constructors."""
    registry.register("jacobian", ExampleController)

    with pytest.raises(TypeError):
        registry.create(
            "jacobian",
            {"unsupported_parameter": True},
        )


def test_callable_factory(registry):
    """Support factory functions in addition to classes."""
    registry.register(
        "scaled",
        lambda gain: {"effective_gain": gain * 2.0},
    )

    result = registry.create("scaled", {"gain": 3.0})

    assert result == {"effective_gain": 6.0}


def test_independent_registries():
    """Keep registrations isolated across independent registries."""
    first = ComponentRegistry()
    second = ComponentRegistry()

    first.register("jacobian", ExampleController)

    assert first.names() == ("jacobian",)
    assert second.names() == ()
