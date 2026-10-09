
"""Tests for configuration-driven simulation construction."""

from pathlib import Path

import numpy as np
import pytest

from robot_models.types import TargetState
from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.state.simulation_state import SimulationState


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def load_configs():
    """Load the three simulation configuration files."""
    return (
        load_yaml(CONFIG_DIR / "target.yaml"),
        load_yaml(CONFIG_DIR / "environment.yaml"),
        load_yaml(CONFIG_DIR / "simulation.yaml"),
    )


def make_initial_state():
    """Create a test state independent of the factory."""
    return SimulationState(
        target=TargetState(
            position=np.array([2.0, 0.0, 1.0]),
            velocity=np.zeros(3),
        ),
        tracking_position_world=np.array([0.0, 0.0, 1.0]),
        time=0.0,
    )


def test_factory_creates_engine():
    """Build and run the configured simulation."""
    engine, dt = SimulationFactory.with_defaults().create(
        *load_configs()
    )

    state = make_initial_state()
    next_state = engine.step(state, dt)

    assert next_state.time == pytest.approx(dt)
    assert np.all(np.isfinite(next_state.target.position))
    assert np.all(np.isfinite(next_state.target.velocity))


def test_factory_without_environment_fields():
    """The engine must run with no environmental fields."""
    target, environment, simulation = load_configs()
    environment["environment"]["acceleration_fields"] = []

    engine, dt = SimulationFactory.with_defaults().create(
        target, environment, simulation
    )

    result = engine.step(make_initial_state(), dt)
    assert np.all(np.isfinite(result.target.position))


def test_factory_rejects_unknown_dynamics():
    """Unsupported dynamics must be rejected by the registry."""
    target, environment, simulation = load_configs()
    target["target"]["dynamics"]["type"] = "unknown_model"

    with pytest.raises(ValueError, match="Unknown component"):
        SimulationFactory.with_defaults().create(
            target, environment, simulation
        )


def test_factory_rejects_invalid_dt():
    """An invalid timestep must be rejected."""
    target, environment, simulation = load_configs()
    simulation["simulation"]["integration"]["parameters"]["dt"] = -0.1

    with pytest.raises(ValueError, match="dt must be positive"):
        SimulationFactory.with_defaults().create(
            target, environment, simulation
        )


def test_composite_field_sums_contributions():
    """Two environmental fields must contribute additively."""
    from robot_simulation.disturbances.composite_field import (
        CompositeAccelerationField,
    )
    from robot_simulation.disturbances.vector_field import (
        RestoringVectorField,
        VectorFieldParameters,
    )

    fields = [
        RestoringVectorField(
            VectorFieldParameters(
                center=np.zeros(3),
                gain_matrix=gain * np.eye(3),
            )
        )
        for gain in (1.0, 2.0)
    ]

    composite = CompositeAccelerationField(fields)

    np.testing.assert_allclose(
        composite.acceleration(np.array([1.0, 0.0, 0.0])),
        [-3.0, 0.0, 0.0],
    )
