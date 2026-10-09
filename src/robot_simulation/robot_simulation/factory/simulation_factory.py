
"""Configuration-driven construction of simulation components."""

from collections.abc import Mapping
from typing import Any

import numpy as np

from robot_simulation.dynamics.target.fight_flight import (
    FightFlightDynamics,
    FightFlightParameters,
)
from robot_simulation.disturbances.composite_field import (
    CompositeAccelerationField,
)
from robot_simulation.disturbances.vector_field import (
    RestoringVectorField,
    VectorFieldParameters,
)
from robot_simulation.engine.simulation_engine import SimulationEngine
from robot_simulation.factory.registry import ComponentRegistry
from robot_simulation.integration.rk4 import RK4Integrator


def make_fight_flight(
    environment_field=None,
    **parameters: Any,
) -> FightFlightDynamics:
    """Construct Fight/Flight from parameter values."""
    return FightFlightDynamics(
        parameters=FightFlightParameters(**parameters),
        environment_field=environment_field,
    )


def make_restoring_field(
    center,
    gain_matrix,
) -> RestoringVectorField:
    """Construct a linear restoring acceleration field."""
    return RestoringVectorField(
        parameters=VectorFieldParameters(
            center=np.asarray(center, dtype=float),
            gain_matrix=np.asarray(gain_matrix, dtype=float),
        )
    )


def make_rk4() -> RK4Integrator:
    """Construct an RK4 integrator."""
    return RK4Integrator()


class SimulationFactory:
    """Assemble a simulation from registered component types."""

    def __init__(
        self,
        dynamics_registry: ComponentRegistry | None = None,
        field_registry: ComponentRegistry | None = None,
        integrator_registry: ComponentRegistry | None = None,
    ) -> None:
        self.dynamics_registry = (
            dynamics_registry
            if dynamics_registry is not None
            else ComponentRegistry()
        )
        self.field_registry = (
            field_registry
            if field_registry is not None
            else ComponentRegistry()
        )
        self.integrator_registry = (
            integrator_registry
            if integrator_registry is not None
            else ComponentRegistry()
        )

    @classmethod
    def with_defaults(cls) -> "SimulationFactory":
        """Register the currently implemented built-in components."""
        factory = cls()

        factory.dynamics_registry.register(
            "fight_flight", make_fight_flight
        )
        factory.field_registry.register(
            "linear_restoring", make_restoring_field
        )
        factory.integrator_registry.register(
            "rk4", make_rk4
        )

        return factory

    def create(
        self,
        target_config: Mapping[str, Any],
        environment_config: Mapping[str, Any],
        simulation_config: Mapping[str, Any],
    ) -> tuple[SimulationEngine, float]:
        """Build the simulation engine and timestep from YAML data."""
        target = target_config["target"]
        environment = environment_config["environment"]
        simulation = simulation_config["simulation"]

        if simulation["backend"] != "numerical":
            raise ValueError("Unsupported simulation backend.")

        fields = [
            self.field_registry.create(field_config)
            for field_config in environment.get(
                "acceleration_fields", []
            )
            if field_config.get("enabled", True)
        ]

        environment_field = CompositeAccelerationField(fields)

        dynamics = self.dynamics_registry.create(
            target["dynamics"],
            environment_field=environment_field,
        )

        integration_config = simulation["integration"]
        integrator = self.integrator_registry.create({
            "type": integration_config["type"],
            "parameters": {
                key: value
                for key, value in integration_config.get(
                    "parameters", {}
                ).items()
                if key != "dt"
            },
        })

        dt = float(integration_config["parameters"]["dt"])

        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("Integration dt must be positive.")

        engine = SimulationEngine(
            dynamics=dynamics,
            integrator=integrator,
        )

        return engine, dt
