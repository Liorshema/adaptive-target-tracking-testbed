
"""Record a deterministic target trajectory for diagnostics."""

import argparse
import csv
from pathlib import Path

import numpy as np

from robot_models.types import TargetState

from robot_simulation.factory.config_loader import load_yaml
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.integration.time_stepper import TimeStepper
from robot_simulation.state.simulation_state import SimulationState


def record_trajectory(
    config_dir: Path,
    initial_position: np.ndarray,
    initial_velocity: np.ndarray,
    tracking_position: np.ndarray,
    duration: float,
    sample_period: float,
) -> list[dict]:
    """Integrate and sample a target trajectory."""
    target_config = load_yaml(config_dir / "target.yaml")
    environment_config = load_yaml(config_dir / "environment.yaml")
    simulation_config = load_yaml(config_dir / "simulation.yaml")

    engine, dt = SimulationFactory.with_defaults().create(
        target_config,
        environment_config,
        simulation_config,
    )

    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive and finite.")

    if not np.isfinite(sample_period) or sample_period <= 0:
        raise ValueError("sample_period must be positive and finite.")

    max_steps = int(
        simulation_config["simulation"]["integration"]["max_steps"]
    )

    stepper = TimeStepper(
        engine=engine,
        dt=dt,
        max_steps=max_steps,
    )

    state = SimulationState(
        target=TargetState(
            position=np.asarray(initial_position, dtype=float),
            velocity=np.asarray(initial_velocity, dtype=float),
        ),
        tracking_position_world=np.asarray(
            tracking_position, dtype=float
        ),
        time=0.0,
    )

    rows = []

    def append_sample():
        position = state.target.position
        velocity = state.target.velocity

        distance = np.linalg.norm(
            position - state.tracking_position_world
        )

        rows.append({
            "time": state.time,
            "x": position[0],
            "y": position[1],
            "z": position[2],
            "vx": velocity[0],
            "vy": velocity[1],
            "vz": velocity[2],
            "distance": distance,
            "speed": np.linalg.norm(velocity),
        })

    append_sample()

    # Include the final time, even when duration is not a
    # multiple of sample_period.
    sample_times = list(
        np.arange(sample_period, duration, sample_period)
    )
    sample_times.append(duration)

    for target_time in sample_times:
        state = stepper.advance(
            state,
            target_time=float(target_time),
        )
        append_sample()

    return rows


def main():
    """Run numerical trajectory diagnostics from the command line."""
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--config-dir", type=Path, required=True
    )
    parser.add_argument(
        "--initial-position",
        type=float, nargs=3, required=True
    )
    parser.add_argument(
        "--initial-velocity",
        type=float, nargs=3, required=True
    )
    parser.add_argument(
        "--tracking-position",
        type=float, nargs=3, required=True
    )
    parser.add_argument(
        "--duration", type=float, required=True
    )
    parser.add_argument(
        "--sample-period", type=float, required=True
    )
    parser.add_argument(
        "--output", type=Path, required=True
    )

    args = parser.parse_args()

    rows = record_trajectory(
        config_dir=args.config_dir,
        initial_position=np.array(args.initial_position),
        initial_velocity=np.array(args.initial_velocity),
        tracking_position=np.array(args.tracking_position),
        duration=args.duration,
        sample_period=args.sample_period,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)

    with args.output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file, fieldnames=list(rows[0])
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Samples: {len(rows)}")
    print(f"CSV: {args.output}")
    print()
    print("time       x          distance    speed")

    for row in rows:
        print(
            f"{row['time']:7.2f}  "
            f"{row['x']:10.5f}  "
            f"{row['distance']:10.5f}  "
            f"{row['speed']:10.5f}"
        )


if __name__ == "__main__":
    main()
