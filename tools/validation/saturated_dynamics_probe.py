"""Sweep deterministic target dynamics using the existing factory and RK4 engine."""

import argparse
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from robot_models.target.target_state import TargetState
from robot_configuration.yaml_loader import load_yaml
from robot_simulation.factory.simulation_factory import SimulationFactory
from robot_simulation.state.simulation_state import SimulationState
from robot_simulation.adapters.target_state import state_to_vector

ROOT = Path(__file__).resolve().parents[2]


def simulate(target, environment, simulation, initial, reference, duration, sample_period):
    engine, dt = SimulationFactory.with_defaults().create(target, environment, simulation)
    state = SimulationState(TargetState(np.array(initial['position']), np.array(initial['velocity'])),
                            np.array(reference['center']), 0.)
    rows = []
    next_sample = 0.
    for step in range(round(duration / dt) + 1):
        time = step * dt
        position = np.array(reference['center']) + np.array(reference['amplitude']) * np.sin(
            reference['angular_frequency'] * time)
        state = replace(state, tracking_position_world=position, time=time)
        if time + dt / 2 >= next_sample:
            vector = state_to_vector(state.target)
            acceleration = engine.dynamics.derivative(
                vector, np.empty(0), time, {'tracking_position_world': position})[3:]
            rows.append([time, *vector.tolist(), *acceleration.tolist(),
                         float(np.linalg.norm(vector[:3] - position))])
            next_sample += sample_period
        if step * dt < duration:
            state = engine.step(state, dt)
    return np.array(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=Path(__file__).with_name('saturated_dynamics_cases.yaml'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', help='Run only one configured candidate')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = load_yaml(args.cases)
    package = ROOT / 'src/robot_simulation'
    scenario = load_yaml(package / 'config/scenarios/reactive_target_saturated.yaml')
    target = load_yaml(package / scenario['target']['config'])
    environment = load_yaml(package / scenario['simulation']['environment_config'])
    simulation = load_yaml(package / scenario['simulation']['config'])
    center = np.array(environment['environment']['acceleration_fields'][0]['parameters']['center'])
    results = {}
    for name, parameters in cases['candidates'].items():
        if args.candidate is not None and name != args.candidate:
            continue
        configured = deepcopy(target)
        configured['target']['dynamics']['parameters'].update(parameters)
        for initial_name, initial in cases['initial_conditions'].items():
            for reference_name, reference in cases['references'].items():
                label = f'{name}_{initial_name}_{reference_name}'
                rows = simulate(configured, environment, simulation, initial, reference,
                                cases['duration'], cases['sample_period'])
                np.save(args.output / f'{label}.npy', rows)
                speed = np.linalg.norm(rows[:, 4:7], axis=1)
                distance = np.linalg.norm(rows[:, 1:4] - center, axis=1)
                results[label] = dict(parameters=parameters, initial=initial, reference=reference,
                    duration=cases['duration'], xyz_ranges=np.ptp(rows[:, 1:4], axis=0).tolist(),
                    max_center_distance=float(distance.max()), mean_speed=float(speed.mean()),
                    max_speed=float(speed.max()), max_acceleration=float(np.linalg.norm(rows[:, 7:10], axis=1).max()),
                    min_z=float(rows[:, 3].min()), max_z=float(rows[:, 3].max()),
                    finite=bool(np.isfinite(rows).all()),
                    practical_bound=bool(distance.max() < cases['max_center_distance'] and
                                         speed.max() < cases['max_speed']))
        print(name, 'completed', flush=True)
    # Repeat an entire nominal trajectory to check deterministic integration.
    repeated_name = args.candidate or 'gentle'
    parameters = cases['candidates'][repeated_name]
    target['target']['dynamics']['parameters'].update(parameters)
    repeated = simulate(target, environment, simulation, cases['initial_conditions']['nominal'],
                        cases['references']['moving'], cases['duration'], cases['sample_period'])
    deterministic = bool(np.array_equal(
        repeated, np.load(args.output / f'{repeated_name}_nominal_moving.npy')))
    result = {'cases': results, 'deterministic_repeat': deterministic}
    (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'count': len(results), 'deterministic': deterministic,
                      'all_finite': all(r['finite'] for r in results.values()),
                      'all_practically_bounded': all(r['practical_bound'] for r in results.values())}))


if __name__ == '__main__':
    main()
