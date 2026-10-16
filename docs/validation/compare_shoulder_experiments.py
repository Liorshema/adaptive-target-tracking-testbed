"""Reproduce Fight/Flight comparisons from preserved raw production captures."""

import argparse
import json
from pathlib import Path
import re

import numpy as np
import yaml

from shoulder_experiment_metrics import experiment_metrics


ROOT = Path(__file__).resolve().parents[2]


def analyze(path):
    """Include stopped/saturated control periods in independent physical metrics."""
    snapshot = yaml.safe_load((path / 'configuration_snapshot.yaml').read_text())
    # YAML snapshot serialization sorts mappings; raw vectors retain description order.
    # These captures share the unchanged ordered description; record that order explicitly.
    description = yaml.safe_load(
        (ROOT / 'src/robot_description/config/robot/robot.yaml').read_text())['robot']
    assert description == snapshot['description'], 'Description differs from captured configuration'
    robot = yaml.safe_load((ROOT / 'src/robot_simulation' /
                           snapshot['simulation']['robot']['config']).read_text())['robot']
    streams = json.loads((path / 'experiment_streams.json').read_text())
    origin = float(re.search(r'Simulation initialized at t=([\d.]+)s',
                            (path / 'runtime.log').read_text()).group(1))
    thresholds = yaml.safe_load((Path(__file__).parent / 'v2_thresholds.yaml').read_text())
    thresholds['duration'] = 40.
    measurements = experiment_metrics(path, streams, description, robot, origin, thresholds,
                                      control=snapshot['control'])
    rows = [json.loads(line) for line in (path / 'samples.jsonl').read_text().splitlines()]
    selected = [row for row in rows
                if 10. <= row['time'] - origin <= 40.]
    bounds = list(description['manipulator']['joints'].values())
    low, high = np.array([b['lower'] for b in bounds]), np.array([b['upper'] for b in bounds])
    q = np.array([entry[1] for entry in streams['joints']])
    velocities = np.abs([row['joint_velocity'] for row in selected]).max(axis=0)
    speed_limits = np.array([b['velocity'] for b in bounds])
    command_limits = np.r_[np.maximum(np.abs(snapshot['control']['base_velocity_lower']),
                                     np.abs(snapshot['control']['base_velocity_upper'])), speed_limits]
    tolerance = thresholds['physical_numerical_tolerance']
    counter = selected[-1]['counters']
    first_counter = selected[0]['counters']
    result = dict(
        source=str(path.relative_to(ROOT)), duration=40., settling_time=10.,
        target_initialization_time=origin, joint_order=list(description['manipulator']['joints']),
        measurements=measurements, measured_joint_speed_peaks=velocities.tolist(),
        joint_speed_limits=speed_limits.tolist(),
        actuator_counters=counter,
        base_displacement=float(np.max(np.linalg.norm(
            np.array([row['base'][:2] for row in selected]) - selected[0]['base'][:2], axis=1))),
        joint_excursions=np.ptp([row['joints'] for row in selected], axis=0).tolist(),
        checks=dict(
            physical_joint_positions=bool(np.all(q >= low - tolerance) and
                                          np.all(q <= high + tolerance)),
            first_link_horizontal=measurements['minimum_link_elevation'] >= -tolerance,
            first_link_base_clearance=measurements['minimum_first_link_box_base_clearance'] >= 0.,
            safe_command_velocities=bool(np.all(np.abs([row['safe'] for row in selected]) <=
                                               command_limits + tolerance)),
            measured_joint_velocities=bool(np.all(velocities <= speed_limits + tolerance)),
            no_settled_watchdog_timeouts=counter['timeout_events'] == first_counter['timeout_events']))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path,
                        default=ROOT / 'docs/validation/shoulder_reactive_verified')
    parser.add_argument('--experiment', type=Path,
                        default=ROOT / 'docs/validation/shoulder_reactive_low_damping')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'docs/validation/shoulder_fight_flight_comparison.json')
    args = parser.parse_args()
    result = {'baseline': analyze(args.baseline.resolve()),
              'low_damping': analyze(args.experiment.resolve())}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
