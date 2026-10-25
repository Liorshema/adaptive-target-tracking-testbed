"""Summarize preserved production captures and export trajectory/error figures."""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


def summarize(path):
    result = json.loads((path / 'result.json').read_text())
    snapshot = yaml.safe_load((path / 'configuration_snapshot.yaml').read_text())
    trace = json.loads((path / 'target_trace.json').read_text())
    states = np.array(trace['states'])
    origin, duration = result['target_initialization_time'], result['thresholds']['duration']
    full = states[(states[:, 0] >= origin) & (states[:, 0] <= origin + duration)]
    selected = full[full[:, 0] >= origin + result['thresholds']['settling_time']]
    center = np.array(snapshot['environment']['environment']['acceleration_fields'][0]['parameters']['center'])
    speed, acceleration = np.linalg.norm(selected[:, 4:7], axis=1), np.linalg.norm(full[:, 7:10], axis=1)
    experiment = result['experiment']
    limits = np.array([j['velocity'] for j in snapshot['description']['manipulator']['joints'].values()])
    raw_peaks = np.array(experiment['raw_joint_speed_peaks'])
    tolerance = result['thresholds']['physical_numerical_tolerance']
    acceptance = yaml.safe_load(Path(__file__).with_name('saturated_dynamics_cases.yaml').read_text())
    reference_rows = [r for r in trace['references'] if origin + 10. <= r[0] <= origin + duration]
    valid_fraction = sum(r[1] is not None for r in reference_rows) / len(reference_rows)
    data = dict(duration=duration, initial_position=snapshot['simulation']['target']['initial_position'],
                initial_velocity=snapshot['simulation']['target']['initial_velocity'],
                dynamics=snapshot['target']['target']['dynamics'],
                mean_target_speed=float(speed.mean()), max_target_speed=float(speed.max()),
                max_target_acceleration=float(acceleration.max()),
                max_center_displacement=float(np.linalg.norm(full[:, 1:4] - center, axis=1).max()),
                xyz_ranges_full=np.ptp(full[:, 1:4], axis=0).tolist(),
                xyz_ranges_settled=np.ptp(selected[:, 1:4], axis=0).tolist(),
                mean_distance_error=experiment['mean_physical_distance_error'],
                max_distance_error=experiment['max_physical_distance_error'],
                mean_pointing_degrees=experiment['mean_physical_pointing_degrees'],
                max_pointing_degrees=experiment['max_physical_pointing_degrees'],
                max_measured_joint_speed=float(raw_peaks.max()), measured_joint_speed_peaks=raw_peaks.tolist(),
                physical_speed_check=bool(np.all(raw_peaks <= limits + tolerance)),
                minimum_joint_limit_margin_settled=result['minimum_joint_limit_margin'],
                minimum_shoulder=experiment['minimum_shoulder'],
                minimum_first_link_clearance=experiment['minimum_first_link_box_base_clearance'],
                mean_rtf=result['mean_rtf'], actuation_counters=result['counters'],
                controller_saturation_cycles=result['saturation_cycles'],
                blocked_cycles=result['blocked_cycles'], settled_tf_available_fraction=valid_fraction,
                finite=bool(np.isfinite(full).all()), bounded_workspace=bool(
                    np.linalg.norm(full[:, 1:4] - center, axis=1).max() < acceptance['closed_loop_workspace_radius']),
                source=str(path.relative_to(ROOT)))
    parameters = snapshot['target']['target']['dynamics']['parameters']
    if 'a_max' in parameters:
        displacement = full[:, 1:4] - full[:, 10:13]
        radius = np.linalg.norm(displacement, axis=1)
        radial = parameters['a_max'] * np.tanh((radius - parameters['preferred_distance']) /
                                               parameters['transition_length'])
        interaction = radial[:, None] * displacement / np.sqrt(
            radius[:, None] ** 2 + parameters['distance_epsilon'] ** 2)
        frozen = full[:, 1:4] - full[0, 10:13]
        frozen_radius = np.linalg.norm(frozen, axis=1)
        frozen_force = parameters['a_max'] * np.tanh((frozen_radius - parameters['preferred_distance']) /
            parameters['transition_length'])[:, None] * frozen / np.sqrt(
                frozen_radius[:, None] ** 2 + parameters['distance_epsilon'] ** 2)
        data['max_interaction_acceleration'] = float(np.linalg.norm(interaction, axis=1).max())
        data['live_vs_frozen_reference_force_rms'] = float(np.sqrt(
            np.mean(np.sum((interaction - frozen_force) ** 2, axis=1))))
    return data, full, origin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'results/validation')
    args = parser.parse_args()
    paths = {'linear_baseline': ROOT / 'results/validation/saturated_linear_baseline',
             'linear_low_damping': ROOT / 'results/validation/saturated_linear_low_damping',
             'saturated': ROOT / 'results/validation/saturated_final_120'}
    result, arrays, origins = {}, {}, {}
    for name, path in paths.items():
        result[name], arrays[name], origins[name] = summarize(path)
    result['comparison_note'] = 'Different initial conditions and durations: observational comparison, not controlled.'
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'saturated_comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'linear_baseline': '#2563eb', 'linear_low_damping': '#dc2626', 'saturated': '#059669'}
    figure = plt.figure(figsize=(15, 4.5))
    for i, (name, data) in enumerate(arrays.items()):
        axes = figure.add_subplot(1, 3, i + 1, projection='3d')
        axes.plot(*data[:, 1:4].T, color=colors[name], linewidth=1.4)
        axes.scatter(*data[0, 1:4], color='black', s=20, label='Start')
        axes.set_title(name.replace('_', ' ') + f" ({result[name]['duration']:g} s)")
        axes.set_xlabel('World X [m]'); axes.set_ylabel('World Y [m]'); axes.set_zlabel('World Z [m]')
    figure.suptitle('Measured target trajectories — independent axis scales; low damping escapes')
    figure.tight_layout()
    figure.savefig(args.output / 'saturated_trajectories.png', dpi=160)
    plt.close(figure)
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for name, data in arrays.items():
        elapsed = data[:, 0] - origins[name]
        axes[0].semilogy(elapsed, np.maximum(np.linalg.norm(data[:, 4:7], axis=1), 1e-5),
                         color=colors[name], label=name.replace('_', ' '))
        physical = np.array(json.loads((paths[name] / 'physical_tracking.json').read_text()))
        axes[1].semilogy(physical[:, 0] - origins[name], np.maximum(np.abs(physical[:, 1]), 1e-5),
                         color=colors[name])
        axes[2].plot(physical[:, 0] - origins[name], physical[:, 2], color=colors[name])
    axes[0].set_ylabel('Target speed [m/s]'); axes[0].legend()
    axes[1].set_ylabel('Absolute range error [m]')
    axes[2].set_ylabel('Actual +X pointing error [deg]'); axes[2].set_xlabel('Elapsed simulation time [s]')
    for ax in axes:
        ax.grid(alpha=.25)
    figure.tight_layout()
    figure.savefig(args.output / 'saturated_tracking.png', dpi=160)
    plt.close(figure)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
