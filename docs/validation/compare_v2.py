"""Render measured production comparisons and the v2 actuator command chain."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'docs/validation'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--v2', default='v2_final_recheck')
    args = parser.parse_args()
    folders = {'Hold': 'v2_hold', 'v1 DLS': 'v2_v1', 'v2': args.v2,
               'No feedforward': 'v2_no_feedforward', 'Zero lookahead': 'v2_no_lookahead'}
    results, samples = {}, {}
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
    for label, folder in folders.items():
        path = DATA / folder
        result = json.loads((path / 'result.json').read_text())
        results[label] = result
        rows = [json.loads(line) for line in (path / 'samples.jsonl').read_text().splitlines()]
        samples[label] = rows
        t = np.array([row['time'] for row in rows]) - result['target_initialization_time']
        axes[0, 0].plot(t, [abs(row['distance_error']) for row in rows], label=label)
        axes[0, 1].plot(t, [row['pointing_degrees'] for row in rows], label=label)
        base = np.array([row['base'] for row in rows])
        axes[1, 0].plot(base[:, 0], base[:, 1], label=label)
        rtf = np.array(json.loads((path / 'rtf.json').read_text()))
        axes[2, 0].plot(rtf[:, 0], rtf[:, 1], label=label)
    rows, result = samples['v2'], results['v2']
    t = np.array([row['time'] for row in rows]) - result['target_initialization_time']
    q = np.array([row['joints'] for row in rows])
    physical = yaml.safe_load((ROOT / 'src/robot_description/config/robot/robot.yaml').read_text())
    names = list(physical['robot']['manipulator']['joints'])
    for index, name in enumerate(names):
        axes[1, 1].plot(t, q[:, index], label=name)
    labels = list(results)
    axes[2, 1].bar(labels, [results[label]['weighted_motion_integral'] for label in labels])
    axes[2, 1].tick_params(axis='x', labelrotation=15)
    titles = ['Measured distance error', 'Measured optical pointing error',
              'Measured base trajectory', 'v2 measured arm configuration',
              'Observed real-time factor', 'Common normalized motion cost integral']
    ylabels = ['Absolute error (m)', 'Angle (degrees)', 'Odometry y (m)',
               'Joint angle (rad)', 'Simulation seconds / wall second', 'Cost × simulation seconds']
    for index, axis in enumerate(axes.flat):
        axis.set(title=titles[index], ylabel=ylabels[index])
        axis.grid(alpha=.25)
        if index < 5:
            axis.legend(fontsize=8)
        if index in [0, 1, 3, 4]:
            axis.set_xlabel('Seconds since target initialization')
            axis.axvline(result['thresholds']['settling_time'], color='gray', ls='--', lw=1)
    axes[1, 0].set_xlabel('Odometry x (m)')
    axes[1, 0].set_aspect('equal', adjustable='datalim')
    fig.savefig(DATA / 'whole_body_v2_comparison.png', dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), constrained_layout=True)
    rows = [row for row in rows if len(row['actuator_input']) == len(row['safe']) and
            len(row['base_command']) == 2 and len(row['position_reference']) == len(names)]
    t = np.array([row['time'] for row in rows]) - result['target_initialization_time']
    q = np.array([row['joints'] for row in rows])
    safe, raw = np.array([r['safe'] for r in rows]), np.array([r['raw'] for r in rows])
    actor = np.array([r['actuator_input'] for r in rows])
    twist = np.array([r['base_command'] for r in rows])
    measured = np.array([r['base_velocity'] for r in rows])
    reference = np.array([r['position_reference'] for r in rows])
    for index in range(2):
        axis = axes[0, index]
        for value, label in [(raw, 'Raw'), (safe, 'Safe'), (actor, 'Actuator input'),
                             (twist, 'Bridged Twist'), (measured, 'Measured odometry')]:
            axis.plot(t, value[:, index], label=label, alpha=.8)
        axis.set_title(['Base translation (m/s)', 'Base yaw (rad/s)'][index])
    for index, name in enumerate(names):
        axis = axes[1 + index // 2, index % 2]
        axis.plot(t, reference[:, index], label='PID position reference')
        axis.plot(t, q[:, index], label='Measured joint position')
        axis.set_title(name + ' (rad)')
    for axis in axes.flat:
        axis.set_xlabel('Seconds since target initialization')
        axis.grid(alpha=.25)
        axis.legend(fontsize=8)
    fig.savefig(DATA / 'whole_body_v2_actuation.png', dpi=150)
    plt.close(fig)
    summary = {'runs': results,
               'note': 'v1 controls position only; its smaller motion cost does not establish '
                       'equal-task efficiency. No-feedforward fails measured wrist speed.'}
    (DATA / 'whole_body_v2_comparison.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
