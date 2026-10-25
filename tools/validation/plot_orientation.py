"""Render actual Gazebo tracking axes before and after the optical-axis correction."""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

DATA = Path(__file__).resolve().parents[2] / 'results/validation'


def main():
    figure = plt.figure(figsize=(13, 6))
    cases = [('orientation_z_baseline', 'Before: +Z controlled, +X intended'),
             ('orientation_x_corrected', 'After: intended +X controlled')]
    for index, (directory, title) in enumerate(cases):
        row = json.loads((DATA / directory / 'summary.json').read_text())['snapshot']
        pose, target = np.asarray(row['physical_pose']), np.asarray(row['target'])
        origin = pose[:3, 3]
        plot = figure.add_subplot(1, 2, index + 1, projection='3d')
        plot.scatter(*origin, color='black', s=40, label='Actual tracking origin')
        for axis, (name, color) in enumerate([('+X shaft', 'red'), ('+Y', 'green'), ('+Z', 'blue')]):
            plot.quiver(*origin, *(.3 * pose[:3, axis]), color=color,
                        label=name, arrow_length_ratio=.18)
        plot.quiver(*origin, *(.8 * np.asarray(row['optical_axis'])), color='gold',
                    linewidth=3, label='Controlled optical axis', arrow_length_ratio=.12)
        plot.plot(*np.column_stack([origin, target]), color='magenta', ls='--',
                  label='Direction to actual target')
        plot.scatter(*target, color='magenta', s=40)
        inferred = np.asarray(row['modeled_pose'])
        local_forward = pose[:3, :3].T @ np.asarray(row['optical_axis'])
        plot.quiver(*inferred[:3, 3], *(.8 * inferred[:3, :3] @ local_forward),
                    color='gray', alpha=.5, label='Wheel-odometry inference')
        plot.set(title=title, xlabel='World X (m)', ylabel='World Y (m)', zlabel='World Z (m)')
        plot.set_zlim(-.1, 1.2)
        plot.set_box_aspect((1.1, .7, 1.3))
        plot.view_init(elev=18, azim=-58)
        plot.legend(fontsize=8, loc='upper left')
        plot.text2D(.02, -.08, f"t={row['time']:.3f} s; intended +X error="
                    f"{row['shaft_pointing']:.2f}°; distance={row['distance_error']+1:.3f} m",
                    transform=plot.transAxes)
    figure.tight_layout()
    figure.savefig(DATA / 'orientation_axes.png', dpi=160, bbox_inches='tight')


if __name__ == '__main__':
    main()
