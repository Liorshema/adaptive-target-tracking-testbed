"""Compare matching target phases using archived measured-FK observations."""

import argparse
import json
from pathlib import Path
import re

import numpy as np
import yaml


def summarize(directory, thresholds):
    """Apply the same target-relative window to both complete experiments."""
    origin = float(re.search(r'Simulation initialized at t=([\d.]+)s',
                            (directory / 'runtime.log').read_text()).group(1))
    rows = [json.loads(line) for line in (directory / 'samples.jsonl').read_text().splitlines()]
    selected = [row for row in rows if thresholds['settling_time'] <= row['time'] - origin <= thresholds['duration']]
    if not selected or rows[-1]['time'] - origin < thresholds['duration'] - .1:
        raise ValueError('Incomplete experiment')
    errors = np.array([row['error'] for row in selected])
    return dict(target_initialization_time=origin, samples=len(selected),
                mean_position_error=float(errors.mean()),
                rmse_position_error=float(np.sqrt(np.mean(errors**2))),
                p95_position_error=float(np.quantile(errors, .95)),
                joint_excursions=np.ptp([row['joints'] for row in selected], axis=0).tolist())


def main():
    """Produce reviewable comparison evidence and fail if thresholds are missed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--active', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--thresholds', type=Path,
                        default=Path(__file__).with_name('closed_loop_thresholds.yaml'))
    args = parser.parse_args()
    thresholds = yaml.safe_load(args.thresholds.read_text())
    active = summarize(args.active, thresholds)
    baseline = summarize(args.baseline, thresholds)
    improvement = 1 - active['mean_position_error'] / baseline['mean_position_error']
    results = dict(active=active, baseline=baseline, relative_mean_improvement=improvement,
                   checks=dict(mean_error=active['mean_position_error'] <= thresholds['max_mean_position_error'],
                               rmse_error=active['rmse_position_error'] <= thresholds['max_rmse_position_error'],
                               meaningful_improvement=improvement >= thresholds['min_relative_improvement']))
    args.output.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    if not all(results['checks'].values()):
        raise RuntimeError('Tracking comparison failed')


if __name__ == '__main__':
    main()
