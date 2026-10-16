"""Complete only the missing M03 captures; preserve all existing outputs."""

import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
VALIDATION = ROOT / 'docs/validation'
CASES = [
    ('drive_after', 'config/reactive_tracking_oscillatory.yaml', 38, False, True),
    ('nominal_before', 'config/reactive_tracking_oscillatory.yaml', 60, True, False),
    ('nominal_after', 'config/reactive_tracking_oscillatory.yaml', 60, False, False),
    ('wide_final', 'config/reactive_tracking_oscillatory_aggressive.yaml', 60, False, False),
]


def main():
    records = []
    for i, (name, config, duration, before, drive) in enumerate(CASES):
        output = VALIDATION / ('m03_' + name)
        if output.exists():
            raise RuntimeError(f'Preserving existing output: {output}')
        command = [sys.executable, str(VALIDATION / 'v2_tracking_gazebo.py'),
            '--case', 'reactive', '--config', config, '--strategy-config', 'config/whole_body.yaml',
            '--physical-ground-truth', '--without-markers', '--shoulder-posture',
            '--estimator-trace', '--duration', str(duration), '--settling-time',
            '5' if drive else '10', '--wall-timeout', '240', '--output', str(output)]
        if before:
            command.append('--before-m03-estimation')
        if drive:
            command.extend(['--drive-fixture', str(VALIDATION / 'm03_drive.yaml')])
        print('Starting', name, flush=True)
        with (VALIDATION / (output.name + '.log')).open('w') as log:
            result = subprocess.run(command, cwd=ROOT, env=dict(os.environ, ROS_DOMAIN_ID=str(143+i)),
                                    stdout=log, stderr=subprocess.STDOUT)
        records.append(dict(name=name, command=command, domain=143+i, returncode=result.returncode))
        (VALIDATION / 'm03_remaining_runs.json').write_text(json.dumps(records, indent=2))
        print('Finished', name, 'exit', result.returncode, flush=True)
        if not (output / 'experiment_streams.json').exists():
            raise RuntimeError(f'Missing evidence: {name}')


if __name__ == '__main__':
    main()
