"""Run the final M02 cases sequentially without changing production configuration."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
VALIDATION = ROOT / 'results/validation'
CASES = [
    ('A_before', 'config/reactive_tracking_oscillatory.yaml', 60, True),
    ('A_after', 'config/reactive_tracking_oscillatory.yaml', 60, False),
    ('B_before', 'config/reactive_tracking_oscillatory_aggressive.yaml', 90, True),
    ('B_after', 'config/reactive_tracking_oscillatory_aggressive.yaml', 90, False),
]


def main():
    results = []
    for i, (name, configuration, duration, before) in enumerate(CASES):
        output = VALIDATION / ('m02_quiet_' + name)
        if output.exists():
            raise RuntimeError(f'Refusing to overwrite existing evidence: {output}')
        command = [sys.executable, str(Path(__file__).with_name('v2_tracking_gazebo.py')),
            '--case', 'reactive', '--config', configuration,
            '--strategy-config', 'config/whole_body.yaml', '--physical-ground-truth',
            '--without-markers', '--shoulder-posture', '--actuation-trace',
            '--duration', str(duration), '--settling-time', '10', '--wall-timeout', '300',
            '--output', str(output)]
        if before:
            command.append('--before-m02-actuation')
        print('Starting', name, flush=True)
        env = dict(os.environ, ROS_DOMAIN_ID=str(130 + i))
        start = time.monotonic()
        with (VALIDATION / (output.name + '.log')).open('w') as log:
            completed = subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                                       stderr=subprocess.STDOUT)
        result = dict(case=name, command=command, domain=130 + i,
                      returncode=completed.returncode, wall_seconds=time.monotonic() - start)
        results.append(result)
        (VALIDATION / 'm02_comparison_runs.json').write_text(json.dumps(results, indent=2))
        print('Finished', name, 'exit', completed.returncode, flush=True)
        # Safety gate failures remain failures; retain and independently audit their raw evidence.
        if not (output / 'experiment_streams.json').exists():
            raise RuntimeError(f'No physical evidence for {name}')
    print('All four evidence captures completed', flush=True)


if __name__ == '__main__':
    main()
