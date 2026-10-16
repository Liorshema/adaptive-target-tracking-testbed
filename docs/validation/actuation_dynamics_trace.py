"""Observe source timestamps of emitted PID position references without changing them."""

import atexit
import json
import os
from pathlib import Path
import runpy
import sys

if os.environ.get('M02_BASELINE_ACTUATION'):
    # Read-only diagnostic reproduction: load the exact archived pre-fix adapter.
    import robot_simulation.actuation.position_reference as reference_module
    import robot_simulation.nodes.velocity_actuation as actuation_module
    baseline = Path(os.environ['M02_BASELINE_ACTUATION'])
    for module, filename in [(reference_module, 'position_reference.py'),
                             (actuation_module, 'velocity_actuation.py')]:
        source = baseline / filename
        exec(compile(source.read_text(), str(source), 'exec'), module.__dict__)

from robot_simulation.nodes.velocity_actuation import VelocityActuationNode

events = []
send = VelocityActuationNode._send


def observe(self, velocity=None):
    send(self, velocity)
    reference = self._reference.position
    joint = self._joint
    command = self._command
    guard = getattr(self, '_position_guard', None)
    events.append(dict(time=self._now(), active=self._was_active,
                       reference=None if reference is None else reference.tolist(),
                       joint_time=None if joint is None else joint[0],
                       measured=None if joint is None else joint[2].tolist(),
                       command_time=None if command is None else command[0],
                       command=None if command is None else command[2].tolist(),
                       reason=self._reason,
                       position_mode=getattr(guard, 'mode', None),
                       recovery_side=[] if guard is None else guard.side.tolist(),
                       recovery_events=getattr(guard, 'events', 0)))


VelocityActuationNode._send = observe
atexit.register(lambda: Path(os.environ['ACTUATION_DYNAMICS_TRACE']).write_text(json.dumps(events)))
executable = sys.argv.pop(1)
runpy.run_path(executable, run_name='__main__')
