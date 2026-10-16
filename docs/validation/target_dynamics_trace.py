"""Read-only trace of exact simulated target state and the live TF integration input."""

import atexit
import json
import os
from pathlib import Path
import runpy
import signal
import sys

import numpy as np
from robot_simulation.nodes.simulation_node import SimulationNode
from robot_simulation.adapters.target_state import state_to_vector

states, references = [], []
publish = SimulationNode._publish_ground_truth
read = SimulationNode._read_tracking_position


def observe_state(self, state):
    publish(self, state)
    vector = state_to_vector(state.target)
    acceleration = self._stepper._engine.dynamics.derivative(
        vector, np.empty(0), state.time,
        {'tracking_position_world': state.tracking_position_world})[3:]
    states.append([state.time, *vector.tolist(), *acceleration.tolist(),
                   *state.tracking_position_world.tolist()])


def observe_reference(self):
    position = read(self)
    references.append([self.get_clock().now().nanoseconds * 1e-9,
                       None if position is None else position.tolist()])
    return position


SimulationNode._publish_ground_truth = observe_state
SimulationNode._read_tracking_position = observe_reference
def save():
    """Finish observational output despite repeated launch shutdown interrupts."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    Path(os.environ['TARGET_DYNAMICS_TRACE']).write_text(
        json.dumps({'states': states, 'references': references}))


atexit.register(save)
executable = sys.argv.pop(1)
runpy.run_path(executable, run_name='__main__')
