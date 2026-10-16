"""Temporary compatibility alias; use robot_simulation.nodes.velocity_actuation."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_simulation.nodes.velocity_actuation')
