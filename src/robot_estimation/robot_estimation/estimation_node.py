"""Temporary compatibility alias; use robot_estimation.nodes.estimation_node."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_estimation.nodes.estimation_node')
