"""Temporary compatibility alias; use robot_control.nodes.control_node."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_control.nodes.control_node')
