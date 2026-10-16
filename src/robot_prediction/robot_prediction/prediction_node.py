"""Temporary compatibility alias; use robot_prediction.nodes.prediction_node."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_prediction.nodes.prediction_node')
