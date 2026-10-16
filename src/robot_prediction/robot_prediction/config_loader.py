"""Temporary compatibility alias; use robot_prediction.configuration.prediction_config."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_prediction.configuration.prediction_config')
