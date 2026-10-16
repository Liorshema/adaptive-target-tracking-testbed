"""Temporary compatibility alias; use robot_prediction.factory.system_factory."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_prediction.factory.system_factory')
