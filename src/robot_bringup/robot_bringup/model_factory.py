"""Temporary compatibility alias; use robot_bringup.factory.model_factory."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_bringup.factory.model_factory')
