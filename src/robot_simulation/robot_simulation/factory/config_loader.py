"""Temporary compatibility alias; use robot_configuration.yaml_loader."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_configuration.yaml_loader')
