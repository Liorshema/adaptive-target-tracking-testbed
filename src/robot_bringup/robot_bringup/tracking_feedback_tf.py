"""Temporary compatibility alias; use robot_bringup.nodes.tracking_feedback_tf."""

from importlib import import_module
import sys

sys.modules[__name__] = import_module('robot_bringup.nodes.tracking_feedback_tf')
