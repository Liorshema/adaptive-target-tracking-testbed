"""Telemetry capture with a pure normalization core and a lazy ROS adapter."""
from .normalization import normalize
from .storage import TelemetryStore


def __getattr__(name):
    if name == 'Recorder':
        from .ros import Recorder
        return Recorder
    raise AttributeError(name)


__all__ = ['normalize', 'TelemetryStore', 'Recorder']
