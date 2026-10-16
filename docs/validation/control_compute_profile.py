"""Read-only launch prefix: startup call profile plus pipeline CPU/wall duration."""

import atexit
import cProfile
import json
import os
from pathlib import Path
import pstats
import runpy
import signal
import sys
import time

from robot_control.pipeline.control_execution_pipeline import ControlExecutionPipeline

destination = Path(os.environ['CONTROL_PROFILE_OUTPUT'])
original = ControlExecutionPipeline.compute
profile = cProfile.Profile(timer=time.thread_time)
samples = []


def compute(self, *args, **kwargs):
    """Observe original computations; profile only the first 40 startup solves."""
    wall, cpu = time.perf_counter(), time.thread_time()
    profiling = len(samples) < 40
    if profiling:
        profile.enable()
    try:
        return original(self, *args, **kwargs)
    finally:
        if profiling:
            profile.disable()
        samples.append([time.perf_counter() - wall, time.thread_time() - cpu, profiling])


def save():
    """Retain evidence after normal ROS launch shutdown."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    (destination / 'compute_cpu_wall.json').write_text(json.dumps(samples))
    profile.dump_stats(str(destination / 'control.prof'))
    with (destination / 'control_profile.txt').open('w') as stream:
        if samples:
            pstats.Stats(profile, stream=stream).sort_stats('cumulative').print_stats(35)


ControlExecutionPipeline.compute = compute
atexit.register(save)
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')
