"""Observe actual estimator callback order without changing fusion decisions."""

import atexit
import json
import os
from pathlib import Path
import runpy
import sys
import time

from archived_estimation_imports import canonicalize_imports

if os.environ.get('M03_BASELINE_ESTIMATION'):
    import robot_estimation.robot.models.process_model as process_module
    import robot_estimation.robot.robot_state_estimator as estimator_module
    import robot_estimation.nodes.estimation_node as node_module
    baseline = Path(os.environ['M03_BASELINE_ESTIMATION'])
    for module, filename in [(process_module, 'process_model.py'),
                             (estimator_module, 'robot_state_estimator.py'),
                             (node_module, 'estimation_node.py')]:
        source = baseline / filename
        exec(compile(canonicalize_imports(source.read_text()), str(source), 'exec'), module.__dict__)

from robot_estimation.nodes.estimation_node import EstimationNode

events = []
commits = []
last_node = None


def instrument(source, method):
    original = getattr(EstimationNode, method)

    def observe(self, message):
        global last_node
        last_node = self
        stamp = self._stamp_to_seconds(message.header.stamp)
        previous = self._last_robot_time
        now = self.get_clock().now().nanoseconds * 1e-9
        rejected = getattr(self, '_time_rejections', 0)
        wall = time.monotonic()
        original(self, message)
        values = None
        try:
            if source == 'wheel':
                values = [message.twist.twist.linear.x, message.twist.twist.angular.z]
            elif source == 'joints':
                names = self.get_parameter('joint_names').value
                indices = [message.name.index(name) for name in names]
                values = [message.position[i] for i in indices] + [
                    message.velocity[i] for i in indices]
            elif source == 'imu':
                values = [message.angular_velocity.z]
        except (ValueError, IndexError):
            pass
        events.append(dict(source=source, stamp=stamp, previous=previous, now=now,
                           wall=wall, duration=time.monotonic() - wall,
                           rejected=getattr(self, '_time_rejections', 0) > rejected,
                           values=values))
    setattr(EstimationNode, method, observe)


for source, method in [('wheel', '_wheel_odometry_callback'),
                       ('joints', '_joint_state_callback'), ('imu', '_imu_callback')]:
    instrument(source, method)
predict = EstimationNode._predict_robot_if_needed


def observe_commit(self, stamp):
    previous = self._last_robot_time
    accepted = predict(self, stamp)
    commits.append(dict(stamp=stamp, previous=previous, accepted=accepted,
                        now=self.get_clock().now().nanoseconds * 1e-9))
    return accepted


EstimationNode._predict_robot_if_needed = observe_commit
atexit.register(lambda: Path(os.environ['ESTIMATION_TIMING_LOG']).write_text(json.dumps(events)))
atexit.register(lambda: Path(os.environ['ESTIMATION_TIMING_LOG']).with_name(
    'processing_commits.json').write_text(json.dumps(commits)))


def final_diagnostics():
    if last_node is None:
        return
    buffer = getattr(last_node, '_robot_buffer', None)
    constraint = getattr(last_node, '_joint_constraint', None)
    result = dict(filter_time=last_node._last_robot_time,
                  timing=None if buffer is None else buffer.diagnostics(),
                  fusion_errors=getattr(last_node, '_fusion_errors', None),
                  conditionings=getattr(constraint, 'conditionings', 0),
                  physical_violation_samples=getattr(constraint, 'physical_violation_samples', 0))
    Path(os.environ['ESTIMATION_TIMING_LOG']).with_name(
        'estimator_final_diagnostics.json').write_text(json.dumps(result, indent=2))


atexit.register(final_diagnostics)
executable = sys.argv.pop(1)
runpy.run_path(executable, run_name='__main__')
