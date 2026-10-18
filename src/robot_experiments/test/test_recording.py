"""Telemetry contracts exercise acquisition-independent message normalization."""
import importlib
import json
from types import SimpleNamespace as NS

import pytest


def recording():
    return importlib.import_module('robot_experiments.recording')


def header(t=2.5, frame='world'):
    seconds = int(t)
    return NS(stamp=NS(sec=seconds, nanosec=round((t-seconds)*1e9)), frame_id=frame)


def vector(*values):
    return NS(**dict(zip(('x', 'y', 'z', 'w'), values)))


def point(t=2.5, values=(1., 2., 3.), frame='world'):
    return NS(header=header(t, frame), point=vector(*values))


def store(tmp_path):
    return recording().TelemetryStore({'topics': {'clock': '/clock', 'target_truth': '/truth',
        'joints': '/joints', 'prediction': '/prediction'}}, tmp_path)


def test_envelope_preserves_source_frame_arrival_clock_and_original_payload():
    record = recording().normalize('target_truth', point(frame='foreign'), 17., 2.)
    assert record['schema_version'] == 1
    assert record['source_time'] == 2.5
    assert record['arrival_monotonic'] == 17.
    assert record['clock'] == 2.
    assert record['frame_id'] == 'foreign'
    assert record['data']['position'] == [1., 2., 3.]
    assert record['raw_message']['header']['frame_id'] == 'foreign'


def test_normalizes_full_joint_and_odometry_observations():
    joint = NS(header=header(), name=['other', 'axis'], position=[1., 2.],
               velocity=[3., 4.], effort=[5., 6.])
    result = recording().normalize('joints', joint, 1., None)
    assert result['data'] == {'names': ['other', 'axis'], 'position': [1., 2.],
                              'velocity': [3., 4.], 'effort': [5., 6.]}
    pose = NS(position=vector(1., 2., 3.), orientation=vector(0., 0., 0., 1.))
    twist = NS(linear=vector(4., 5., 6.), angular=vector(7., 8., 9.))
    odom = NS(header=header(), child_frame_id='base',
              pose=NS(pose=pose, covariance=[0.]*36),
              twist=NS(twist=twist, covariance=[1.]*36))
    data = recording().normalize('base_truth', odom, 1., 2.)['data']
    assert data['linear_velocity'] == [4., 5., 6.]
    assert data['angular_velocity'] == [7., 8., 9.]
    assert data['pose_covariance'] == [0.]*36
    assert data['twist_covariance'] == [1.]*36


def test_preserves_covariance_and_absolute_prediction_times():
    estimate = NS(header=header(), mean=[1., 2.], covariance=[1., 0., 0., 2.])
    assert recording().normalize('target_estimate', estimate, 1., 2.)['data']['covariance'] == estimate.covariance
    prediction = NS(header=header(), timestamps=[2.6, 2.7], states=list(range(12)),
                    horizon=2, state_dimension=6)
    data = recording().normalize('prediction', prediction, 1., 2.)['data']
    assert data['timestamps'] == [2.6, 2.7]
    assert data['states'] == [list(range(6)), list(range(6, 12))]


def test_preserves_controller_command_and_actuation_counters():
    pose = NS(position=vector(1., 2., 3.), orientation=vector(0., 0., 0., 1.))
    control = NS(header=header(), valid=True, reason='', actual_pose=pose,
                 desired_pose=pose, target_position=vector(4., 5., 6.), raw_velocity=[1., 2.],
                 safe_velocity=[0., 1.], computation_time=.01, task_error=[2.],
                 weighted_motion_cost=3., task_residual=4., active_bounds=1)
    data = recording().normalize('control', control, 1., 2.)['data']
    assert data['actual_position'] == [1., 2., 3.]
    assert data['target_position'] == [4., 5., 6.]
    assert data['task_error'] == [2.]
    command = NS(header=header(), velocity=[1., 2., 3.], joint_names=['axis'],
                 active=True, session_id='session', sequence=7)
    assert recording().normalize('command', command, 1., 2.)['data']['sequence'] == 7
    actuation = NS(header=header(), accepted_commands=10, rejected_commands=2,
                   timeout_events=3, reference_limit_events=4, inactive_commands=5,
                   feedback_rejections=6, commanded_velocity=[1., 2., 3.],
                   position_reference=[1.], measured_positions=[.9], enabled=True,
                   active=False, reason='watchdog')
    result = recording().normalize('actuation', actuation, 1., 2.)
    assert result['data']['timeout_events'] == 3
    assert result['data']['measured_positions'] == [.9]
    assert result['data']['feedback_rejections'] == 6


def test_rejected_nonfinite_or_bad_shapes_never_make_stream_ready(tmp_path):
    telemetry = store(tmp_path)
    telemetry.record('target_truth', point(values=(float('nan'), 2., 3.)), 1.)
    telemetry.record('prediction', NS(header=header(), horizon=2, state_dimension=6,
                     timestamps=[3., 4.], states=[0.]), 2.)
    assert not telemetry.ready(['target_truth'])
    assert telemetry.streams['target_truth'] == []
    assert telemetry.availability['target_truth']['invalid_count'] == 1
    telemetry.close()
    events = [json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert len(events) == 2
    assert all(event['type'] == 'rejected_message' for event in events)
    assert events[0]['raw_message']['point']['x'] == {'nonfinite': 'nan'}


def test_append_only_out_of_order_sources_and_missing_metadata(tmp_path):
    telemetry = store(tmp_path)
    telemetry.record('target_truth', point(t=3.), 10.)
    telemetry.record('target_truth', point(t=2.), 11.)
    assert telemetry.ready(['target_truth'])
    assert not telemetry.ready(['target_truth', 'joints'])
    assert [r['source_time'] for r in telemetry.streams['target_truth']] == [3., 2.]
    assert telemetry.availability['target_truth']['out_of_order_count'] == 1
    assert telemetry.availability['joints']['record_count'] == 0
    assert telemetry.availability['target_truth']['last_source_time'] == 2.
    telemetry.close()
    telemetry.close()
    records = [json.loads(line) for line in (tmp_path/'target_truth.jsonl').read_text().splitlines()]
    assert [r['source_time'] for r in records] == [3., 2.]
    assert records[0]['provenance']['topic'] == '/truth'


def test_clock_progress_reset_and_no_header_diagnostic_time(tmp_path):
    telemetry = store(tmp_path)
    telemetry.record('clock', NS(clock=NS(sec=9, nanosec=0)), 10.)
    assert telemetry.clock == 9.
    assert telemetry.last_clock_monotonic == 10.
    telemetry.record('clock', NS(clock=NS(sec=9, nanosec=0)), 11.)
    assert telemetry.last_clock_monotonic == 10.
    telemetry.record('clock', NS(clock=NS(sec=8, nanosec=0)), 12.)
    assert telemetry.clock_reset
    telemetry.record('diagnostics', NS(data='{"state":"ok"}'), 13.)
    record = telemetry.streams['diagnostics'][0]
    assert record['source_time'] is None
    assert record['data']['parsed'] == {'state': 'ok'}
    assert record['clock'] == 8.
    assert any(event['type'] == 'clock_reset' for event in telemetry.events)
    telemetry.close()


@pytest.mark.parametrize('stream,message', [
    ('target_estimate', NS(header=header(), mean=[1., 2.], covariance=[1.])),
    ('prediction', NS(header=header(), horizon=2, state_dimension=6,
                      timestamps=[3., 2.], states=[0.]*12)),
    ('joints', NS(header=header(), name=['axis'], position=[0.], velocity=[1., 2.])),
])
def test_shape_errors_are_explicit(stream, message):
    with pytest.raises(ValueError):
        recording().normalize(stream, message, 1., 2.)


def fake_ros(monkeypatch, fail_subscription=False):
    """Replace only the ROS transport boundary; normalization/storage remain real."""
    import sys
    from types import ModuleType
    resources = NS(contexts=[], nodes=[], executors=[])

    class Context:
        def __init__(self):
            self.closed = False
            resources.contexts.append(self)

        def init(self, args):
            self.args = args

        def try_shutdown(self):
            self.closed = True

    class Node:
        def __init__(self, name, context):
            self.context, self.closed, self.subscriptions = context, False, []
            resources.nodes.append(self)

        def create_subscription(self, message_type, topic, callback, qos):
            if fail_subscription:
                raise RuntimeError('transport unavailable')
            self.subscriptions.append((message_type, topic, callback, qos))

        def destroy_node(self):
            self.closed = True

    class Executor:
        def __init__(self, context):
            self.context, self.closed, self.timeouts = context, False, []
            resources.executors.append(self)

        def add_node(self, node):
            self.node = node

        def spin_once(self, timeout_sec):
            self.timeouts.append(timeout_sec)

        def shutdown(self, timeout_sec):
            self.closed = True

        def remove_node(self, node):
            pass

    class QoS:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    modules = {
        'rclpy.context': {'Context': Context}, 'rclpy.node': {'Node': Node},
        'rclpy.executors': {'SingleThreadedExecutor': Executor},
        'rclpy.qos': {'QoSProfile': QoS, 'ReliabilityPolicy': NS(BEST_EFFORT='best_effort'),
                      'DurabilityPolicy': NS(VOLATILE='volatile')},
    }
    for _, (module, name) in recording().normalization.STREAM_TYPES.items():
        modules.setdefault(module, {})[name] = type(name, (), {})
    for name, fields in modules.items():
        module = ModuleType(name)
        module.__dict__.update(fields)
        monkeypatch.setitem(sys.modules, name, module)
    return resources


def test_ros_recorder_requires_observed_messages_and_owns_context(tmp_path, monkeypatch):
    resources = fake_ros(monkeypatch)
    recorder = recording().Recorder({'topics': {'clock': '/clock', 'target_truth': '/target'}}, tmp_path)
    assert not recorder.ready(['target_truth'])
    node = resources.nodes[0]
    assert node.context is resources.contexts[0]
    for _, topic, callback, qos in node.subscriptions:
        assert qos.reliability == 'best_effort'
        if topic == '/clock':
            callback(NS(clock=NS(sec=3, nanosec=0)))
        elif topic == '/target':
            callback(point())
    assert recorder.ready(['clock', 'target_truth'])
    assert recorder.clock == 3.
    recorder.spin_once()
    assert resources.executors[0].timeouts == [.05]
    recorder.close()
    recorder.close()
    assert all(resource.closed for resource in [*resources.nodes, *resources.contexts,
                                                *resources.executors])
    assert json.loads((tmp_path/'target_truth.jsonl').read_text())['clock'] == 3.


def test_ros_constructor_failure_closes_owned_resources(tmp_path, monkeypatch):
    resources = fake_ros(monkeypatch, fail_subscription=True)
    with pytest.raises(RuntimeError, match='transport unavailable'):
        recording().Recorder({'topics': {'clock': '/clock'}}, tmp_path)
    assert all(resource.closed for resource in [*resources.nodes, *resources.contexts,
                                                *resources.executors])
    assert (tmp_path/'events.jsonl').exists()


def queued_executor(resources, monkeypatch, messages):
    callbacks = {topic: callback for _, topic, callback, _ in resources.nodes[0].subscriptions}
    executor = resources.executors[0]
    queue = list(messages)

    def spin_once(timeout_sec):
        executor.timeouts.append(timeout_sec)
        if queue:
            topic, message = queue.pop(0)
            callbacks[topic](message)

    monkeypatch.setattr(executor, 'spin_once', spin_once)
    return queue


def test_ros_spin_drains_available_streams_and_preserves_source_clock(tmp_path, monkeypatch):
    resources = fake_ros(monkeypatch)
    recorder = recording().Recorder({'topics': {'clock': '/clock', 'target_truth': '/target'}}, tmp_path)
    queue = queued_executor(resources, monkeypatch, [
        ('/clock', NS(clock=NS(sec=3, nanosec=0))), ('/target', point(t=2.5)),
        ('/clock', NS(clock=NS(sec=4, nanosec=0))), ('/target', point(t=3.5)),
    ])
    recorder.spin_once(timeout=.025)
    assert not queue
    assert recorder.clock == 4.
    assert [r['source_time'] for r in recorder.streams['target_truth']] == [2.5, 3.5]
    assert [r['clock'] for r in recorder.streams['target_truth']] == [3., 4.]
    assert resources.executors[0].timeouts == [.025, 0., 0., 0., 0.]
    recorder.close()


def test_ros_spin_bounds_work_even_with_continuously_available_messages(tmp_path, monkeypatch):
    resources = fake_ros(monkeypatch)
    recorder = recording().Recorder({'topics': {'target_truth': '/target'}}, tmp_path)
    monkeypatch.setattr(importlib.import_module('robot_experiments.recording.ros'),
                        'monotonic', lambda: 0., raising=False)
    queue = queued_executor(resources, monkeypatch, [('/target', point())] * 400)
    recorder.spin_once()
    assert len(recorder.streams['target_truth']) == 128
    assert len(queue) == 272
    recorder.close()


def test_ros_spin_stops_draining_when_wall_budget_is_exhausted(tmp_path, monkeypatch):
    resources = fake_ros(monkeypatch)
    recorder = recording().Recorder({'topics': {'target_truth': '/target'}}, tmp_path)
    moments = iter([0., .005, .011])
    monkeypatch.setattr(importlib.import_module('robot_experiments.recording.ros'),
                        'monotonic', lambda: next(moments), raising=False)
    queue = queued_executor(resources, monkeypatch, [('/target', point())] * 400)
    recorder.spin_once()
    assert len(recorder.streams['target_truth']) == 2
    assert len(queue) == 398
    recorder.close()

def test_readiness_can_require_fresh_source_epochs(tmp_path):
    telemetry=recording().TelemetryStore({'readiness_max_age':.08},tmp_path)
    telemetry.record('clock',NS(clock=NS(sec=3,nanosec=0)),1.)
    telemetry.record('target_truth',point(2.0),1.1)
    assert not telemetry.ready(['clock','target_truth'])
    telemetry.record('target_truth',point(2.98),1.2)
    assert telemetry.ready(['clock','target_truth'])
