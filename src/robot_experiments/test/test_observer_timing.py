"""Opt-in observer diagnostics preserve scientific data and bounded servicing."""

import importlib.util
import json
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from robot_experiments.recording.observer_timing import ObserverTiming
from robot_experiments.recording.storage import TelemetryStore


def point(t=1.):
    """Create a real stamped payload understood by the existing normalizer."""
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=int((t % 1) * 1e9)),
                        frame_id='world'), point=NS(x=1., y=2., z=3.))


def helpers():
    """Reuse the existing ROS fake without starting a context or changing production."""
    spec = importlib.util.spec_from_file_location(
        'observer_existing_tests', Path(__file__).with_name('test_recording.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_enabled_scientific_envelopes_and_files_are_identical(tmp_path):
    """No scientific schema, raw payload, source stamp or explicit arrival is rewritten."""
    collector = ObserverTiming(tmp_path / 'diagnostics', ['clock', 'target_truth'], monitor=False)
    stores = [TelemetryStore({}, tmp_path / 'off'),
              TelemetryStore({}, tmp_path / 'on', diagnostics=collector)]
    for store in stores:
        store.record('clock', NS(clock=NS(sec=1, nanosec=0)), 2.)
        store.record('target_truth', point(), 2.1)
        assert store.ready(['clock', 'target_truth'])
    assert stores[0].streams == stores[1].streams
    for name in ['clock.jsonl', 'target_truth.jsonl', 'events.jsonl']:
        assert (tmp_path / 'off' / name).read_bytes() == (tmp_path / 'on' / name).read_bytes()
    assert not (tmp_path / 'diagnostics').exists(), 'no diagnostic hot-path file writes'
    for store in stores:
        store.close()
    doc = collector.finalize(complete=True)
    assert doc['schema_version'] == 1 and doc['complete']
    assert doc['callbacks']['streams']['target_truth']['count'] == 1
    assert (tmp_path / 'diagnostics/observer_timing.json').exists()


def test_controlled_stage_durations_and_bounded_event_memory(tmp_path):
    """Known stage boundaries distinguish normalization, serialization, write and other work."""
    collector = ObserverTiming(tmp_path, ['target_truth'], monitor=False, capacity=4)
    for i in range(20):
        collector.callback('target_truth', {
            'entry': float(i), 'normalization_start': i+.01, 'normalization_end': i+.03,
            'serialization_start': i+.04, 'serialization_end': i+.05,
            'write_start': i+.05, 'write_end': i+.25, 'exit': i+.26},
            {'source_time': float(i), 'clock': i-.1}, None, None)
    doc = collector.snapshot()
    stages = doc['callbacks']['streams']['target_truth']['stages_s']
    assert stages['normalization']['mean'] == pytest.approx(.02)
    assert stages['serialization']['mean'] == pytest.approx(.01)
    assert stages['write']['mean'] == pytest.approx(.20)
    assert stages['total']['max'] == pytest.approx(.26)
    assert len(doc['callbacks']['long_operations']) == 4
    assert doc['callbacks']['streams']['target_truth']['count'] == 20
    assert doc['observer_clock']['streams']['target_truth']['ahead_count'] == 20
    assert doc['bounds']['capacity'] == 4


def test_repeated_diagnostics_reuse_summary_accumulators(tmp_path, monkeypatch):
    """Ordinary observations must not allocate and discard fresh histogram accumulators."""
    from robot_experiments.recording import observer_timing
    c = ObserverTiming(tmp_path, ['joints'], monitor=False)
    marks = dict.fromkeys(['entry', 'normalization_start', 'normalization_end',
                           'serialization_start', 'serialization_end', 'write_start',
                           'write_end', 'exit'], 1.)
    c.begin_batch(.02)
    c.executor_call(.02, 0., .001)
    c.callback('joints', marks, {'source_time': 1., 'clock': 1.}, None, None)

    def unexpected_allocation(self):
        raise AssertionError('discarded per-observation histogram allocation')

    monkeypatch.setattr(observer_timing.Summary, '__init__', unexpected_allocation)
    for _ in range(20):
        c.executor_call(.02, 0., .001)
        c.callback('joints', marks, {'source_time': 1., 'clock': 1.}, None, None)
    c.end_batch('no_callback')
    doc = c.snapshot()
    assert doc['callbacks']['streams']['joints']['count'] == 21
    assert doc['callbacks']['streams']['joints']['stages_s']['total']['count'] == 21
    assert doc['executor']['spin_calls_s']['0.02']['count'] == 21


@pytest.mark.parametrize('enabled', [False, True])
def test_partial_batches_do_not_starve_later_ready_streams(tmp_path, monkeypatch, enabled):
    """The installed public iterator must continue across wait/drain and batch boundaries."""
    # Use the installed iterator, replacing only its DDS-ready generator with
    # a deterministic continuously-ready subscription snapshot (R1A control).
    from rclpy.executors import Executor
    names = ['clock', 'target_truth', 'target_measurement', 'joints', 'base_truth',
             'robot_estimate', 'target_estimate', 'prediction', 'control', 'command',
             'actuation']
    h = helpers()
    resources = h.fake_ros(monkeypatch)
    module = h.recording()
    clock = NS(value=0.)
    monkeypatch.setattr(importlib.import_module('robot_experiments.recording.ros'),
                        'monotonic', lambda: clock.value)
    c = ObserverTiming(tmp_path/'diag', names, monitor=False,
                       now=lambda: clock.value) if enabled else None
    recorder = module.Recorder({'topics': {}}, tmp_path/'data', diagnostics=c)
    seen = []

    def ready(**kwargs):
        for name in names:
            yield name

    state = NS(_cb_iter=None, _last_args=None, _last_kwargs=None,
               _wait_for_ready_callbacks=ready)

    def spin(timeout_sec):
        seen.append(Executor.wait_for_ready_callbacks(state, timeout_sec=timeout_sec))
        recorder._callback_count += 1
        clock.value += .004

    monkeypatch.setattr(resources.executors[0], 'spin_once', spin)
    for _ in range(4):
        recorder.spin_once(.02)
    assert set(seen) == set(names), f'permanently neglected ready streams: {set(names)-set(seen)}'
    assert seen[:11] == names
    recorder.close()


def test_metadata_features_and_unavailable_sequences_are_honest(tmp_path):
    """DDS sequences stay distinct from source epochs; unknown metadata is not fabricated."""
    c = ObserverTiming(tmp_path, ['joints'], monitor=False)
    marks = dict.fromkeys(['entry', 'normalization_start', 'normalization_end',
                           'serialization_start', 'serialization_end', 'write_start',
                           'write_end', 'exit'], 1.)
    for sequence in [1, 4, 0]:
        c.callback('joints', marks, {'source_time': sequence * 99., 'clock': 1.}, {
            'publisher_gid': bytes([1]*24), 'publication_sequence_number': sequence,
            'source_timestamp': 123456, 'received_timestamp': 123459}, None)
    c.callback('joints', marks, {'source_time': 1., 'clock': 1.}, None, None)
    m = c.snapshot()['middleware']['streams']['joints']
    assert m['publication_sequence_gaps'] == 2
    assert m['fields']['reception_sequence_number']['available_count'] == 0
    assert m['fields']['publication_sequence_number']['unavailable_count'] == 2
    assert c.snapshot()['middleware']['message_lost_events']['value'] is None


@pytest.mark.parametrize('limit', ['count_budget', 'time_budget'])
def test_instrumented_batches_keep_timeouts_and_existing_limits(tmp_path, monkeypatch, limit):
    """The existing executor still receives one blocking call followed by zero-time calls."""
    h = helpers()
    resources = h.fake_ros(monkeypatch)
    module = h.recording()
    c = ObserverTiming(tmp_path / 'diag', ['target_truth'], monitor=False)
    recorder = module.Recorder({'topics': {'target_truth': '/target'}}, tmp_path / 'data',
                               diagnostics=c)
    # Existing fakes use message-only callbacks: directly enqueue through the real receiver.
    executor = resources.executors[0]

    def spin(timeout_sec):
        executor.timeouts.append(timeout_sec.timeout)
        recorder._receive_timed('target_truth', point(), None)

    monkeypatch.setattr(executor, 'spin_once', spin)
    clock = NS(value=0.)
    original_spin = spin

    def timed_spin(timeout_sec):
        original_spin(timeout_sec)
        if limit == 'time_budget' and len(executor.timeouts) == 2:
            clock.value = .011

    monkeypatch.setattr(executor, 'spin_once', timed_spin)
    c.now = lambda: clock.value
    monkeypatch.setattr(importlib.import_module('robot_experiments.recording.ros'),
                        'monotonic', lambda: clock.value)
    recorder.spin_once(.02)
    expected = 128 if limit == 'count_budget' else 2
    assert executor.timeouts == [.02] + [0.] * (expected-1)
    b = c.snapshot()['executor']['recent_batches'][-1]
    assert b['stop_reason'] == limit and b['callbacks'] == expected
    assert b['per_stream']['target_truth'] == expected
    assert b['order'] == ['target_truth'] * expected
    assert b['count_budget_hit'] is (limit == 'count_budget')
    assert b['time_budget_hit'] is (limit == 'time_budget')
    recorder.close()


def test_scientific_flush_error_wins_over_diagnostic_shutdown_error(tmp_path, monkeypatch):
    """Secondary diagnostic persistence cannot mask a primary scientific flush error."""
    c = ObserverTiming(tmp_path / 'diag', ['clock'], monitor=False)
    store = TelemetryStore({}, tmp_path/'data', diagnostics=c)

    def scientific_error():
        raise OSError('scientific flush failed')

    store._files['clock'] = NS(flush=scientific_error, close=lambda: None)
    monkeypatch.setattr(c, 'close', lambda: (_ for _ in ()).throw(
        RuntimeError('diagnostic flush failed')))
    with pytest.raises(OSError, match='scientific flush failed'):
        store.close()


def test_batch_boundary_flags_do_not_conflate_spin_calls_and_callbacks(tmp_path):
    """A final empty spin or overlong callback has distinct measured boundary flags."""
    clock = NS(value=0.)
    c = ObserverTiming(tmp_path, ['clock'], monitor=False, now=lambda: clock.value)
    c.begin_batch(.02)
    c._batch['executor_calls'] = 128
    c._batch['callbacks'] = 127
    c.start_drain()
    clock.value = .02
    c.end_batch('count_budget')
    b = c.snapshot()['executor']['recent_batches'][-1]
    assert not b['count_budget_hit'] and b['spin_count_budget_hit']
    assert b['time_budget_hit'] and not b['time_budget_checked_stop']


def test_collector_failure_is_visible_and_does_not_mask_writer_exception(tmp_path, monkeypatch):
    """Instrumentation cannot silently succeed; an existing scientific writer error wins."""
    c = ObserverTiming(tmp_path / 'diag', ['target_truth'], monitor=False)
    store = TelemetryStore({}, tmp_path / 'data', diagnostics=c)

    def fail(*args):
        raise RuntimeError('collector broken')

    monkeypatch.setattr(c, 'callback', fail)
    with pytest.raises(RuntimeError, match='collector broken'):
        store.record('target_truth', point())
    store._files['target_truth'] = NS(write=lambda x: (_ for _ in ()).throw(
        OSError('scientific writer broken')), flush=lambda: None, close=lambda: None)
    with pytest.raises(OSError, match='scientific writer broken'):
        store.record('target_truth', point())
    store.close()
    assert not c.finalize(complete=True)['complete']


def test_partial_shutdown_artifact_and_failure_state(tmp_path):
    """Close is bounded and leaves partial diagnostics explicit until runner finalization."""
    c = ObserverTiming(tmp_path, ['clock'], monitor=False)
    c.close()
    doc = json.loads((tmp_path / 'observer_timing.json').read_text())
    assert doc['complete'] is False
    doc = c.finalize(complete=False, error={'code': 'readiness_timeout'})
    assert doc['run_error']['code'] == 'readiness_timeout'


def test_cli_enables_diagnostics_outside_experiment_configuration(tmp_path, monkeypatch, capsys):
    """Only the explicit flag passes the diagnostic option to the existing runner."""
    from robot_experiments import cli
    src = Path(__file__).resolve().parents[2]
    calls = []
    monkeypatch.setattr(cli, 'run_experiment', lambda *args, **kwargs: (
        calls.append(kwargs) or (tmp_path, 0)))
    argv = ['--config', str(src / 'robot_experiments/config/paired_prediction_composition.yaml')]
    assert cli.main(argv, share=lambda p: src/p) == 0
    assert cli.main(argv+['--observer-diagnostics'], share=lambda p: src/p) == 0
    assert calls == [{}, {'observer_diagnostics': True}]
    capsys.readouterr()


def test_real_slow_normalize_serialize_write_are_measured_once(tmp_path, monkeypatch):
    """The real store measures its actual operations without repeating scientific writes."""
    from robot_experiments.recording import storage
    c = ObserverTiming(tmp_path / 'diag', ['target_truth'], monitor=False)
    store = TelemetryStore({}, tmp_path / 'data', diagnostics=c)
    normalize, dumps = storage.normalize, storage.json.dumps
    calls = []

    def slow_normalize(*args):
        time.sleep(.003)
        return normalize(*args)

    def slow_dumps(*args, **kwargs):
        calls.append('serialize')
        time.sleep(.003)
        return dumps(*args, **kwargs)

    def slow_write(text):
        calls.append('write')
        time.sleep(.025)

    store._files['target_truth'] = NS(write=slow_write, flush=lambda: None, close=lambda: None)
    monkeypatch.setattr(storage, 'normalize', slow_normalize)
    monkeypatch.setattr(storage.json, 'dumps', slow_dumps)
    store.record('target_truth', point())
    assert calls == ['serialize', 'write']
    stages = c.snapshot()['callbacks']['streams']['target_truth']['stages_s']
    assert stages['normalization']['min'] >= .003
    assert stages['serialization']['min'] >= .003
    assert stages['write']['min'] >= .025
    assert c.snapshot()['callbacks']['long_operations'][-1]['source_time'] == 1.
    assert not c.directory.exists()
    store.close()


def test_enabled_subscription_requires_message_info_and_keeps_qos(tmp_path, monkeypatch):
    """Rclpy one-argument binding must fail for the diagnostic callback only."""
    import inspect
    h = helpers()
    resources = h.fake_ros(monkeypatch)
    c = ObserverTiming(tmp_path / 'diag', ['clock'], monitor=False)
    recorder = h.recording().Recorder(
        {'topics': {'clock': '/clock'}}, tmp_path / 'data', diagnostics=c)
    _, topic, callback, qos = resources.nodes[0].subscriptions[0]
    assert topic == '/clock' and qos.depth == 100 and qos.reliability == 'best_effort'
    with pytest.raises(TypeError):
        inspect.signature(callback).bind(object())
    inspect.signature(callback).bind(object(), object())
    callback(NS(clock=NS(sec=3, nanosec=0)), {'publisher_gid': bytes([1]*24)})
    assert c.snapshot()['callbacks']['streams']['clock']['count'] == 1
    recorder.close()


@pytest.mark.parametrize('failure', ['readiness_timeout', 'interrupted', 'close_failed'])
@pytest.mark.parametrize('diagnostic_write_fails', [False, True])
def test_runner_preserves_original_failure_and_restores_environment(
        tmp_path, monkeypatch, failure, diagnostic_write_fails):
    """Failed diagnostic finalization cannot replace timeout/interrupt scientific status."""
    from robot_experiments import recording, runner
    from robot_experiments.configuration import load_experiment
    from robot_experiments.execution import RunFailure
    src = Path(__file__).resolve().parents[2]
    spec = load_experiment(src/'robot_experiments/config/nominal.yaml', lambda p: src/p)

    class Recorder:
        def __init__(self, context, directory, *, diagnostics):
            self.streams, self.availability, self.events = {}, {}, []

        def spin_once(self, timeout):
            pass

        def close(self):
            if failure == 'close_failed':
                raise OSError('recorder close failed')

    class Process:
        def __init__(self, command, env, log):
            self.log = log

        def start(self):
            self.log.write_text('diagnostic test startup\n')
            return self

        def stop(self, grace):
            return {'survivors': [], 'returncode': 0}

    def fail(*args, **kwargs):
        if failure == 'close_failed':
            return {'window_start': 1., 'window_end': 2.}
        if failure == 'interrupted':
            raise KeyboardInterrupt()
        raise RunFailure(failure)

    monkeypatch.setattr(recording, 'Recorder', Recorder)
    monkeypatch.setattr(runner, 'OwnedLaunch', Process)
    monkeypatch.setattr(runner, 'run_window', fail)
    init = ObserverTiming.__init__
    monkeypatch.setattr(ObserverTiming, '__init__',
                        lambda self, *a, **k: init(self, *a, monitor=False, **k))
    if diagnostic_write_fails:
        monkeypatch.setattr(ObserverTiming, 'finalize', lambda *a, **k: (_ for _ in ()).throw(
            OSError('diagnostic disk full')))
    import os
    before = {k: os.environ.get(k) for k in
              ['ROS_DOMAIN_ID', 'GZ_PARTITION', 'GZ_IP', 'ROS_LOG_DIR']}
    monkeypatch.setattr(runner, 'component_identities', lambda *a: ({}, []))
    monkeypatch.setattr(runner, 'evaluate', lambda *a: {
        'time_series': {}, 'safety': {}, 'metrics': {}})
    monkeypatch.setattr(runner, '_acceptance', lambda *a: (None, {}))
    directory = tmp_path/'run'
    result = runner.run_one(spec, spec['variants'][0], 42, directory, lambda p: src/p,
                            observer_diagnostics=True)
    assert result['status']['execution_status'] == 'failed'
    assert {k: os.environ.get(k) for k in before} == before
    assert result['status']['error']['code'] == (failure if failure != 'close_failed'
                                                 else 'exception')
    assert result['acceptance'] is None
    if not diagnostic_write_fails:
        doc = json.loads((directory/'diagnostics/observer_timing.json').read_text())
        assert not doc['complete']
        assert doc['run_error']['code'] == (failure if failure != 'close_failed'
                                            else 'exception')
