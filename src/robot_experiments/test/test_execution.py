"""Isolation, ownership and simulation-window lifecycle contracts."""

import os
import signal
import subprocess
import sys
import time

import psutil
import pytest

from robot_experiments import execution
from robot_experiments.execution import DomainLease, OwnedLaunch, RunFailure, run_window


class ClockRecorder:
    def __init__(self, values, ready_after=1, reset_after=None):
        self.values = iter(values)
        self.clock = None
        self.clock_reset = False
        self.ready_after = ready_after
        self.reset_after = reset_after
        self.calls = 0
        self.monotonic = 0.0

    def spin_once(self, timeout):
        self.monotonic += timeout
        self.calls += 1
        self.clock = next(self.values, self.clock)
        self.clock_reset = self.calls == self.reset_after

    def ready(self, streams):
        assert streams == ['truth']
        return self.calls >= self.ready_after


class Running:
    def poll(self):
        return None


def window(monkeypatch, recorder, process=None, **kwargs):
    monkeypatch.setattr(execution.time, 'monotonic', lambda: recorder.monotonic)
    options = dict(required_streams=['truth'], duration=2.0, settling_time=1.0,
                   readiness_timeout=1.0, stall_timeout=1.0, wall_timeout=10.0)
    options.update(kwargs)
    return run_window(process or Running(), recorder, **options)


def test_window_measures_duration_after_settling_and_readiness(monkeypatch):
    recorder = ClockRecorder([0, 1, 2, 3, 4, 5, 6], ready_after=3)
    ticks = []
    result = window(monkeypatch, recorder, tick=lambda: ticks.append(recorder.clock))
    assert result['origin'] == 2
    assert result['window_start'] == 3
    assert result['window_end'] == 5
    assert result['stop_clock'] == 5
    assert result['monotonic_elapsed'] == pytest.approx(.12)
    assert ticks


@pytest.mark.parametrize('values,ready_after,reset_after,kwargs,code', [
    ([0] * 100, 1, None, {'readiness_timeout': .1}, 'readiness_timeout'),
    ([1] * 100, 100, None, {'readiness_timeout': .1}, 'readiness_timeout'),
    ([1] * 100, 1, None, {'stall_timeout': .1}, 'clock_stalled'),
    ([1, 2, 3], 1, 3, {}, 'clock_reset'),
    ([1, 2, 1], 1, None, {}, 'clock_reset'),
    ([i * .001 for i in range(100)], 1, None, {'wall_timeout': .1}, 'wall_timeout'),
])
def test_window_deadlines_and_clock_failures(monkeypatch, values, ready_after,
                                           reset_after, kwargs, code):
    recorder = ClockRecorder(values, ready_after, reset_after)
    with pytest.raises(RunFailure) as failure:
        window(monkeypatch, recorder, **kwargs)
    assert failure.value.code == code
    assert isinstance(failure.value.details, dict)


def test_window_rejects_process_exit_before_readiness(monkeypatch):
    class Exited:
        def poll(self):
            return 3
    with pytest.raises(RunFailure) as failure:
        window(monkeypatch, ClockRecorder([0]), Exited())
    assert failure.value.code == 'process_exited'
    assert failure.value.details['returncode'] == 3


def test_domain_lease_is_exclusive_and_builds_private_environment(tmp_path):
    original = dict(os.environ)
    with DomainLease(log_dir=tmp_path / 'ros') as first:
        assert 20 <= first.domain_id <= 220
        assert first.env['ROS_DOMAIN_ID'] == str(first.domain_id)
        assert first.env['GZ_PARTITION'] == first.partition
        assert first.env['GZ_IP'] == '127.0.0.1'
        assert first.env['ROS_LOG_DIR'] == str(tmp_path / 'ros')
        with pytest.raises(RunFailure) as failure:
            with DomainLease(first.domain_id, log_dir=tmp_path):
                pass
        assert failure.value.code == 'domain_unavailable'
        with DomainLease(log_dir=tmp_path) as second:
            assert first.domain_id != second.domain_id
            assert first.partition != second.partition
    assert dict(os.environ) == original
    with DomainLease(first.domain_id, log_dir=tmp_path) as reused:
        assert reused.domain_id == first.domain_id


def alive(pid):
    try:
        return psutil.Process(pid).status() != psutil.STATUS_ZOMBIE
    except psutil.NoSuchProcess:
        return False


def wait_file(path, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if path.exists():
            value = path.read_text()
            if value:
                return value
        time.sleep(.01)
    pytest.fail(f'child did not create {path}')


def test_owned_launch_kills_orphan_descendants_and_preserves_unrelated(tmp_path):
    child_pid_file = tmp_path / 'child.pid'
    child_code = ('import os,signal,time; '
                  'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                  'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                  f'open({str(child_pid_file)!r},"w").write(str(os.getpid())); '
                  'time.sleep(60)')
    parent_code = f'import subprocess; subprocess.Popen([{sys.executable!r}, "-c", {child_code!r}])'
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    launch = OwnedLaunch([sys.executable, '-c', parent_code], dict(os.environ), tmp_path / 'launch.log')
    child_pid = None
    try:
        launch.start()
        child_pid = int(wait_file(child_pid_file))
        end = time.monotonic() + 3
        while launch.poll() is None and time.monotonic() < end:
            time.sleep(.01)
        started = time.monotonic()
        outcome = launch.stop(grace=.3)
        assert time.monotonic() - started < 1.5
        assert not alive(child_pid)
        assert unrelated.poll() is None
        assert outcome['survivors'] == []
        assert launch.stop(grace=.1)['survivors'] == []
    finally:
        launch.stop(grace=.1)
        unrelated.terminate()
        unrelated.wait(timeout=3)
        if child_pid and alive(child_pid):
            os.kill(child_pid, signal.SIGKILL)


def test_owned_launch_uses_supplied_env_and_captures_output(tmp_path):
    log = tmp_path / 'launch.log'
    launch = OwnedLaunch([sys.executable, '-c',
                          'import os; print(os.environ["PRIVATE_RUN"] , flush=True)'],
                         {**os.environ, 'PRIVATE_RUN': 'isolated'}, log)
    launch.start()
    end = time.monotonic() + 3
    while launch.poll() is None and time.monotonic() < end:
        time.sleep(.01)
    result = launch.stop(grace=.1)
    assert launch.poll() == 0
    assert log.read_text().strip() == 'isolated'
    assert result['survivors'] == []


def test_window_detects_clock_stall_while_waiting_for_streams(monkeypatch):
    recorder = ClockRecorder([1] * 100, ready_after=100)
    with pytest.raises(RunFailure) as failure:
        window(monkeypatch, recorder, stall_timeout=.1, readiness_timeout=1)
    assert failure.value.code == 'clock_stalled'


def test_known_descendant_new_session_is_owned_after_launcher_exits(tmp_path):
    pid_file = tmp_path / 'escaped.pid'
    child_code = ('import os,time; '
                  f'open({str(pid_file)!r},"w").write(str(os.getpid())); time.sleep(60)')
    parent_code = ('import subprocess,time; '
                   f'subprocess.Popen([{sys.executable!r},"-c",{child_code!r}], '
                   'start_new_session=True); time.sleep(.15)')
    launch = OwnedLaunch([sys.executable, '-c', parent_code], dict(os.environ), tmp_path / 'owned.log')
    child_pid = None
    try:
        launch.start()
        child_pid = int(wait_file(pid_file))
        end = time.monotonic() + 3
        while launch.poll() is None and time.monotonic() < end:
            time.sleep(.01)
        assert launch.stop(grace=.3)['survivors'] == []
        assert not alive(child_pid)
    finally:
        launch.stop(grace=.1)
        if child_pid and alive(child_pid):
            os.kill(child_pid, signal.SIGKILL)


def test_window_counts_sampling_time_against_wall_deadline(monkeypatch):
    recorder = ClockRecorder([1, 2])

    def slow_tick():
        recorder.monotonic += 1

    with pytest.raises(RunFailure) as failure:
        window(monkeypatch, recorder, wall_timeout=.5, tick=slow_tick)
    assert failure.value.code == 'wall_timeout'
    assert recorder.calls == 1


@pytest.mark.parametrize('trial', range(5))
def test_immediate_new_session_orphan_remains_owned(tmp_path, trial):
    pid_file = tmp_path / f'immediate-{trial}.pid'
    child_code = ('import os,time,signal; '
                  'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                  'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                  f'open({str(pid_file)!r},"w").write(str(os.getpid())); '
                  'time.sleep(60)')
    parent_code = ('import subprocess; '
                   f'subprocess.Popen([{sys.executable!r},"-c",{child_code!r}], '
                   'start_new_session=True)')
    base_env = dict(os.environ)
    launch = OwnedLaunch([sys.executable, '-c', parent_code], base_env,
                         tmp_path / 'immediate.log')
    unrelated = subprocess.Popen([sys.executable, '-c',
                                  'import time; time.sleep(60)'], env=base_env)
    child_pid = None
    try:
        launch.start()
        child_pid = int(wait_file(pid_file))
        end = time.monotonic() + 3
        while launch.poll() is None and time.monotonic() < end:
            time.sleep(.01)
        result = launch.stop(grace=.3)
        assert not alive(child_pid)
        assert result['survivors'] == []
        assert unrelated.poll() is None
        assert dict(os.environ) == base_env
    finally:
        launch.stop(grace=.1)
        unrelated.terminate()
        unrelated.wait(timeout=3)
        if child_pid and alive(child_pid):
            os.kill(child_pid, signal.SIGKILL)


def test_poll_does_not_scan_process_tree(monkeypatch, tmp_path):
    launch = OwnedLaunch(['unused'], {}, tmp_path / 'unused.log')
    launch.process = Running()

    def forbidden_snapshot(**kwargs):
        pytest.fail('poll must not scan the process table or descendants')

    monkeypatch.setattr(launch, '_snapshot', forbidden_snapshot)
    started = time.monotonic()
    for _ in range(100):
        assert launch.poll() is None
    assert time.monotonic() - started < .1


def test_stop_allows_launch_to_order_child_shutdown(tmp_path):
    """Runner SIGINT must reach the launch owner before its managed children."""
    ready = tmp_path/'ready'
    signal_file = tmp_path/'child-int'
    child = ('import signal,time,pathlib; '
             f'signal.signal(signal.SIGINT,lambda *a:pathlib.Path({str(signal_file)!r}).write_text("unexpected")); '
             f'pathlib.Path({str(ready)!r}).write_text("ready"); time.sleep(60)')
    parent = ('import signal,subprocess,time,sys; '
              f'p=subprocess.Popen([sys.executable,"-c",{child!r}]); '
              'signal.signal(signal.SIGINT,lambda *a:(time.sleep(.1),p.terminate(),p.wait(),sys.exit(0))); time.sleep(60)')
    launch = OwnedLaunch([sys.executable, '-c', parent], dict(os.environ), tmp_path/'ordered.log').start()
    try:
        wait_file(ready)
        outcome = launch.stop(grace=2.)
        assert not signal_file.exists()
        assert outcome['returncode'] == 0
        assert outcome['survivors'] == []
    finally:
        launch.stop(grace=.2)
