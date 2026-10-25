"""Private simulator sessions and bounded simulation-time execution windows."""

from __future__ import annotations

import fcntl
import math
import os
import secrets
import signal
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path

import psutil


class RunFailure(RuntimeError):
    """An interrupted run with a stable machine-readable failure code."""

    def __init__(self, code: str, details: dict | None = None):
        """Store a failure code and serializable run details."""
        self.code = code
        self.details = details or {}
        super().__init__(f'{code}: {self.details}')


class DomainLease:
    """Hold an advisory exclusive ROS domain lease for the complete run."""

    def __init__(self, domain_id: int | None = None, *,
                 log_dir: Path | None = None, base_env: dict | None = None):
        """Prepare a requested or randomly selected domain without acquiring it."""
        if domain_id is not None and not 20 <= domain_id <= 220:
            raise ValueError('domain_id must be between 20 and 220')
        self.domain_id = domain_id
        self.partition = f'robot-experiments-{uuid.uuid4().hex}'
        self.log_dir = Path(log_dir) if log_dir is not None else (
            Path(tempfile.gettempdir()) / self.partition / 'ros')
        self.env = dict(os.environ if base_env is None else base_env)
        self._lock = None

    def __enter__(self):
        """Acquire the lease and return its private launch environment."""
        candidates = ([self.domain_id] if self.domain_id is not None else
                      secrets.SystemRandom().sample(range(20, 221), 201))
        for candidate in candidates:
            lock = open(f'/tmp/robot_experiments-domain-{candidate}.lock', 'a+')
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lock.close()
                continue
            self._lock = lock
            self.domain_id = candidate
            try:
                self.log_dir.mkdir(parents=True, exist_ok=True)
                self.env.update(ROS_DOMAIN_ID=str(candidate), GZ_PARTITION=self.partition,
                                GZ_IP='127.0.0.1', ROS_LOG_DIR=str(self.log_dir))
            except BaseException:
                self.__exit__(None, None, None)
                raise
            return self
        raise RunFailure('domain_unavailable', {'requested_domain': self.domain_id})

    def __exit__(self, exc_type, exc, traceback):
        """Release the domain even when the run raises an exception."""
        if self._lock is not None:
            fcntl.flock(self._lock.fileno(), fcntl.LOCK_UN)
            self._lock.close()
            self._lock = None
        # Keep the inode: unlinking permits concurrent leases on different inodes.


class OwnedLaunch:
    """Launch in a new session and signal only that session or known children.

    Session membership and a private inherited environment marker find orphans,
    including children which immediately create a new session. Recorded process
    identities protect unrelated processes after PID reuse. Ownership excludes
    children that deliberately clear the marker and escape the POSIX session
    before being observed; normal bringup descendants inherit the marker.
    """

    def __init__(self, command: list[str], env: dict, log: Path):
        """Copy the command and environment for one owned launch."""
        self.command = list(command)
        self.env = dict(env)
        self._owner_marker = uuid.uuid4().hex
        self.env['_ROBOT_EXPERIMENTS_OWNER'] = self._owner_marker
        self.log = Path(log)
        self.process = None
        self._session = None
        self._known = {}
        self._mutex = threading.Lock()
        self._finished = threading.Event()
        self._monitor = None
        self._output = None
        self._orphan_scan_done = False

    def start(self):
        """Start a private process session and retain descendant identities."""
        if self.process is not None:
            raise RuntimeError('launch has already started')
        self.log.parent.mkdir(parents=True, exist_ok=True)
        self._output = self.log.open('ab', buffering=0)
        try:
            self.process = subprocess.Popen(
                self.command, env=self.env, stdout=self._output,
                stderr=subprocess.STDOUT, start_new_session=True)
            self._session = self.process.pid
            try:
                root = psutil.Process(self.process.pid)
                self._known[root.pid] = root.create_time()
            except psutil.NoSuchProcess:
                pass
            self._snapshot()
            self._monitor = threading.Thread(target=self._watch, daemon=True)
            self._monitor.start()
        except BaseException:
            self._output.close()
            raise
        return self

    def _watch(self):
        while not self._finished.wait(.2):
            exited = self.process.poll() is not None
            scan_orphans = exited and not self._orphan_scan_done
            self._snapshot(scan_marker=scan_orphans, scan_session=scan_orphans)
            if scan_orphans:
                self._orphan_scan_done = True

    def _snapshot(self, *, scan_marker=False, scan_session=False):
        if self.process is None:
            return []
        with self._mutex:
            # Check identities before traversing: a reused launcher PID cannot
            # confer ownership of the replacement process or its children.
            owned = []
            for pid, created in self._known.items():
                try:
                    candidate = psutil.Process(pid)
                    if candidate.create_time() == created:
                        owned.append(candidate)
                except psutil.Error:
                    continue
            discovered = list(owned)
            session_anchor = False
            traversed = set()
            for candidate in owned:
                try:
                    if scan_session:
                        session_anchor |= os.getsid(candidate.pid) == self._session
                    if candidate.pid not in traversed:
                        descendants = candidate.children(recursive=True)
                        discovered.extend(descendants)
                        traversed.update(child.pid for child in descendants)
                except (psutil.Error, ProcessLookupError, PermissionError):
                    continue
            # An existing owned member (including an unreaped parent) proves
            # this numeric session still belongs to us before scanning orphans.
            # Environment scans are reserved for parent exit and shutdown,
            # rather than reading process tables on normal watcher ticks.
            if session_anchor or scan_marker:
                for candidate in psutil.process_iter(['pid']):
                    try:
                        in_session = (session_anchor
                                      and os.getsid(candidate.pid) == self._session)
                        if in_session or (scan_marker and candidate.environ().get(
                                '_ROBOT_EXPERIMENTS_OWNER') == self._owner_marker):
                            discovered.append(candidate)
                    except (psutil.Error, ProcessLookupError, PermissionError):
                        continue
            for candidate in discovered:
                try:
                    self._known[candidate.pid] = candidate.create_time()
                except psutil.Error:
                    continue
            alive = []
            for pid, created in list(self._known.items()):
                try:
                    candidate = psutil.Process(pid)
                    if (candidate.create_time() == created
                            and candidate.status() != psutil.STATUS_ZOMBIE):
                        alive.append(candidate)
                except psutil.Error:
                    continue
            return alive

    def poll(self):
        """Return the launcher exit code without process-table traversal."""
        return None if self.process is None else self.process.poll()

    def stop(self, grace: float = 5.0) -> dict:
        """Escalate signals within one bounded grace period; report survivors."""
        if not math.isfinite(grace) or grace < 0:
            raise ValueError('grace must be finite and nonnegative')
        started = time.monotonic()
        signals = []
        phases = []
        if self.process is None:
            return {'returncode': None, 'survivors': [], 'signals': [], 'elapsed': 0.0}
        # Each phase has a share of a single bounded shutdown budget.
        for signum, fraction in ((signal.SIGINT, .5), (signal.SIGTERM, .4),
                                 (signal.SIGKILL, .1)):
            alive = self._snapshot(scan_marker=True, scan_session=True)
            if not alive:
                break
            signals.append(signum.name)
            # Launch owns graceful child ordering. Only bypass it if the owner
            # has already exited or its bounded graceful phase did not finish.
            recipients = alive
            if signum == signal.SIGINT and self.process.poll() is None:
                recipients = [p for p in alive if p.pid == self.process.pid]
            phases.append({'signal': signum.name, 'recipients': [
                {'pid': p.pid, 'created': self._known[p.pid]} for p in recipients]})
            for candidate in recipients:
                try:
                    candidate.send_signal(signum)
                except psutil.Error:
                    pass
            deadline = time.monotonic() + grace * fraction
            while time.monotonic() < deadline:
                if not self._snapshot():
                    break
                time.sleep(min(.01, max(0, deadline - time.monotonic())))
        self._finished.set()
        if self._monitor is not None:
            self._monitor.join(timeout=.1)
        survivors = [candidate.pid
                     for candidate in self._snapshot(scan_marker=True, scan_session=True)]
        returncode = self.process.poll()
        if self._output is not None:
            self._output.close()
        return {'returncode': returncode, 'survivors': survivors, 'signals': signals,
                'phases': phases, 'elapsed': time.monotonic() - started}


def run_window(process, recorder, *, required_streams: list, duration: float,
               settling_time: float, readiness_timeout: float, stall_timeout: float,
               wall_timeout: float, tick=None) -> dict:
    """Wait for readiness, settle, and measure a complete simulation-time window.

    Recorder clocks are seconds. Monotonic deadlines are independent of simulator
    speed; a backward clock invalidates the run rather than changing its epoch.
    """
    for name, value in (('duration', duration), ('settling_time', settling_time),
                        ('readiness_timeout', readiness_timeout),
                        ('stall_timeout', stall_timeout), ('wall_timeout', wall_timeout)):
        if not math.isfinite(value) or value < 0 or (name != 'settling_time' and value == 0):
            raise ValueError(f'{name} must be finite and positive (settling may be zero)')
    started = time.monotonic()
    last_progress = started
    previous_clock = None
    origin = None
    while True:
        returncode = process.poll()
        if returncode is not None:
            raise RunFailure('process_exited', {'returncode': returncode})
        recorder.spin_once(.02)
        now = time.monotonic()
        clock = recorder.clock
        details = {'clock': clock, 'monotonic_elapsed': now - started, 'origin': origin}
        if recorder.clock_reset or (clock is not None and previous_clock is not None
                                    and clock < previous_clock):
            raise RunFailure('clock_reset', details)
        if clock is not None and (not math.isfinite(clock) or clock < 0):
            raise RunFailure('clock_reset', details)
        if clock is not None and (previous_clock is None or clock > previous_clock):
            last_progress = now
        previous_clock = clock
        if tick is not None:
            tick()
        returncode = process.poll()
        now = time.monotonic()
        details['monotonic_elapsed'] = now - started
        if returncode is not None:
            raise RunFailure('process_exited', {**details, 'returncode': returncode})
        if now - started >= wall_timeout:
            raise RunFailure('wall_timeout', details)
        if origin is None:
            if clock is not None and clock > 0 and recorder.ready(required_streams):
                origin = clock
            elif now - started >= readiness_timeout:
                raise RunFailure('readiness_timeout', details)
        if clock is not None and clock > 0 and now - last_progress >= stall_timeout:
            raise RunFailure('clock_stalled', details)
        if origin is not None and clock >= origin + settling_time + duration:
            return {'origin': origin, 'window_start': origin + settling_time,
                    'window_end': origin + settling_time + duration, 'stop_clock': clock,
                    'monotonic_elapsed': now - started}
