"""Own one optional local Dashboard server; never own experiments or Run workers."""

import argparse
import fcntl
import json
import os
import signal
import socket
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

MODULE = 'robot_dashboard.launcher'


def state_directory(port):
    """Keep process metadata/logs in cache, separate from experiments and Results."""
    return (
        Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache'))
        / 'adaptive_tracking'
        / f'dashboard-{port}'
    )


def process_start(pid):
    """Read Linux process birth ticks, guarding PID recycling."""
    if type(pid) is not int or pid <= 0:
        return None
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def owned(record, expected_port=None):
    """Require birth, user, interpreter and the complete dedicated serve command."""
    pid, port = record.get('pid'), record.get('port')
    if (type(port) is not int or not 1 <= port <= 65535
            or expected_port is not None and port != expected_port
            or process_start(pid) is None or process_start(pid) != record.get('start')):
        return False
    try:
        args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        if args and args[-1] == b'':
            args.pop()
        expected = [os.fsencode(sys.executable), b'-u', b'-m', MODULE.encode(),
                    b'serve', b'--port', str(port).encode()]
        return args == expected and Path(f'/proc/{pid}').stat().st_uid == os.getuid()
    except OSError:
        return False


def health(port):
    """Read the existing web server's lightweight application identity endpoint."""
    try:
        with urlopen(f'http://127.0.0.1:{port}/dashboard-health', timeout=1) as response:
            value = json.load(response)
        return (
            value
            if isinstance(value, dict) and value.get('service') == 'robot_dashboard'
            else None
        )
    except (OSError, URLError, ValueError):
        return None


@contextmanager
def locked(directory):
    """Serialize startup and shutdown across concurrent Windows/WSL clicks."""
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'owner.lock').open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def read_record(directory):
    """Read owned process metadata; malformed state never authorizes a signal."""
    try:
        value = json.loads((directory / 'server.json').read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def terminate(record, expected_port=None):
    """Stop only the identity-checked Dashboard process group."""
    if not owned(record, expected_port):
        return
    pid = record['pid']
    if os.getpgid(pid) != pid:
        raise RuntimeError('Dashboard group identity changed; refusing to signal')
    os.killpg(pid, signal.SIGTERM)
    deadline = time.monotonic() + 5
    while owned(record, expected_port) and time.monotonic() < deadline:
        time.sleep(0.1)
    if owned(record, expected_port):
        os.killpg(pid, signal.SIGKILL)


def manage(action, port, timeout):
    """Start/reuse/stop the UI with bounded readiness and explicit startup errors."""
    directory = state_directory(port)
    with locked(directory):
        record = read_record(directory)
        if action == 'stop':
            terminate(record, port)
            (directory / 'server.json').unlink(missing_ok=True)
            print('Dashboard stopped (owned instance only).')
            return 0
        current = health(port)
        if owned(record, port) and current and current.get('pid') == record['pid']:
            print(f'Dashboard already running: http://localhost:{port}')
            return 0
        if owned(record, port):
            raise RuntimeError(
                'Owned Dashboard is not healthy; use Stop, inspect server.log, then retry'
            )
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError(
                    f'Port {port} is occupied by an unowned server. '
                    'Stop that server before starting this launcher.'
                )
        with (directory / 'server.log').open('ab') as log:
            process = subprocess.Popen(
                [sys.executable, '-u', '-m', MODULE, 'serve', '--port', str(port)],
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        record = {'pid': process.pid, 'start': process_start(process.pid), 'port': port}
        temporary = directory / 'server.json.tmp'
        temporary.write_text(json.dumps(record))
        temporary.replace(directory / 'server.json')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and process.poll() is None:
            current = health(port)
            if current and current.get('pid') == process.pid:
                print(f'Dashboard ready: http://localhost:{port}\nLog: {directory / "server.log"}')
                return 0
            time.sleep(0.2)
        terminate(record, port)
        process.wait(timeout=5)
        (directory / 'server.json').unlink(missing_ok=True)
        raise RuntimeError(f'Dashboard startup failed. Inspect {directory / "server.log"}')


def main(argv=None):
    """WSL lifecycle entry point; default Catalog stays with the existing CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'serve'])
    parser.add_argument('--port', type=int, default=8060)
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 0 < args.timeout <= 300:
        parser.error('port must be 1..65535 and startup timeout in (0,300] seconds')
    if args.action == 'serve':
        from .cli import main as dashboard

        return dashboard(['--port', str(args.port)])
    try:
        return manage(args.action, args.port, args.timeout)
    except (OSError, RuntimeError) as error:
        print(f'Dashboard error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
