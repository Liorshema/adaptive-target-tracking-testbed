"""Ownership checks must never signal a recycled or unrelated process."""

import os

from robot_dashboard import launcher


def test_launcher_uses_cache_and_does_not_own_unrelated_process(tmp_path, monkeypatch):
    """A live unrelated PID is never adopted from a forged/stale metadata file."""
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    assert launcher.state_directory(8060).is_relative_to(tmp_path)
    assert not launcher.owned({'pid': os.getpid(), 'start': launcher.process_start(os.getpid())})
    assert not launcher.owned({'pid': -1, 'start': 'invalid'})
    assert not (tmp_path / 'catalog.yaml').exists()


def test_process_start_reads_current_process_and_rejects_missing_pid():
    """Ownership includes process birth, not just a reusable numeric PID."""
    assert launcher.process_start(os.getpid())
    assert launcher.process_start(-1) is None


def test_module_looking_arguments_do_not_authorize_unrelated_python_process():
    """A Python -c process may carry arbitrary module-looking positional arguments."""
    import subprocess
    import sys
    import time

    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(2)',
                              '-m', launcher.MODULE, 'serve'], start_new_session=True)
    try:
        time.sleep(.05)
        record = {'pid': child.pid, 'start': launcher.process_start(child.pid), 'port': 8060}
        assert not launcher.owned(record)
    finally:
        child.terminate()
        child.wait(timeout=5)
