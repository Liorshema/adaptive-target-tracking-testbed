"""The existing CLI can opt into container-owned cooperative worker shutdown."""

import json
import signal
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('owned, errors, exit_code', [
    (False, [], 0), (True, [], 0), (True, [{'error': 'worker_lost'}], 1)])
def test_dashboard_shutdown_is_opt_in_and_restores_signal_handler(
        tmp_path, monkeypatch, owned, errors, exit_code):
    from robot_dashboard import cli
    from robot_dashboard import app
    from robot_experiments.jobs import ExperimentJobs

    previous = signal.getsignal(signal.SIGTERM)
    stopped = []

    def stop_all(self, **kwargs):
        stopped.append(self.directory)
        return {'jobs': [], 'errors': errors}

    monkeypatch.setattr(ExperimentJobs, 'stop_all', stop_all)

    def serve(**kwargs):
        assert kwargs['host'] == '0.0.0.0' and kwargs['port'] == 8050
        if owned:
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)

    fake = SimpleNamespace(server=SimpleNamespace(get=lambda path: lambda f: f), run=serve)
    monkeypatch.setattr(app, 'create_app', lambda backend: fake)
    arguments = ['--host', '0.0.0.0', '--data-dir', str(tmp_path)]
    if owned:
        arguments += ['--stop-jobs-on-exit']
    assert cli.main(arguments) == exit_code
    assert len(stopped) == int(owned)
    assert signal.getsignal(signal.SIGTERM) == previous
    reports = list((tmp_path / 'shutdown').glob('*.json'))
    assert len(reports) == int(owned)
    if owned:
        assert json.loads(reports[0].read_text())['errors'] == errors
