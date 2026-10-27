"""
Bounded experiment workers independent of clients, with persistent provenance.

Workers delegate all ROS/Gazebo ownership to the existing M08 runner. Stopping
requests cooperative cancellation, preserving its normal finally/cleanup path.
"""
from datetime import datetime
from datetime import timezone
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

import psutil

from .catalog import definition_hash
from .catalog import ExperimentCatalog
from .reporting import write_json


class ExperimentJobs:
    """A file-backed headless execution interface; no GUI event loop owns ROS."""

    def __init__(self, directory):
        self.directory = Path(directory).resolve()

    def _path(self, identifier):
        if not isinstance(identifier, str) or str(uuid.UUID(identifier)) != identifier:
            raise ValueError('job ID must be a canonical UUID')
        return self.directory/identifier

    def start(self, catalog, identifier, revision=None, *, output=None, runtime_overrides=None):
        # Validate before reserving a job or acquiring any simulator process.
        spec = catalog.prepare(
            identifier, revision, output=output, runtime_overrides=runtime_overrides)
        job_id = str(uuid.uuid4())
        directory = self._path(job_id)
        directory.mkdir(parents=True)
        request = {'job_id': job_id, 'record': spec['experiment_definition'],
                   'catalog': str(catalog.path), 'output': str(spec['output']['directory']),
                   'runtime_overrides': runtime_overrides or {}}
        write_json(directory/'request.json', request)
        write_json(directory/'status.json', {
            'job_id': job_id, 'state': 'queued',
            'experiment_identity': spec['experiment_identity'],
            'created_utc': datetime.now(timezone.utc).isoformat()})
        try:
            with (directory/'worker.log').open('x') as log:
                process = subprocess.Popen(
                    [sys.executable, '-m', 'robot_experiments.jobs', '--worker', str(directory)],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            write_json(directory/'process.json', {'pid': process.pid,
                       'created': psutil.Process(process.pid).create_time()})
        except Exception as error:
            write_json(directory/'status.json', {
                'job_id': job_id, 'state': 'failed', 'error': str(error)})
            raise
        return self.status(job_id)

    def status(self, identifier):
        directory = self._path(identifier)
        status = json.loads((directory/'status.json').read_text())
        status['artifacts'] = str(directory)
        status['stop_requested'] = (directory/'stop.request').exists()
        if status['state'] in {'queued', 'running'} and (directory/'process.json').exists():
            owner = json.loads((directory/'process.json').read_text())
            if not self._alive(owner):
                status.update(
                    state='worker_lost',
                    error='worker exited without final status; inspect artifacts')
        return status

    @staticmethod
    def _alive(owner):
        try:
            process = psutil.Process(owner['pid'])
            return (abs(process.create_time()-owner['created']) < .001
                    and process.status() != psutil.STATUS_ZOMBIE)
        except psutil.NoSuchProcess:
            return False

    def stop(self, identifier, *, timeout=25.):
        directory = self._path(identifier)
        status = self.status(identifier)
        if status['state'] not in {'queued', 'running'}:
            return status
        # A cooperative request is idempotent. It cannot asynchronously interrupt
        # native launch cleanup, even when clients retry or stop concurrently.
        (directory/'stop.request').touch(exist_ok=True)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            status = self.status(identifier)
            if status['state'] not in {'queued', 'running'}:
                return status
            time.sleep(.05)
        raise TimeoutError(
            'worker did not finish cleanup; inspect job artifacts '
            '(no unsafe force-kill performed)')


def history(directory, identifier=None):
    """Expose recorded identities/status/artifact references without recomputing metrics."""
    results = []
    for path in sorted(Path(directory).rglob('experiment_identity.json')):
        identity = json.loads(path.read_text())
        if identifier is not None and identity['id'] != identifier:
            continue
        study = path.parent
        runs = []
        for status in sorted((study/'runs').glob('*/status.json')):
            value = json.loads(status.read_text())
            runs.append({'run_id': value.get('run_id'), 'status': value,
                         'metrics': str(status.with_name('metrics.json')),
                         'telemetry': str(status.parent/'telemetry')})
        results.append({'experiment_identity': identity, 'study': str(study), 'runs': runs,
                        'summary': str(study/'summary.json')})
    return results


def worker(directory):
    directory = Path(directory).resolve()
    request = json.loads((directory/'request.json').read_text())
    if directory.name != request['job_id']:
        raise ValueError('job directory/identity mismatch')
    record = request['record']
    if record['sha256'] != definition_hash(record['definition'], record['overrides']):
        raise ValueError('job definition fingerprint mismatch')
    status = json.loads((directory/'status.json').read_text())
    status.update(state='running', started_utc=datetime.now(timezone.utc).isoformat())
    write_json(directory/'status.json', status)
    code = 1
    try:
        if (directory/'stop.request').exists():
            raise KeyboardInterrupt()
        catalog = ExperimentCatalog(request['catalog'])
        # Use the accepted immutable revision, even if the catalog was later edited.
        spec = catalog._prepare(record, request['output'], request['runtime_overrides'])
        if (directory/'stop.request').exists():
            raise KeyboardInterrupt()
        from .runner import run_experiment
        study, code = run_experiment(spec, catalog.share,
                                     cancel_requested=lambda: (directory/'stop.request').exists())
        status.update(study=str(study), state='cancelled' if (directory/'stop.request').exists()
                      else 'completed' if code in (0, 3) else 'failed', exit_code=code)
    except KeyboardInterrupt:
        status.update(state='cancelled', error='interrupted', exit_code=1)
    except Exception as error:
        status.update(state='failed', error=f'{type(error).__name__}: {error}', exit_code=1)
    finally:
        status['ended_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(directory/'status.json', status)
    return code


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', type=Path, required=True)
    raise SystemExit(worker(parser.parse_args().worker))
