"""
Bounded experiment workers independent of clients, with persistent provenance.

Workers delegate all ROS/Gazebo ownership to the existing M08 runner. Stopping
requests cooperative cancellation, preserving its normal finally/cleanup path.
"""
from contextlib import contextmanager
import fcntl
import signal
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

    @contextmanager
    def _preview_lock(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        with (self.directory/'simulation.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try: yield
            finally: fcntl.flock(lock, fcntl.LOCK_UN)

    def preview_status(self, identifier=None):
        pointer=self.directory/'simulation.json'
        if identifier is None:
            if not pointer.exists():
                return {'state':'closed','job_id':None,'kind':'simulation',
                        'cleanup_complete':True,'error':None}
            identifier=json.loads(pointer.read_text())['job_id']
        request=json.loads((self._path(identifier)/'request.json').read_text())
        if request.get('kind')!='simulation':
            raise ValueError('Job is not an interactive simulation')
        return self.status(identifier)

    def _preview_blocks(self):
        status=self.preview_status()
        return bool(status['job_id'] and (
            status['state'] in {'opening','paused','running','closing','worker_lost'}
            or not status.get('cleanup_complete',False)))

    def start(self, catalog, identifier, revision=None, *, output=None, runtime_overrides=None,
              preview_id=None, session_id=None):
        # The preview shares this existing owner, not a competing process engine.
        with self._preview_lock():
            # Preflight before closing anything: an invalid Play must preserve preview.
            spec=catalog.prepare(identifier, revision, output=output,
                                 runtime_overrides=runtime_overrides)
            if spec['runtime'].get('gui'):
                from .simulation import display_capabilities
                display=display_capabilities()
                if not display['available']: raise ValueError(display['reason'])
            extra={}
            if session_id is not None:
                if not isinstance(session_id, str) or str(uuid.UUID(session_id)) != session_id:
                    raise ValueError('session ID must be a canonical UUID')
                extra['session_id'] = session_id
            if preview_id is not None:
                current=self.preview_status()
                if (current.get('job_id') != preview_id or current['state'] != 'paused'
                        or current.get('experiment_identity') != spec['experiment_identity']):
                    raise ValueError('Play requires the exact paused saved revision; Close before changing selection')
                if not spec['runtime'].get('gui'):
                    raise ValueError('Preview handoff requires a visible scored GUI')
                closed=self.stop(preview_id)
                if closed['state'] != 'closed' or closed.get('cleanup_complete') is not True:
                    raise ValueError('Preview cleanup unresolved; no scored Run was started')
                extra['handoff_from']=preview_id
            if self._preview_blocks():
                raise ValueError('Close the interactive simulation and verify cleanup before Run')
            return self._create(catalog,spec,runtime_overrides or {},**extra)

    def start_simulation(self,catalog,identifier,revision,*,variant,seed=None):
        from .simulation import display_capabilities
        display=display_capabilities()
        if not display['available']: raise ValueError(display['reason'])
        with self._preview_lock():
            current=self.preview_status()
            if self._preview_blocks():
                if current['state'] in {'opening','paused','running'} and (
                    current.get('experiment_identity',{}).get('id')==identifier
                    and current['experiment_identity']['revision']==revision
                    and current.get('variant')==variant
                    and (seed is None or current.get('seed')==seed)):
                    return current
                raise ValueError('Close the existing simulation and resolve cleanup before restart')
            for path in self.directory.glob('*/request.json'):
                state=self.status(path.parent.name)
                if state['state'] in {'queued','running','worker_lost'}:
                    raise ValueError('Close/cancel the active Job before opening a preview')
            runtime={'enable_actuation':False,'gui':True}
            spec=catalog.prepare(identifier,revision,output=self.directory/'preview',
                                 runtime_overrides=runtime)
            if variant not in {v['name'] for v in spec['variants']}:
                raise ValueError('Select a saved variant for the simulation')
            seed=spec['seeds'][0] if seed is None else seed
            if type(seed) is not int or seed not in spec['seeds']:
                raise ValueError('Select a seed from the saved revision')
            owner=psutil.Process()
            return self._create(catalog,spec,runtime,kind='simulation',variant=variant,seed=seed,
                                owner={'pid':owner.pid,'created':owner.create_time()})

    def _spawn_worker(self,directory):
        with (directory/'worker.log').open('x') as log:
            process=subprocess.Popen(
                [sys.executable,'-m','robot_experiments.jobs','--worker',str(directory)],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            write_json(directory/'process.json',{'pid':process.pid,
                       'created':psutil.Process(process.pid).create_time()})
        except Exception as error:
            raise RuntimeError('Worker spawned but identity recording failed; cleanup unknown') from error

    def _create(self,catalog,spec,runtime,**extra):
        job_id=str(uuid.uuid4());directory=self._path(job_id);directory.mkdir(parents=True)
        request={'job_id':job_id,'record':spec['experiment_definition'],
                 'catalog':str(catalog.path),'output':str(spec['output']['directory']),
                 'runtime_overrides':runtime,**extra}
        write_json(directory/'request.json',request)
        preview=extra.get('kind')=='simulation'
        status={'job_id':job_id,'state':'opening' if preview else 'queued',
                'experiment_identity':spec['experiment_identity'],
                'created_utc':datetime.now(timezone.utc).isoformat()}
        if 'handoff_from' in extra:
            status['handoff_from']=extra['handoff_from']
        if preview:
            status.update(kind='simulation',variant=extra['variant'],seed=extra['seed'],
                          enable_actuation=False,gui=True,cleanup_complete=False)
            write_json(self.directory/'simulation.json',{'job_id':job_id})
        write_json(directory/'status.json',status)
        try: self._spawn_worker(directory)
        except Exception as error:
            status.update(state='error' if preview else 'failed',error=str(error),
                          cleanup_complete=isinstance(error,OSError))
            write_json(directory/'status.json',status)
            raise
        return self.status(job_id)

    def status(self, identifier):
        directory = self._path(identifier)
        status = json.loads((directory/'status.json').read_text())
        status['artifacts'] = str(directory)
        status['stop_requested'] = (directory/'stop.request').exists()
        if status['state'] in {'queued', 'opening', 'paused', 'running', 'closing'} and (directory/'process.json').exists():
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
        if status['state'] not in {'queued', 'opening', 'paused', 'running', 'closing'}:
            return status
        # A cooperative request is idempotent. It cannot asynchronously interrupt
        # native launch cleanup, even when clients retry or stop concurrently.
        (directory/'stop.request').touch(exist_ok=True)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            status = self.status(identifier)
            if status['state'] not in {'queued', 'opening', 'paused', 'running', 'closing'}:
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
    preview=request.get('kind')=='simulation'
    if preview:
        def request_close(signum,frame):
            (directory/'stop.request').touch(exist_ok=True)
        signal.signal(signal.SIGTERM,request_close)
        signal.signal(signal.SIGINT,request_close)
    status.update(state='opening' if preview else 'running',
                  started_utc=datetime.now(timezone.utc).isoformat())
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
        if preview:
            from .simulation import run_preview
            return run_preview(spec,request,directory,catalog.share,status,
                               lambda:(directory/'stop.request').exists())
        from .runner import run_experiment

        def study_created(study):
            status['study'] = str(study)
            write_json(directory/'status.json', status)

        study, code = run_experiment(
            spec, catalog.share, on_study_created=study_created,
            cancel_requested=lambda: (directory/'stop.request').exists())
        status.update(study=str(study), state='cancelled' if (directory/'stop.request').exists()
                      else 'completed' if code in (0, 3) else 'failed', exit_code=code)
    except KeyboardInterrupt:
        status.update(state='closed' if preview else 'cancelled', error='interrupted',
                      cleanup_complete=True,exit_code=1)
    except Exception as error:
        status.update(state='error' if preview else 'failed',
                      error=f'{type(error).__name__}: {error}',
                      cleanup_complete=(status.get('cleanup_complete',False)
                                        or 'launch_owner' not in status),exit_code=1)
    finally:
        status['ended_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(directory/'status.json', status)
    return code


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', type=Path, required=True)
    raise SystemExit(worker(parser.parse_args().worker))
