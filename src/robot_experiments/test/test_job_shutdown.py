"""Deployment shutdown uses cooperative Jobs cancellation, never invented results."""

import json
import os
import subprocess
import sys
import threading
import time
import uuid

import psutil

from robot_experiments.jobs import ExperimentJobs
from robot_experiments.reporting import write_json


def active_job(jobs, state='running', *, alive=True):
    identifier = str(uuid.uuid4())
    directory = jobs.directory / identifier
    directory.mkdir(parents=True)
    write_json(directory / 'request.json', {'job_id': identifier})
    write_json(directory / 'status.json', {'job_id': identifier, 'state': state})
    write_json(directory / 'process.json', {
        'pid': os.getpid(),
        'created': psutil.Process().create_time() if alive else 0.0,
    })
    return identifier, directory


def test_shutdown_requests_all_jobs_before_waiting_and_keeps_terminal_evidence(tmp_path):
    jobs = ExperimentJobs(tmp_path / 'jobs')
    first, a = active_job(jobs)
    second, b = active_job(jobs, 'queued')
    completed, c = active_job(jobs, 'completed')
    original = (c / 'status.json').read_bytes()

    def finish():
        deadline = time.monotonic() + 3
        while not all((p / 'stop.request').exists() for p in (a, b)):
            if time.monotonic() > deadline:
                return
            time.sleep(.01)
        for identifier, path in [(first, a), (second, b)]:
            write_json(path / 'status.json', {'job_id': identifier, 'state': 'cancelled'})

    thread = threading.Thread(target=finish)
    thread.start()
    try:
        report = jobs.stop_all(timeout=2)
    finally:
        thread.join(timeout=4)
    assert not report['errors']
    assert {j['job_id'] for j in report['jobs']} == {first, second}
    assert all(j['state'] == 'cancelled' for j in report['jobs'])
    assert not (c / 'stop.request').exists()
    assert (c / 'status.json').read_bytes() == original


def test_shutdown_timeout_and_lost_worker_do_not_claim_completion(tmp_path):
    jobs = ExperimentJobs(tmp_path / 'jobs')
    active, a = active_job(jobs)
    lost, b = active_job(jobs, alive=False)
    report = jobs.stop_all(timeout=0)
    assert (a / 'stop.request').exists()
    assert not (b / 'stop.request').exists()
    assert report['errors']
    assert {j['job_id']: j['state'] for j in report['jobs']} == {
        active: 'running', lost: 'worker_lost'}
    assert json.loads((a / 'status.json').read_text())['state'] == 'running'
    assert json.loads((b / 'status.json').read_text())['state'] == 'running'


def test_shutdown_reports_corrupt_job_instead_of_overwriting_it(tmp_path):
    jobs = ExperimentJobs(tmp_path / 'jobs')
    identifier, path = active_job(jobs)
    (path / 'status.json').write_text('{')
    report = jobs.stop_all(timeout=0)
    assert report['errors'][0]['job_id'] == identifier
    assert (path / 'status.json').read_text() == '{'


def test_worker_lost_during_shutdown_is_reported_as_unverified(tmp_path):
    """An actual child exit after the stop request cannot produce a clean receipt."""
    jobs = ExperimentJobs(tmp_path / 'jobs')
    identifier, path = active_job(jobs)
    code = ('import os, pathlib, sys, time\n'
            'request = pathlib.Path(sys.argv[1])\n'
            'while not request.exists(): time.sleep(.005)\n'
            'os._exit(7)\n')
    child = subprocess.Popen([sys.executable, '-c', code, str(path / 'stop.request')])
    try:
        write_json(path / 'process.json', {
            'pid': child.pid, 'created': psutil.Process(child.pid).create_time()})
        report = jobs.stop_all(timeout=2)
        assert report['jobs'][0]['state'] == 'worker_lost'
        assert any(e['job_id'] == identifier for e in report['errors'])
        assert json.loads((path / 'status.json').read_text())['state'] == 'running'
    finally:
        if child.poll() is None:
            child.kill()
        child.wait(timeout=5)
