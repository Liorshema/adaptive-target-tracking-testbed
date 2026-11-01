"""Preview lifecycle contracts reuse real Catalog materialization and Jobs ownership."""

import json
import os
from pathlib import Path

import psutil
import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.jobs import ExperimentJobs

SRC = Path(__file__).resolve().parents[2]


def share(name):
    return SRC / name


@pytest.fixture
def api(tmp_path, monkeypatch):
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', share)
    catalog.initialize()
    from robot_experiments import simulation

    monkeypatch.setattr(
        simulation,
        'display_capabilities',
        lambda: {'available': True, 'reason': 'controlled X11 transport'},
    )

    def spawn(self, directory):
        process = psutil.Process()
        (directory / 'process.json').write_text(
            json.dumps({'pid': process.pid, 'created': process.create_time()})
        )

    monkeypatch.setattr(ExperimentJobs, '_spawn_worker', spawn)
    return ExperimentBackend(tmp_path / 'state', catalog=catalog)


def test_gui_is_validated_frozen_and_default_headless(tmp_path):
    catalog = ExperimentCatalog(share=share)
    default = catalog.prepare('EXP-NOMINAL', 3)
    assert default['runtime']['gui'] is False
    gui = catalog.prepare(
        'EXP-NOMINAL', 3, runtime_overrides={'enable_actuation': False, 'gui': True}
    )
    from robot_experiments.configuration import materialize_run

    frozen = materialize_run(gui['variants'][0], gui['seeds'][0], tmp_path, share, gui['runtime'])
    assert frozen['plan']['runtime']['gui'] is True
    assert frozen['plan']['runtime']['enable_actuation'] is False
    assert frozen['plan']['runtime']['publish_commands'] is False
    assert (
        catalog.get('EXP-NOMINAL', 3)['definition']['experiment']['runtime'].get('gui', False)
        is False
    )
    with pytest.raises(ValueError, match='gui'):
        catalog.prepare('EXP-NOMINAL', 3, runtime_overrides={'gui': 'true'})


def test_preview_singleton_frozen_identity_and_controlled_restart(api):
    first = api.open_simulation('EXP-NOMINAL', 3, variant='imm')
    second = api.open_simulation('EXP-NOMINAL', 3, variant='imm')
    assert first['job_id'] == second['job_id']
    assert first['kind'] == 'simulation' and first['state'] == 'opening'
    requests = list((api.directory / 'jobs').glob('*/request.json'))
    assert len(requests) == 1
    request = json.loads(requests[0].read_text())
    assert request['runtime_overrides'] == {'enable_actuation': False, 'gui': True}
    assert request['owner']['pid'] == os.getpid()
    with pytest.raises(ValueError, match='Close'):
        api.open_simulation('EXP-NOMINAL', 2, variant='imm')
    with pytest.raises(ValueError, match='Close'):
        api.start_run('EXP-NOMINAL', 3, output=api.directory / 'runs')
    # A recorded terminal state is reopenable only after proven cleanup.
    path = requests[0].with_name('status.json')
    state = json.loads(path.read_text())
    state.update(state='error', cleanup_complete=False)
    path.write_text(json.dumps(state))
    with pytest.raises(ValueError, match='cleanup'):
        api.open_simulation('EXP-NOMINAL', 3, variant='imm')
    state.update(state='closed', cleanup_complete=True)
    path.write_text(json.dumps(state))
    assert api.open_simulation('EXP-NOMINAL', 3, variant='imm')['job_id'] != first['job_id']


def test_preview_bad_inputs_do_not_acquire_processes(api, monkeypatch):
    from robot_experiments import simulation

    for revision, variant in [(None, 'imm'), (3, 'absent')]:
        with pytest.raises(ValueError):
            api.open_simulation('EXP-NOMINAL', revision, variant=variant)
    monkeypatch.setattr(
        simulation,
        'display_capabilities',
        lambda: {'available': False, 'reason': 'X11 display unavailable'},
    )
    with pytest.raises(ValueError, match='X11'):
        api.open_simulation('EXP-NOMINAL', 3, variant='imm')
    assert not list((api.directory / 'jobs').glob('*/request.json'))


def test_display_detection_does_not_claim_native_wayland_support(monkeypatch):
    from robot_experiments.simulation import display_capabilities

    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.setenv('WAYLAND_DISPLAY', 'wayland-0')
    result = display_capabilities()
    assert not result['available'] and 'X11' in result['reason']


@pytest.mark.parametrize(
    'failure', [None, 'readiness', 'spin', 'survivors', 'start', 'persistence', 'advanced']
)
def test_preview_loop_reports_actual_readiness_and_always_cleans(tmp_path, monkeypatch, failure):
    from contextlib import contextmanager
    from robot_experiments import simulation

    spec = ExperimentCatalog(share=share).prepare(
        'EXP-NOMINAL', 3, runtime_overrides={'enable_actuation': False, 'gui': True}
    )
    process = psutil.Process()
    ticks = [0]
    stops = []

    class Launch:
        def __init__(self, command, env, log):
            assert env['QT_QPA_PLATFORM'] == 'xcb'
            assert 'closed_loop.launch.py' in command
            self.process = process

        def start(self):
            if failure == 'start':
                raise RuntimeError('post-spawn failure')
            return self

        def poll(self):
            return None

        def identities(self):
            return [{'pid': process.pid, 'created': process.create_time()}]

        def stop(self, grace):
            stops.append(grace)
            return {'survivors': [999999] if failure == 'survivors' else [], 'returncode': 0}

    @contextmanager
    def clock(topic):
        value = [1.0]

        def spin():
            ticks[0] += 1
            if failure == 'spin':
                raise RuntimeError('observer transport failed')
            if failure == 'advanced':
                value[0] += 0.1

        yield value, spin

    monkeypatch.setattr(simulation, 'display_capabilities', lambda: {'available': True})
    original_write = simulation.write_json
    if failure == 'persistence':

        def write(path, value):
            if path.name == 'status.json' and value.get('state') == 'closing':
                raise OSError('status persistence unavailable')
            original_write(path, value)

        monkeypatch.setattr(simulation, 'write_json', write)
    monkeypatch.setattr(simulation, 'OwnedLaunch', Launch)
    monkeypatch.setattr(simulation, 'observe_clock', clock)
    monkeypatch.setattr(
        simulation,
        'owned_windows',
        lambda pids: (
            []
            if failure == 'readiness'
            else [{'pid': process.pid, 'window_id': 1, 'mapped': True}]
        ),
    )
    if failure == 'readiness':
        times = iter([0.0, 46.0])
        monkeypatch.setattr(simulation.time, 'monotonic', lambda: next(times))
    request = {
        'variant': 'imm',
        'seed': 42,
        'owner': {'pid': process.pid, 'created': process.create_time()},
    }
    status = {'state': 'opening'}
    code = simulation.run_preview(spec, request, tmp_path, share, status, lambda: ticks[0] >= 3)
    assert stops == [spec['execution']['shutdown_timeout']]
    assert not (tmp_path / 'metrics.json').exists() and 'run_id' not in status
    if failure:
        assert code == 1 and status['state'] == 'error'
        assert status['cleanup_complete'] is (failure != 'survivors')
    else:
        assert code == 0 and status['state'] == 'closed' and status['cleanup_complete']
        assert status['readiness'] == 'owned mapped GUI; configured paused startup'
    frozen = json.loads((tmp_path / 'resolved_configuration.json').read_text())
    assert frozen['unscored'] and frozen['enable_actuation'] is False
    import yaml

    assert (
        yaml.safe_load((tmp_path / 'inputs/simulation.yaml').read_text())['gazebo'][
            'start_running'
        ]
        is False
    )


def test_explicit_headless_start_overrides_saved_gui(api):
    from copy import deepcopy

    definition = deepcopy(api.catalog.get('EXP-NOMINAL', 3)['definition'])
    definition['experiment']['runtime']['gui'] = True
    api.catalog.create('EXP-GUI-BASE', 'GUI base', definition, {})
    job = api.start_run('EXP-GUI-BASE', 1, output=api.directory / 'runs')
    request = json.loads((api.directory / 'jobs' / job['job_id'] / 'request.json').read_text())
    assert request['runtime_overrides'] == {'enable_actuation': False, 'gui': False}
    assert api.catalog.get('EXP-GUI-BASE', 1)['definition']['experiment']['runtime']['gui'] is True


def test_x11_open_and_close_share_the_display_lock(monkeypatch):
    from types import SimpleNamespace
    from robot_experiments import simulation

    calls = []

    class Function:
        def __init__(self, name, result=None):
            self.name = name
            self.result = result

        def __call__(self, *args):
            assert simulation._DISPLAY_LOCK._is_owned(), self.name
            calls.append(self.name)
            return self.result

    library = SimpleNamespace(
        **{
            name: Function(name, result)
            for name, result in [
                ('XOpenDisplay', 123),
                ('XCloseDisplay', 0),
                ('XSetErrorHandler', None),
                ('XSync', 0),
            ]
        }
    )
    monkeypatch.setenv('DISPLAY', ':controlled')
    monkeypatch.setattr(simulation.C, 'CDLL', lambda name: library)
    with simulation._display():
        pass
    assert calls[0] == 'XOpenDisplay' and 'XCloseDisplay' in calls


def test_paused_materialization_preserves_normal_start_policy(tmp_path):
    import yaml
    from robot_experiments.configuration import materialize_run

    spec = ExperimentCatalog(share=share).prepare('EXP-NOMINAL', 3)
    normal = tmp_path / 'normal'
    paused = tmp_path / 'paused'
    normal.mkdir()
    paused.mkdir()
    materialize_run(spec['variants'][0], 42, normal, share, spec['runtime'])
    frozen = materialize_run(
        spec['variants'][0], 42, paused, share, spec['runtime'], start_running=False
    )
    assert yaml.safe_load((normal / 'inputs/simulation.yaml').read_text())['gazebo'][
        'start_running'
    ]
    assert not yaml.safe_load((paused / 'inputs/simulation.yaml').read_text())['gazebo'][
        'start_running'
    ]
    assert frozen['simulation_path'] == paused.resolve() / 'inputs/simulation.yaml'


def paused_preview(api):
    first = api.open_simulation('EXP-NOMINAL', 3, variant='imm')
    path = api.directory / 'jobs' / first['job_id'] / 'status.json'
    state = json.loads(path.read_text())
    state.update(state='paused')
    path.write_text(json.dumps(state))
    return first['job_id'], path


def test_handoff_closes_exact_paused_owner_before_start_default_off(api, monkeypatch):
    identifier, path = paused_preview(api)
    stopped = []

    def stop(owner, **options):
        assert owner == identifier
        assert len(list((api.directory / 'jobs').glob('*/request.json'))) == 1
        state = json.loads(path.read_text())
        state.update(state='closed', cleanup_complete=True)
        path.write_text(json.dumps(state))
        stopped.append(owner)
        return state

    monkeypatch.setattr(api.jobs, 'stop', stop)
    job = api.start_run(
        'EXP-NOMINAL', 3, output=api.directory / 'runs', gui=True, preview_id=identifier
    )
    assert stopped == [identifier] and job['job_id'] != identifier
    request = json.loads((api.directory / 'jobs' / job['job_id'] / 'request.json').read_text())
    assert request['handoff_from'] == identifier
    assert request['runtime_overrides'] == {'enable_actuation': False, 'gui': True}


@pytest.mark.parametrize('failure', ['identity', 'cleanup', 'validation', 'gui'])
def test_handoff_refuses_unsafe_or_invalid_transition(api, monkeypatch, failure):
    identifier, path = paused_preview(api)
    calls = []

    def stop(owner, **options):
        calls.append(owner)
        return {'state': 'error', 'cleanup_complete': False}

    monkeypatch.setattr(api.jobs, 'stop', stop)
    with pytest.raises((ValueError, TimeoutError)):
        api.start_run(
            'EXP-NOMINAL',
            2 if failure == 'identity' else 3,
            output=api.directory / 'runs',
            gui=failure != 'gui',
            enable_actuation='invalid' if failure == 'validation' else False,
            preview_id=identifier,
        )
    assert len(list((api.directory / 'jobs').glob('*/request.json'))) == 1
    assert calls == ([identifier] if failure == 'cleanup' else [])


def test_paused_owner_is_active_for_duplicate_open_close_and_worker_loss(api):
    identifier, path = paused_preview(api)
    assert api.open_simulation('EXP-NOMINAL', 3, variant='imm')['job_id'] == identifier
    with pytest.raises(ValueError, match='Close'):
        api.start_run('EXP-NOMINAL', 3, output=api.directory / 'runs')
    process = path.with_name('process.json')
    process.write_text(json.dumps({'pid': 99999999, 'created': 0}))
    assert api.get_simulation_status()['state'] == 'worker_lost'
