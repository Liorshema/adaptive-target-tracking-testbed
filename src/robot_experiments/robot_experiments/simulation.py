"""Unscored interactive preview using existing materialization and launch ownership.

X11 inspection is lazy and only verifies mapped windows belonging to the owned
process tree. It is not a renderer, process manager or scientific observer.
"""

from contextlib import contextmanager
import ctypes as C
import os
import time
import threading

import psutil

from .configuration import materialize_run
from .execution import DomainLease, OwnedLaunch, RunFailure
from .reporting import write_json
from .recording.ros import observe_clock

_DISPLAY_LOCK = threading.RLock()
_XERROR = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p)


class _Attributes(C.Structure):
    _fields_ = [
        (name, kind)
        for name, kind in [
            ('x', C.c_int),
            ('y', C.c_int),
            ('width', C.c_int),
            ('height', C.c_int),
            ('border_width', C.c_int),
            ('depth', C.c_int),
            ('visual', C.c_void_p),
            ('root', C.c_ulong),
            ('class_', C.c_int),
            ('bit_gravity', C.c_int),
            ('win_gravity', C.c_int),
            ('backing_store', C.c_int),
            ('backing_planes', C.c_ulong),
            ('backing_pixel', C.c_ulong),
            ('save_under', C.c_int),
            ('colormap', C.c_ulong),
            ('map_installed', C.c_int),
            ('map_state', C.c_int),
            ('all_event_masks', C.c_long),
            ('your_event_mask', C.c_long),
            ('do_not_propagate_mask', C.c_long),
            ('override_redirect', C.c_int),
            ('screen', C.c_void_p),
        ]
    ]


@contextmanager
def _display():
    if not os.environ.get('DISPLAY'):
        raise ValueError(
            'X11 DISPLAY is required; native Wayland-only window verification is unsupported'
        )
    # Serialize opening as well as inspection: Dash may query from multiple threads.
    with _DISPLAY_LOCK:
        lib = C.CDLL('libX11.so.6')
        lib.XOpenDisplay.argtypes = [C.c_char_p]
        lib.XOpenDisplay.restype = C.c_void_p
        lib.XCloseDisplay.argtypes = [C.c_void_p]
        display = lib.XOpenDisplay(None)
        if not display:
            raise ValueError('X11 display cannot be opened; check DISPLAY and display access')
        # Windows can disappear between reads; contain Xlib's fatal BadWindow handler.
        lib.XSetErrorHandler.argtypes = [C.c_void_p]
        lib.XSetErrorHandler.restype = C.c_void_p
        lib.XSync.argtypes = [C.c_void_p, C.c_int]
        ignore = _XERROR(lambda connection, event: 0)
        previous = lib.XSetErrorHandler(C.cast(ignore, C.c_void_p))
        try:
            yield lib, display
        finally:
            lib.XSync(display, False)
            lib.XCloseDisplay(display)
            lib.XSetErrorHandler(previous)


def display_capabilities():
    """Check a real X11 connection without importing ROS or optional dashboard code."""
    try:
        with _display():
            return {
                'available': True,
                'reason': 'Owned X11 GUI; actuation OFF; Close before changing configuration.',
            }
    except (ValueError, OSError) as error:
        return {'available': False, 'reason': str(error)}


def owned_windows(pids):
    """Return mapped nontrivial X11 windows whose recorded PID is owned by this launch."""
    windows = []
    with _display() as (x, display):
        W = C.c_ulong
        WP = C.POINTER(W)
        x.XDefaultRootWindow.argtypes = [C.c_void_p]
        x.XDefaultRootWindow.restype = W
        x.XQueryTree.argtypes = [C.c_void_p, W, WP, WP, C.POINTER(WP), C.POINTER(C.c_uint)]
        x.XGetWindowAttributes.argtypes = [C.c_void_p, W, C.POINTER(_Attributes)]
        x.XInternAtom.argtypes = [C.c_void_p, C.c_char_p, C.c_int]
        x.XInternAtom.restype = W
        x.XGetWindowProperty.argtypes = [
            C.c_void_p,
            W,
            W,
            C.c_long,
            C.c_long,
            C.c_int,
            W,
            WP,
            C.POINTER(C.c_int),
            WP,
            WP,
            C.POINTER(C.POINTER(C.c_ubyte)),
        ]
        x.XFetchName.argtypes = [C.c_void_p, W, C.POINTER(C.c_void_p)]
        x.XFree.argtypes = [C.c_void_p]
        atom = x.XInternAtom(display, b'_NET_WM_PID', False)
        queue = [(x.XDefaultRootWindow(display), 0)]
        visited = 0
        while queue and visited < 4096:
            window, depth = queue.pop()
            visited += 1
            a = _Attributes()
            if not x.XGetWindowAttributes(display, window, C.byref(a)):
                continue
            if a.map_state == 2 and a.width >= 100 and a.height >= 100:
                actual = W()
                fmt = C.c_int()
                items = W()
                remaining = W()
                data = C.POINTER(C.c_ubyte)()
                code = x.XGetWindowProperty(
                    display,
                    window,
                    atom,
                    0,
                    1,
                    False,
                    0,
                    C.byref(actual),
                    C.byref(fmt),
                    C.byref(items),
                    C.byref(remaining),
                    C.byref(data),
                )
                try:
                    pid = (
                        C.cast(data, C.POINTER(W))[0]
                        if code == 0 and fmt.value == 32 and items.value
                        else None
                    )
                    if pid in pids:
                        text = C.c_void_p()
                        title = ''
                        if x.XFetchName(display, window, C.byref(text)) and text.value:
                            title = C.string_at(text.value).decode(errors='replace')
                            x.XFree(text)
                        windows.append(
                            {
                                'window_id': int(window),
                                'pid': int(pid),
                                'title': title,
                                'x': a.x,
                                'y': a.y,
                                'width': a.width,
                                'height': a.height,
                                'mapped': True,
                            }
                        )
                finally:
                    if data:
                        x.XFree(data)
            if depth < 3:
                root = W()
                parent = W()
                children = WP()
                count = C.c_uint()
                if x.XQueryTree(
                    display,
                    window,
                    C.byref(root),
                    C.byref(parent),
                    C.byref(children),
                    C.byref(count),
                ):
                    queue.extend((children[i], depth + 1) for i in range(min(count.value, 4096)))
                    if children:
                        x.XFree(children)
    return windows



def owner_alive(owner):
    try:
        p = psutil.Process(owner['pid'])
        return p.create_time() == owner['created'] and p.status() != psutil.STATUS_ZOMBIE
    except psutil.Error:
        return False


def run_preview(spec, request, directory, share, status, stop_requested):
    """Own one frozen, unscored GUI composition until Close, owner loss or failure."""
    process = None
    cleanup = None
    error = None
    reason = None

    def publish(**values):
        status.update(values)
        write_json(directory / 'status.json', status)

    try:
        capability = display_capabilities()
        if not capability['available']:
            raise ValueError(capability['reason'])
        variant = next(v for v in spec['variants'] if v['name'] == request['variant'])
        prepared = materialize_run(
            variant, request['seed'], directory, share, spec['runtime'], start_running=False
        )
        write_json(
            directory / 'resolved_configuration.json',
            {
                'experiment_identity': spec['experiment_identity'],
                'variant': request['variant'],
                'seed': request['seed'],
                'system': prepared['plan'],
                'owner_snapshot': prepared['snapshot'],
                'unscored': True,
                'enable_actuation': False,
                'gui': True,
            },
        )
        if stop_requested() or not owner_alive(request['owner']):
            reason = 'close_requested' if stop_requested() else 'dashboard_owner_exited'
            return 0
        with DomainLease(log_dir=directory / 'ros-log') as lease:
            lease.env['QT_QPA_PLATFORM'] = 'xcb'
            publish(
                isolation={'ros_domain_id': lease.domain_id, 'gazebo_partition': lease.partition},
                gui_environment={k: lease.env.get(k) for k in ('DISPLAY', 'QT_QPA_PLATFORM')},
            )
            command = [
                'ros2',
                'launch',
                'robot_bringup',
                'closed_loop.launch.py',
                f'system_config:={prepared["system_path"]}',
            ]
            publish(launch_command=command)
            old = {k: os.environ.get(k) for k in ('ROS_DOMAIN_ID', 'ROS_LOG_DIR')}
            try:
                os.environ.update({k: lease.env[k] for k in old})
                with observe_clock(prepared['context']['topics']['clock']) as (clock, spin):
                    process = OwnedLaunch(command, lease.env, directory / 'launch.log')
                    process.start()
                    publish(
                        launch_owner={
                            'pid': process.process.pid,
                            'created': psutil.Process(process.process.pid).create_time(),
                        }
                    )
                    started = time.monotonic()
                    last_scan = 0.0
                    last_publish = 0.0
                    windows = []
                    paused_clock = None
                    while True:
                        if stop_requested() or not owner_alive(request['owner']):
                            reason = (
                                'close_requested' if stop_requested() else 'dashboard_owner_exited'
                            )
                            publish(state='closing', close_reason=reason)
                            break
                        if process.poll() is not None:
                            raise RunFailure('simulation_exited', {'returncode': process.poll()})
                        spin()
                        if clock[0] is not None:
                            if paused_clock is None:
                                paused_clock = clock[0]
                            elif clock[0] > paused_clock:
                                raise RunFailure('preview_advanced', {
                                    'message': 'Use dashboard Play to start a scored experiment',
                                    'initial_clock': paused_clock, 'clock': clock[0]})
                        now = time.monotonic()
                        if now - last_scan >= 0.5:
                            windows = owned_windows({p['pid'] for p in process.identities()})
                            last_scan = now
                        if windows:
                            if now - last_publish >= 1.0 or status['state'] != 'paused':
                                publish(
                                    state='paused',
                                    windows=windows,
                                    clock=clock[0],
                                    owned_processes=process.identities(),
                                    readiness='owned mapped GUI; configured paused startup',
                                )
                                last_publish = now
                        elif (
                            status['state'] == 'opening'
                            and now - started >= spec['execution']['readiness_timeout']
                        ):
                            raise RunFailure(
                                'gui_readiness_timeout', {'clock': clock[0], 'windows': windows}
                            )
            finally:
                for key, value in old.items():
                    if value is None:
                        os.environ.pop(key, None)
                    else:
                        os.environ[key] = value
                if process is not None:
                    try:
                        publish(state='closing')
                    finally:
                        # Cleanup cannot depend on being able to write a status file.
                        cleanup = process.stop(spec['execution']['shutdown_timeout'])
    except Exception as failure:
        error = {'code': getattr(failure, 'code', type(failure).__name__), 'message': str(failure)}
    finally:
        complete = process is None or (cleanup is not None and not cleanup['survivors'])
        if not complete:
            error = {
                'code': 'cleanup_incomplete',
                'message': 'Inspect owned survivors before restarting',
            }
        publish(
            state='error' if error else 'closed',
            error=error,
            cleanup=cleanup,
            cleanup_complete=complete,
            close_reason=reason,
        )
    return 1 if error else 0
