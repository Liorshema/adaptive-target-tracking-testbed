"""Read-only semantic scene with bounded histories and source-time provenance."""

from collections import deque
from numbers import Real

import numpy as np

from robot_bringup.visualization.configuration import validate_configuration
from robot_bringup.visualization.geometry import finite_array, radius_shell, rigid_transform

from robot_models.whole_body.kinematics import WholeBodyKinematics

from scipy.spatial.transform import Rotation


HISTORIES = ('target_truth', 'target_estimate', 'robot_truth', 'robot_estimate')


def _time(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value):
        raise ValueError('Time must be a finite number')
    return float(value)


class Scene:
    """Consume already transformed observations; produce complete replacement snapshots.

    Only physical robot truth and measured joints can produce camera FK. The injected
    model already incorporates its authoritative fixed tracking-frame mount.
    """

    def __init__(self, config, model, joint_names, joint_limits, forward_axis, radius):
        """Inject physical geometry, joint limits and tracking parameters."""
        self.config = validate_configuration(config)
        self.model = model
        self.joint_names = tuple(joint_names)
        if (not self.joint_names or len(set(self.joint_names)) != len(self.joint_names) or
                any(not isinstance(name, str) or not name for name in self.joint_names) or
                len(self.joint_names) != model.manipulator.dof):
            raise ValueError('Joint names must match model DOF and be unique')
        self.forward_axis = finite_array(forward_axis, (3,), 'forward_axis')
        norm = np.linalg.norm(self.forward_axis)
        if norm <= np.finfo(float).eps:
            raise ValueError('forward_axis must be nonzero')
        self.forward_axis /= norm
        # The radius is injected from the control contract, never a visual default.
        radius_shell([0., 0., 0.], radius, samples=4)
        self.radius = float(radius)
        self.limits = None
        if joint_limits is not None:
            read = (joint_limits.__getitem__ if isinstance(joint_limits, dict)
                    else lambda key: getattr(joint_limits, key))
            self.limits = {key: finite_array(read(key), (len(self.joint_names),), key)
                           for key in ('lower', 'upper', 'velocity')}
            if (np.any(self.limits['lower'] >= self.limits['upper']) or
                    np.any(self.limits['velocity'] <= 0)):
                raise ValueError('Joint position and velocity limits must be ordered and positive')
        self.reset()

    def reset(self):
        """Forget observations, forecasts, and simulation-clock epoch."""
        self._latest = {}
        self._histories = {key: deque(maxlen=self.config['max_points']) for key in HISTORIES}
        self._breaks = set()
        self._last_now = None

    def _clock(self, now):
        now = _time(now)
        if self._last_now is not None and now < self._last_now:
            self.reset()
        self._last_now = now
        return now

    def receive(self, key, stamp, frame, data, now):
        """Accept a valid source sample without allowing it to overwrite newer data."""
        try:
            now = self._clock(now)
            stamp = _time(stamp)
            if (key not in (*HISTORIES, 'prediction', 'joints') or
                    stamp > now + self.config['future_tolerance']):
                return False
            if (not isinstance(frame, str) or
                    (key != 'joints' and frame != self.config['frame_id'])):
                self._break_history(key, stamp)
                return False
            previous = self._latest.get(key)
            if previous is not None and stamp < previous['stamp']:
                return False
            payload = self._validate_data(key, data, stamp)
            self._latest[key] = {'stamp': stamp, **payload}
            if key in HISTORIES:
                history = self._histories[key]
                point = {'stamp': stamp, 'position': payload['position'].tolist()}
                gap_or_jump = history and (
                    stamp - history[-1]['stamp'] > self.config['history_max_gap'] or
                    np.linalg.norm(payload['position'] - history[-1]['position']) >
                    self.config['history_max_step'])
                if key in self._breaks or gap_or_jump:
                    history.clear()
                self._breaks.discard(key)
                if history and history[-1]['stamp'] == stamp:
                    history[-1] = point
                else:
                    history.append(point)
                self._prune(history, now)
            return True
        except (ValueError, TypeError, KeyError, IndexError, OverflowError):
            if isinstance(key, str) and isinstance(stamp, Real) and np.isfinite(stamp):
                self._break_history(key, stamp)
            return False

    def _break_history(self, key, stamp):
        """Break the next accepted trail after invalid data, without replacing valid state."""
        if key in HISTORIES and stamp >= self._latest.get(key, {}).get('stamp', -np.inf):
            self._breaks.add(key)

    def invalidate(self, key, stamp):
        """Let adapters break trails for samples rejected before semantic conversion."""
        try:
            stamp = _time(stamp)
            self._break_history(key, stamp)
        except (ValueError, TypeError):
            return

    def _validate_data(self, key, data, stamp):
        if not isinstance(data, dict):
            raise ValueError('Observation payload must be a mapping')
        if key == 'prediction':
            if set(data) != {'states', 'timestamps'}:
                raise ValueError('Forecast requires states and timestamps')
            times = np.asarray(data['timestamps'])
            if times.ndim != 1 or not len(times):
                raise ValueError('Forecast timestamps must be a nonempty vector')
            times = finite_array(times, times.shape, 'timestamps')
            states = finite_array(data['states'], (len(times), 6), 'states')
            if np.any(np.diff(times) <= 0) or np.any(times <= stamp):
                raise ValueError('Forecast timestamps must increase strictly after source epoch')
            count = self.config['max_points']
            return {'states': states[:count], 'timestamps': times[:count]}
        if key == 'joints':
            if set(data) != {'names', 'position', 'velocity'}:
                raise ValueError('Joints require names, position and velocity')
            names = data['names']
            if (not isinstance(names, (list, tuple)) or
                    any(not isinstance(name, str) for name in names) or
                    len(set(names)) != len(names) or not set(self.joint_names).issubset(names)):
                raise ValueError('Joint observation must include each configured joint once')
            position = finite_array(data['position'], (len(names),), 'position')
            velocity = finite_array(data['velocity'], (len(names),), 'velocity')
            order = [names.index(name) for name in self.joint_names]
            return {'position': position[order], 'velocity': velocity[order]}
        fields = {'position', 'orientation'} if key.startswith('robot_') else {'position'}
        if set(data) != fields:
            raise ValueError('Invalid pose/point payload fields')
        result = {'position': finite_array(data['position'], (3,), 'position')}
        if 'orientation' in fields:
            quaternion = finite_array(data['orientation'], (4,), 'orientation')
            norm = np.linalg.norm(quaternion)
            if not np.isclose(norm, 1., atol=1e-3, rtol=0.):
                raise ValueError('Orientation must be a unit quaternion in xyzw order')
            result['orientation'] = quaternion / norm
        return result

    def _prune(self, history, now):
        oldest = now - self.config['history_seconds']
        while history and history[0]['stamp'] < oldest:
            history.popleft()

    def _fresh(self, key, now):
        sample = self._latest.get(key)
        age = self.config['prediction_max_age'] if key == 'prediction' else self.config['max_age']
        if sample is None or now - sample['stamp'] > age:
            return None
        return sample

    def _camera(self, now):
        base, joints = self._fresh('robot_truth', now), self._fresh('joints', now)
        if (base is None or joints is None or
                abs(base['stamp'] - joints['stamp']) > self.config['max_skew']):
            return None
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(base['orientation']).as_matrix()
        transform[:3, 3] = base['position']
        try:
            pose = rigid_transform(WholeBodyKinematics.compute(
                self.model, transform, joints['position']))
        except (ValueError, TypeError, IndexError):
            return None
        return pose, min(base['stamp'], joints['stamp']), max(base['stamp'], joints['stamp'])

    def _marker(self, ns, kind, stamp, color, points=(), position=None, text='', scale=None):
        result = {'ns': ns, 'id': 0, 'kind': kind, 'stamp': float(stamp),
                  'color': list(self.config['colors'][color]),
                  'points': np.asarray(points, dtype=float).reshape(-1, 3).tolist(),
                  'scale': list(scale or [self.config['width']] * 3)}
        if position is not None:
            result['position'] = np.asarray(position).tolist()
        if text:
            result['text'] = text
        return result

    def snapshot(self, now):
        """Return the entire scene. Adapters must delete geometry absent from this result."""
        now = self._clock(now)
        paths = {}
        for key, history in self._histories.items():
            self._prune(history, now)
            if self._fresh(key, now) is None:
                history.clear()
            paths[key] = [{'stamp': point['stamp'], 'position': list(point['position'])}
                          for point in history] if self.config['layers'][key] else []
        prediction = self._fresh('prediction', now)
        paths['target_prediction'] = []
        if prediction is not None and self.config['layers']['target_prediction']:
            paths['target_prediction'] = [
                {'stamp': float(stamp), 'position': state[:3].tolist()}
                for stamp, state in zip(prediction['timestamps'], prediction['states'])
                if stamp >= now]
        markers = self._geometry(now) if self.config['enabled'] else []
        if not self.config['enabled']:
            paths = {key: [] for key in paths}
        return {'frame_id': self.config['frame_id'], 'paths': paths, 'markers': markers}

    def _geometry(self, now):
        markers = []
        cfg = self.config
        layers = cfg['layers']
        for key, color in (('target_truth', 'truth'), ('target_estimate', 'estimate')):
            sample = self._fresh(key, now)
            if layers[key + '_current'] and sample is not None:
                markers.append(self._marker(key + '_current', 'sphere', sample['stamp'], color,
                                            position=sample['position'],
                                            scale=[cfg['point_sizes'][color]] * 3))
        camera = self._camera(now)
        target = self._fresh('target_truth', now)
        if camera is not None:
            pose, stamp, newest_stamp = camera
            origin = pose[:3, 3]
            if layers['camera_axis']:
                arrow_scale = [cfg['width'], 2 * cfg['width'], 2 * cfg['width']]
                markers.append(self._marker('camera_origin', 'sphere', stamp, 'origin',
                                            position=origin, scale=[cfg['origin_size']] * 3))
                for index, axis in enumerate(('x', 'y', 'z')):
                    endpoint = origin + cfg['axis_length'] * pose[:3, index]
                    markers.append(self._marker('camera_axis_' + axis, 'arrow', stamp, axis,
                                                [origin, endpoint], scale=arrow_scale))
                endpoint = origin + cfg['forward_length'] * pose[:3, :3] @ self.forward_axis
                markers.append(self._marker('camera_optical', 'arrow', stamp, 'optical',
                                            [origin, endpoint], scale=arrow_scale))
            if (layers['line_of_sight'] and target is not None and
                    max(newest_stamp, target['stamp']) - min(stamp, target['stamp'])
                    <= cfg['max_skew']):
                distance = float(np.linalg.norm(target['position'] - origin))
                source_stamp = min(stamp, target['stamp'])
                markers.append(self._marker('line_of_sight', 'line_list', source_stamp,
                                            'target', [origin, target['position']],
                                            scale=[cfg['overlay_widths']['line_of_sight']] * 3))
                text = (f'Actual camera-target: {distance:.3f} m; '
                        f'desired R: {self.radius:.3f} m')
                markers.append(self._marker('distance', 'text', source_stamp,
                                            'origin', position=(origin + target['position']) / 2,
                                            text=text,
                                            scale=[0., 0., cfg['origin_size'] * 2]))
        if layers['tracking_radius'] and target is not None:
            markers.append(self._marker('tracking_radius', 'line_list', target['stamp'], 'radius',
                                        radius_shell(target['position'], self.radius),
                                        scale=[cfg['overlay_widths']['tracking_radius']] * 3))
        joints = self._fresh('joints', now)
        if layers['joint_limits'] and joints is not None and self.limits is not None:
            lower, upper, velocity = (self.limits[key] for key in ('lower', 'upper', 'velocity'))
            q, dq = joints['position'], joints['velocity']
            violated = (np.any(q < lower - .001) or np.any(q > upper + .001) or
                        np.any(np.abs(dq) > velocity + .001))
            q_util = np.abs(2 * (q - lower) / (upper - lower) - 1)
            dq_util = np.abs(dq) / velocity
            heading = 'MEASURED LIMIT VIOLATION' if violated else 'Measured joint utilization'
            text = heading + '\n'
            text += '\n'.join(f'{name}: q={q[i]:.3f} [{lower[i]:.3f},{upper[i]:.3f}] rad '
                              f'dq={dq[i]:.3f}/{velocity[i]:.3f} rad/s '
                              f'q util={q_util[i]:.3f} dq util={dq_util[i]:.3f}'
                              for i, name in enumerate(self.joint_names))
            position = camera[0][:3, 3] if camera is not None else np.zeros(3)
            markers.append(self._marker('joint_limits', 'text', joints['stamp'],
                                        'safety' if violated else 'origin', position=position,
                                        text=text, scale=[0., 0., cfg['origin_size'] * 2]))
        return markers
