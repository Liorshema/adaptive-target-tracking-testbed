"""ROS-independent conversion of source messages to the versioned telemetry schema."""
from collections.abc import Mapping
import json
import math
from numbers import Real


STREAM_TYPES = {
    'clock': ('rosgraph_msgs.msg', 'Clock'),
    'target_truth': ('geometry_msgs.msg', 'PointStamped'),
    'target_measurement': ('geometry_msgs.msg', 'PointStamped'),
    'joints': ('sensor_msgs.msg', 'JointState'),
    'base_truth': ('nav_msgs.msg', 'Odometry'),
    'odometry': ('nav_msgs.msg', 'Odometry'),
    'robot_estimate': ('robot_interfaces.msg', 'StateEstimate'),
    'target_estimate': ('robot_interfaces.msg', 'StateEstimate'),
    'prediction': ('robot_interfaces.msg', 'PredictedTrajectory'),
    'control': ('robot_interfaces.msg', 'ControlTelemetry'),
    'command': ('robot_interfaces.msg', 'SafeVelocityCommand'),
    'actuation': ('robot_interfaces.msg', 'ActuationTelemetry'),
    'diagnostics': ('std_msgs.msg', 'String'),
}


def message_data(value, *, quarantine=False):
    """Preserve every ROS field without importing ROS; tag nonfinite rejected values."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, Real):
        if not math.isfinite(value):
            if quarantine:
                return {'nonfinite': str(value)}
            raise ValueError('nonfinite numeric field')
        return float(value)
    if isinstance(value, Mapping):
        return {str(key): message_data(item, quarantine=quarantine)
                for key, item in value.items()}
    if hasattr(value, 'get_fields_and_field_types'):
        fields = value.get_fields_and_field_types()
        return {name: message_data(getattr(value, name), quarantine=quarantine)
                for name in fields}
    if hasattr(value, '__dict__'):
        return {name: message_data(item, quarantine=quarantine)
                for name, item in vars(value).items() if not name.startswith('_')}
    try:
        return [message_data(item, quarantine=quarantine) for item in value]
    except TypeError as error:
        raise ValueError(f'unsupported message field type: {type(value).__name__}') from error


def _vector(value, size=3):
    return [value[name] for name in ('x', 'y', 'z', 'w')[:size]]


def _stamp(value):
    seconds, nanos = value['sec'], value['nanosec']
    if not isinstance(seconds, int) or not isinstance(nanos, int) or not 0 <= nanos < 10**9:
        raise ValueError('invalid source timestamp')
    return seconds + nanos * 1e-9


def _numbers(values, length=None):
    if not isinstance(values, list) or (length is not None and len(values) != length):
        raise ValueError('numeric array has incompatible shape')
    if any(isinstance(value, bool) or not isinstance(value, Real) for value in values):
        raise ValueError('numeric array has nonnumeric elements')
    return values


def _normalize_data(stream, raw):
    fields = {key: value for key, value in raw.items() if key != 'header'}
    if stream == 'clock':
        return {'time': _stamp(raw['clock'])}
    if stream in ('target_truth', 'target_measurement'):
        return {'position': _numbers(_vector(raw['point']), 3)}
    if stream == 'joints':
        names = raw['name']
        if not names or len(set(names)) != len(names) or any(not isinstance(n, str) for n in names):
            raise ValueError('joint names must be nonempty and unique')
        position = _numbers(raw['position'], len(names))
        velocity = _numbers(raw['velocity'])
        if velocity and len(velocity) != len(names):
            raise ValueError('joint velocity shape differs from joint names')
        effort = _numbers(raw.get('effort', []))
        if effort and len(effort) != len(names):
            raise ValueError('joint effort shape differs from joint names')
        return {'names': names, 'position': position, 'velocity': velocity, 'effort': effort}
    if stream in ('base_truth', 'odometry'):
        pose, twist = raw['pose']['pose'], raw['twist']['twist']
        return {'position': _numbers(_vector(pose['position']), 3),
                'orientation': _numbers(_vector(pose['orientation'], 4), 4),
                'linear_velocity': _numbers(_vector(twist['linear']), 3),
                'angular_velocity': _numbers(_vector(twist['angular']), 3),
                'pose_covariance': _numbers(raw['pose']['covariance'], 36),
                'twist_covariance': _numbers(raw['twist']['covariance'], 36),
                'child_frame_id': raw['child_frame_id']}
    if stream in ('robot_estimate', 'target_estimate'):
        mean = _numbers(raw['mean'])
        if not mean:
            raise ValueError('empty state estimate')
        _numbers(raw['covariance'], len(mean)**2)
        return fields
    if stream == 'prediction':
        horizon, dimension = raw['horizon'], raw['state_dimension']
        if (isinstance(horizon, bool) or isinstance(dimension, bool) or
                not isinstance(horizon, int) or not isinstance(dimension, int) or
                horizon <= 0 or dimension <= 0):
            raise ValueError('prediction dimensions must be positive integers')
        timestamps = _numbers(raw['timestamps'], horizon)
        if any(second <= first for first, second in zip(timestamps, timestamps[1:])):
            raise ValueError('prediction absolute timestamps must increase')
        states = _numbers(raw['states'], horizon * dimension)
        return {**fields, 'states': [states[i*dimension:(i+1)*dimension] for i in range(horizon)]}
    if stream == 'control':
        for field in ('raw_velocity', 'safe_velocity'):
            _numbers(raw[field])
        if len(raw['raw_velocity']) != len(raw['safe_velocity']):
            raise ValueError('raw and safe control velocities have different shapes')
        return {**fields,
                'actual_position': _numbers(_vector(raw['actual_pose']['position']), 3),
                'actual_orientation': _numbers(_vector(raw['actual_pose']['orientation'], 4), 4),
                'target_position': _numbers(_vector(raw['target_position']), 3)}
    if stream == 'command':
        _numbers(raw['velocity'], len(raw['joint_names']) + 2)
        return fields
    if stream == 'actuation':
        for field in ('commanded_velocity', 'position_reference', 'measured_positions'):
            _numbers(raw[field])
        return fields
    if stream == 'diagnostics':
        data = {'text': raw['data']}
        try:
            parsed = json.loads(raw['data'])
            # JSON strings may themselves contain a non-standard NaN literal.
            message_data(parsed)
            data['parsed'] = parsed
        except (ValueError, TypeError):
            pass
        return data
    raise ValueError(f'unknown telemetry stream: {stream}')


def normalize(stream, message, arrival_monotonic, clock):
    """Normalize one observation, rejecting malformed numeric data.

    Source time is null only for genuinely unstamped diagnostic strings. It is
    never replaced with a host timestamp or an estimated simulation timestamp.
    Original message fields, source frame, and arrival clock remain inspectable.
    """
    try:
        raw = message_data(message)
        arrival = float(arrival_monotonic)
        if not math.isfinite(arrival) or (clock is not None and not math.isfinite(clock)):
            raise ValueError('nonfinite reception clock')
        data = _normalize_data(stream, raw)
        stamp = data['time'] if stream == 'clock' else (
            _stamp(raw['header']['stamp']) if 'header' in raw else None)
        if stamp is None and stream != 'diagnostics':
            raise ValueError('missing source timestamp')
        frame = raw.get('header', {}).get('frame_id', '')
        if not isinstance(frame, str):
            raise ValueError('invalid source frame identifier')
        return {'schema_version': 1, 'source_time': stamp,
                'arrival_monotonic': arrival,
                'clock': stamp if stream == 'clock' else clock,
                'frame_id': frame, 'data': data, 'raw_message': raw,
                'units': {'source_time': 's', 'arrival_monotonic': 's', 'clock': 's',
                          'position': 'm', 'orientation': 'quaternion_xyzw',
                          'linear_velocity': 'm/s', 'angular_velocity': 'rad/s',
                          'joint_position': 'rad', 'joint_velocity': 'rad/s'}}
    except (KeyError, TypeError, AttributeError, OverflowError) as error:
        raise ValueError(f'malformed {stream}: {error}') from error
