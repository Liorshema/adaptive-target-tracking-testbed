"""Imperfect external planar-localization sensor abstraction, independent of ROS."""
import heapq
import numpy as np


def validate_configuration(config):
    """Reject ignored settings and nonphysical source/noise/timing contracts."""
    required = {'enabled', 'source_topic', 'source_frame', 'child_frame', 'topic',
                'diagnostic_topic', 'diagnostic_period', 'rate_hz', 'timer_period', 'latency', 'latency_jitter',
                'max_delivery_age', 'future_tolerance', 'white_noise_std',
                'calibration_bias_std', 'dropout_probability', 'outages',
                'outlier_probability', 'outlier_offset', 'unobserved_variance', 'random_seed'}
    if not isinstance(config, dict) or set(config) != required:
        raise ValueError('Localization sensor settings are missing or unknown')
    if type(config['enabled']) is not bool:
        raise ValueError('Sensor enabled must be boolean')
    for name in ['source_topic', 'source_frame', 'child_frame', 'topic', 'diagnostic_topic']:
        if not isinstance(config[name], str) or not config[name].strip():
            raise ValueError('Sensor topic/frame must be nonempty')
    if config['topic'] in {config['source_topic'], config['diagnostic_topic']}:
        raise ValueError('Measurement output must be independent of source/diagnostics')
    for name in ['rate_hz', 'timer_period', 'diagnostic_period', 'max_delivery_age', 'unobserved_variance']:
        if type(config[name]) not in (float, int) or not np.isfinite(config[name]) or config[name] <= 0:
            raise ValueError('Sensor rates/age/variance must be finite positive')
    for name in ['latency', 'latency_jitter', 'future_tolerance']:
        if type(config[name]) not in (float, int) or not np.isfinite(config[name]) or config[name] < 0:
            raise ValueError('Sensor latency/tolerance must be finite nonnegative')
    if config['latency'] + config['latency_jitter'] >= config['max_delivery_age']:
        raise ValueError('Sensor delivery budget must exceed modeled latency')
    for name in ['dropout_probability', 'outlier_probability']:
        if type(config[name]) not in (float, int) or not np.isfinite(config[name]) or not 0 <= config[name] <= 1:
            raise ValueError('Sensor probability must be in [0,1]')
    for name in ['white_noise_std', 'calibration_bias_std', 'outlier_offset']:
        v = np.asarray(config[name], dtype=float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise ValueError('Sensor noise/bias/outlier must be finite planar vector')
        if name != 'outlier_offset' and (np.any(v < 0) or (name == 'white_noise_std' and np.any(v <= 0))):
            raise ValueError('Sensor white noise must be positive, bias nonnegative')
    if type(config['random_seed']) is not int or not 0 <= config['random_seed'] < 2**32:
        raise ValueError('Sensor seed must be uint32')
    if not isinstance(config['outages'], list):
        raise ValueError('Sensor outages must be a list')
    for interval in config['outages']:
        if np.asarray(interval).shape != (2,) or not np.isfinite(interval).all() or not 0 <= interval[0] < interval[1]:
            raise ValueError('Sensor outage requires [start,end] source seconds')


class PlanarLocalizationSensor:
    """Acquire biased/noisy poses, then release them at delayed source-clock times."""

    def __init__(self, config):
        validate_configuration(config)
        self.config = config
        self.counters = {'acquired': 0, 'published': 0, 'dropped': 0, 'expired': 0}
        self.reset()

    def reset(self):
        """Discard previous-epoch pending observations and reproduce the seeded epoch."""
        self.rng = np.random.default_rng(self.config['random_seed'])
        self.bias = self.rng.normal(size=3) * self.config['calibration_bias_std']
        self.covariance = np.diag(np.square(self.config['white_noise_std'])
                                  + np.square(self.config['calibration_bias_std']))
        self.pending = []
        self.last_stamp = None
        self.sequence = 0

    def observe(self, value, stamp):
        """Use source acquisition time; reject duplicates and former-epoch arrivals."""
        z = np.asarray(value, dtype=float)
        if z.shape != (3,) or not np.isfinite(z).all() or not np.isfinite(stamp) or stamp < 0:
            raise ValueError('Invalid external-pose source')
        if not self.config['enabled'] or (self.last_stamp is not None and
                stamp - self.last_stamp < 1 / self.config['rate_hz'] - 1e-9):
            return
        self.last_stamp = stamp
        self.counters['acquired'] += 1
        if (any(low <= stamp < high for low, high in self.config['outages'])
                or self.rng.random() < self.config['dropout_probability']):
            self.counters['dropped'] += 1
            return
        noisy = z + self.bias + self.rng.normal(size=3) * self.config['white_noise_std']
        if self.rng.random() < self.config['outlier_probability']:
            noisy += self.config['outlier_offset']
        noisy[2] = np.arctan2(np.sin(noisy[2]), np.cos(noisy[2]))
        release = stamp + self.config['latency'] + self.rng.uniform(0, self.config['latency_jitter'])
        self.sequence += 1
        heapq.heappush(self.pending, (release, self.sequence, stamp, noisy))

    def release(self, now):
        """Return stamped imperfect observations without relabeling delayed samples."""
        if not np.isfinite(now):
            raise ValueError('Invalid sensor clock')
        result = []
        while self.pending and self.pending[0][0] <= now:
            _, _, stamp, value = heapq.heappop(self.pending)
            if now - stamp > self.config['max_delivery_age']:
                self.counters['expired'] += 1
                continue
            self.counters['published'] += 1
            result.append((stamp, value.copy(), self.covariance.copy()))
        return result
