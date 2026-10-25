"""Bounded opt-in observer diagnostics; never a source of scientific timestamps."""

import json
import threading
import time
from bisect import bisect_left
from collections import deque
from pathlib import Path


BANDS = (.00001, .00005, .0001, .0005, .001, .005, .01, .02, .05, .08, .15, .2, .3, 1., 10.)
FIELDS = ('publisher_gid', 'source_timestamp', 'received_timestamp',
          'publication_sequence_number', 'reception_sequence_number')
INTERVALS = (('normalization', 'normalization_start', 'normalization_end'),
             ('serialization', 'serialization_start', 'serialization_end'),
             ('write', 'write_start', 'write_end'), ('total', 'entry', 'exit'))


class Summary:
    """Constant-size exact count/min/mean/max plus bounded duration histogram."""

    def __init__(self):
        """Allocate a fixed histogram independent of sample count."""
        self.count = 0
        self.total = 0.
        self.low = self.high = None
        self.bins = [0] * (len(BANDS) + 1)

    def add(self, value):
        """Accumulate one numeric observation."""
        self.count += 1
        self.total += value
        if self.low is None or value < self.low:
            self.low = value
        if self.high is None or value > self.high:
            self.high = value
        self.bins[bisect_left(BANDS, value)] += 1

    def document(self):
        """Return explicit units from the containing schema key."""
        return {'count': self.count, 'min': self.low, 'mean': self.total/self.count
                if self.count else None, 'max': self.high, 'histogram': self.bins.copy()}


class ObserverTiming:
    """Own bounded summaries, callback/batch rings and an independent root sampler."""

    def __init__(self, directory, streams, *, monitor=True, capacity=128, now=time.monotonic):
        """Allocate bounded rings; resource sampling is optional for offline tests."""
        self.directory = Path(directory)
        self.now = now
        self.capacity = capacity
        self.streams = tuple(streams)
        self.processing = {s: {'count': 0, 'last_entry': None, 'last_source_time': None,
                               'last_observer_clock': None, 'source_discontinuities': 0,
                               'stages': {name: Summary() for name in
                                          ('normalization', 'serialization', 'write',
                                           'total', 'pre_normalization', 'other')},
                               'gap': Summary(), 'ahead_count': 0,
                               'age': Summary()} for s in streams}
        self.metadata = {}
        for stream in streams:
            fields = {f: {'available_count': 0, 'unavailable_count': 0} for f in FIELDS}
            self.metadata[stream] = {'fields': fields, 'publishers': {},
                                     'publication_sequence_gaps': 0, 'sequence_resets': 0}
        self.events = deque(maxlen=capacity)
        self.service_gaps = deque(maxlen=capacity)
        self.middleware_samples = deque(maxlen=capacity)
        self.sequence_events = deque(maxlen=capacity)
        self.batches = deque(maxlen=capacity)
        self.long_batches = deque(maxlen=capacity)
        self.batch_count = 0
        self.batch_summary = Summary()
        self.between_batches = Summary()
        self.stop_counts = {}
        self._batch = None
        self._last_batch_end = None
        self.executor_calls = {}
        self._last_timeout = None
        self.timeout_switches = 0
        self.failure = None
        self.capabilities = {}
        self.resource_samples = deque(maxlen=capacity)
        self.resource_summaries = {}
        self.resource_error = None
        self._stop = threading.Event()
        self._thread = None
        if monitor:
            self._thread = threading.Thread(target=self._monitor, daemon=True,
                                            name='observer-resource-sampler')
            self._thread.start()

    def failed(self, error):
        """Retain the first collector fault without masking a scientific exception."""
        if self.failure is None:
            self.failure = str(error)[:512]

    def begin_batch(self, timeout):
        """Mark observer-level boundaries, not private executor dispatch hooks."""
        start = self.now()
        self.batch_count += 1
        if self._last_batch_end is not None:
            gap = start-self._last_batch_end
            self.between_batches.add(gap)
            if gap >= .02:
                self.service_gaps.append({'kind': 'between_batches',
                                          'start': self._last_batch_end, 'end': start,
                                          'duration_s': gap})
        self._batch = {'id': self.batch_count, 'start_monotonic': start,
                       'initial_timeout_s': timeout, 'callbacks': 0, 'per_stream': {},
                       'order': [], 'executor_calls': 0, 'executor_duration_s': 0.}

    def executor_call(self, timeout, start, end):
        """Measure public spin-call envelope, including callback work and dispatch/wait."""
        if self._last_timeout is not None and self._last_timeout != timeout:
            self.timeout_switches += 1
        self._last_timeout = timeout
        key = str(timeout)
        summary = self.executor_calls.get(key)
        if summary is None:
            summary = self.executor_calls[key] = Summary()
        summary.add(end-start)
        self._batch['executor_calls'] += 1
        self._batch['executor_duration_s'] += end-start

    def start_drain(self):
        """Exclude the initial blocking spin from the existing 10 ms drain budget."""
        self._batch['drain_start_monotonic'] = self.now()

    def end_batch(self, reason):
        """Keep bounded batch evidence and current per-stream service ages."""
        b = self._batch
        if b is None:
            return
        end = self.now()
        if reason == 'count_budget' and b['callbacks'] < 128:
            reason = 'spin_count_budget'
        elapsed = end-b.get('drain_start_monotonic', end)
        b.update(end_monotonic=end, duration_s=end-b['start_monotonic'], stop_reason=reason,
                 count_budget_hit=b['callbacks'] >= 128,
                 spin_count_budget_hit=b['executor_calls'] >= 128,
                 time_budget_hit=reason == 'time_budget' or elapsed >= .01,
                 time_budget_checked_stop=reason == 'time_budget',
                 drain_duration_s=elapsed,
                 since_last_service_s={s: end-p['last_entry'] if p['last_entry'] is not None
                                       else None for s, p in self.processing.items()})
        self.batch_summary.add(b['duration_s'])
        self.stop_counts[reason] = self.stop_counts.get(reason, 0)+1
        self.batches.append(b)
        if b['duration_s'] >= .08:
            self.long_batches.append(b)
        self._last_batch_end = end
        self._batch = None

    def callback(self, stream, marks, record, info, error):
        """Aggregate measured stages once; retain no payload copies or per-message files."""
        p = self.processing[stream]
        p['count'] += 1
        entry, end = marks['entry'], marks['exit']
        if p['last_entry'] is not None:
            gap = entry-p['last_entry']
            p['gap'].add(gap)
            if gap >= .08:
                self.service_gaps.append({'kind': 'callback_entry_gap', 'stream': stream,
                                          'start': p['last_entry'], 'end': entry,
                                          'duration_s': gap})
        p['last_entry'] = entry
        measured = {}
        operation_duration = 0.
        for name, left, right in INTERVALS:
            if left in marks and right in marks:
                value = marks[right]-marks[left]
                p['stages'][name].add(value)
                measured[name] = value
                if name != 'total':
                    operation_duration += value
        if 'normalization_start' in marks:
            p['stages']['pre_normalization'].add(
                marks['normalization_start']-entry)
        other = measured['total']-operation_duration
        p['stages']['other'].add(max(0., other))
        if record is not None:
            stamp, clock = record['source_time'], record['clock']
            if stamp is not None and p['last_source_time'] is not None:
                if stamp <= p['last_source_time']:
                    p['source_discontinuities'] += 1
            p['last_source_time'], p['last_observer_clock'] = stamp, clock
            if stamp is not None and clock is not None:
                p['age'].add(clock-stamp)
                p['ahead_count'] += stamp > clock+1e-9
        if self._batch is not None:
            b = self._batch
            b['callbacks'] += 1
            b['per_stream'][stream] = b['per_stream'].get(stream, 0)+1
            if len(b['order']) < 128:
                b['order'].append(stream)
        if end-entry >= .02 or error:
            self.events.append({'stream': stream, 'marks': marks.copy(), 'durations_s': measured,
                                'source_time': record['source_time'] if record else None,
                                'observer_clock': record['clock'] if record else None,
                                'error': str(error)[:256] if error else None})
        values, gap = self._metadata(stream, info)
        if p['count'] == 1 or p['count'] % 64 == 0 or gap:
            sample = {'stream': stream, 'callback_entry': entry, 'metadata': values,
                      'source_time': record['source_time'] if record else None,
                      'observer_clock': record['clock'] if record else None,
                      'publication_sequence_gap': gap}
            self.middleware_samples.append(sample)
            if gap:
                self.sequence_events.append(sample)

    def _metadata(self, stream, info):
        m = self.metadata[stream]
        values = {}
        for key in FIELDS:
            value = info.get(key) if isinstance(info, dict) else getattr(info, key, None)
            if key == 'publisher_gid':
                try:
                    value = bytes(value).hex() if value is not None else None
                except (TypeError, ValueError):
                    value = None
                if value and not any(c != '0' for c in value):
                    value = None
            elif not isinstance(value, int) or value <= 0 or value == 2**64-1:
                value = None
            m['fields'][key]['available_count' if value is not None else 'unavailable_count'] += 1
            values[key] = value
        gid = values['publisher_gid']
        gap = 0
        if gid is not None and (gid in m['publishers'] or len(m['publishers']) < 4):
            publisher = m['publishers'].setdefault(gid, {})
            sequence = values['publication_sequence_number']
            previous = publisher.get('publication_sequence_number')
            if sequence is not None and previous is not None:
                if sequence > previous+1:
                    gap = sequence-previous-1
                    m['publication_sequence_gaps'] += gap
                elif sequence <= previous:
                    m['sequence_resets'] += 1
            publisher.update({k: v for k, v in values.items() if v is not None})
        elif gid is not None:
            m['untracked_publisher_observations'] = m.get('untracked_publisher_observations', 0)+1
        return values, gap

    def _monitor(self):
        """Sample root-process/host counters independently at 2 Hz, with bounded memory."""
        try:
            import psutil
            process = psutil.Process()
            process.cpu_percent()
            while not self._stop.wait(.5):
                row = {'monotonic': self.now(), 'cpu_percent': process.cpu_percent(),
                       'rss_bytes': process.memory_info().rss}
                try:
                    row['io'] = process.io_counters()._asdict()
                except (psutil.Error, NotImplementedError):
                    row['io'] = None
                try:
                    row['host_load_1m'] = psutil.getloadavg()[0]
                    row['leader_schedstat'] = Path('/proc/self/schedstat').read_text().strip()
                    row['cgroup_cpu_stat'] = Path('/sys/fs/cgroup/cpu.stat').read_text().strip()
                except (OSError, NotImplementedError):
                    row['host_load_1m'] = row.get('host_load_1m')
                self.resource_samples.append(row)
                for name in ['cpu_percent', 'rss_bytes']:
                    self.resource_summaries.setdefault(name, Summary()).add(row[name])
        except Exception as error:
            self.resource_error = str(error)[:512]

    def snapshot(self):
        """Serialize only at shutdown; histogram percentiles remain explicit approximations."""
        callbacks, ages = {}, {}
        for s, p in self.processing.items():
            callbacks[s] = {k: p[k] for k in ['count', 'last_entry', 'last_source_time',
                                              'last_observer_clock', 'source_discontinuities']}
            callbacks[s].update(stages_s={k: v.document() for k, v in p['stages'].items()
                                          if v.count},
                                entry_gap_s=p['gap'].document())
            ages[s] = {'signed_age_s': p['age'].document(), 'ahead_count': p['ahead_count']}
        return {'schema_version': 1, 'enabled': True, 'complete': False,
                'instrumentation': {'implementation': 'observer_timing',
                                    'duration_clock': 'host_monotonic_seconds',
                                    'source_clock': 'simulation_seconds',
                                    'histogram_upper_bounds_s': list(BANDS)+[None],
                                    'callback_end': 'before diagnostic aggregation',
                                    'executor_dispatch': None,
                                    'dispatch_reason':
                                        'public spin-call and callback boundaries only'},
                'bounds': {'capacity': self.capacity, 'batch_order': 128,
                           'publishers_per_stream': 4},
                'failure': self.failure, 'executor': {
                    'batch_count': self.batch_count,
                    'batch_duration_s': self.batch_summary.document(),
                    'between_batches_s': self.between_batches.document(),
                    'budget_hit_counts': self.stop_counts.copy(),
                    'timeout_switches': self.timeout_switches,
                    'spin_calls_s': {k: v.document() for k, v in self.executor_calls.items()},
                    'recent_batches': list(self.batches), 'long_batches': list(self.long_batches),
                    'generator_restarts': None,
                    'restart_reason': 'no private executor hook; timeout switches observed'},
                'callbacks': {'streams': callbacks, 'long_operations': list(self.events)},
                'service_gaps': list(self.service_gaps),
                'observer_clock': {'streams': ages},
                'middleware': {'capabilities': self.capabilities.copy(), 'streams': self.metadata,
                               'samples': list(self.middleware_samples),
                               'sequence_events': list(self.sequence_events),
                               'timestamp_domain':
                                   'raw middleware ns; no ROS/monotonic conversion',
                               'unavailable_reason':
                                   'field absent, null or unsupported nonpositive/sentinel',
                               'message_lost_events': {
                                   'value': None, 'reason':
                                       'no new event waitable; loss callback not attached'},
                               'sequence_gap_semantics':
                                   'per observed publisher; not loss-location proof'},
                'resources': {'samples': list(self.resource_samples),
                              'summary': {k: v.document()
                                          for k, v in self.resource_summaries.items()},
                              'error': self.resource_error, 'off_cpu_trace': None,
                              'off_cpu_reason':
                                  'no scheduler trace; schedstat is leader-thread counters'},
                'overhead': {'measured_here': False,
                             'reason': 'use matched external replay benchmark'}}

    def finalize(self, *, complete=False, error=None):
        """Atomically persist one bounded artifact, retaining partial/error state."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.)
            if self._thread.is_alive():
                self.failed('resource sampler did not stop within 1 second')
        document = self.snapshot()
        document.update(complete=bool(complete and self.failure is None), run_error=error)
        self.directory.mkdir(parents=True, exist_ok=True)
        temporary = self.directory / 'observer_timing.json.tmp'
        temporary.write_text(json.dumps(document, indent=2, allow_nan=False))
        temporary.replace(self.directory / 'observer_timing.json')
        return document

    def close(self):
        """Stop the independent sampler and attempt a partial artifact on every shutdown."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.)
            if self._thread.is_alive():
                self.failed('resource sampler did not stop within 1 second')
        self.finalize(complete=False)
