"""Append-only source observations and explicit availability/error accounting."""
import json
from pathlib import Path
import time

from .normalization import STREAM_TYPES, message_data, normalize


class TelemetryStore:
    """Receive normalized observations without requiring a ROS runtime."""

    def __init__(self, context, directory):
        self.context = context
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.streams = {key: [] for key in STREAM_TYPES}
        self.availability = {key: {'topic': context.get('topics', {}).get(key),
                                  'record_count': 0, 'invalid_count': 0,
                                  'out_of_order_count': 0, 'last_source_time': None}
                             for key in STREAM_TYPES}
        self.events = []
        self.clock = None
        self.last_clock_monotonic = time.monotonic()
        self.clock_reset = False
        self._files = {}
        self._event_file = (self.directory / 'events.jsonl').open('x', buffering=1)
        self._closed = False

    def _event(self, kind, stream, arrival, **fields):
        event = {'schema_version': 1, 'type': kind, 'stream': stream,
                 'arrival_monotonic': arrival, 'clock': self.clock, **fields}
        self._event_file.write(json.dumps(event, allow_nan=False) + '\n')
        self.events.append(event)

    def record(self, stream, message, arrival_monotonic=None):
        if self._closed:
            raise RuntimeError('telemetry store is closed')
        arrival = time.monotonic() if arrival_monotonic is None else arrival_monotonic
        if stream not in self.streams:
            raise ValueError(f'unknown telemetry stream: {stream}')
        metadata = self.availability[stream]
        try:
            record = normalize(stream, message, arrival, self.clock)
        except ValueError as error:
            metadata['invalid_count'] += 1
            self._event('rejected_message', stream, arrival, reason=str(error),
                        raw_message=message_data(message, quarantine=True))
            return None
        stamp = record['source_time']
        previous = metadata['last_source_time']
        if stamp is not None and previous is not None and stamp < previous:
            metadata['out_of_order_count'] += 1
            self._event('out_of_order_source', stream, arrival,
                        source_time=stamp, previous_source_time=previous)
        if stream == 'clock':
            if self.clock is not None and stamp < self.clock:
                self.clock_reset = True
                self._event('clock_reset', stream, arrival,
                            source_time=stamp, previous_source_time=self.clock)
            if self.clock is None or stamp != self.clock:
                self.last_clock_monotonic = arrival
            self.clock = stamp
        record['provenance'] = {'topic': metadata['topic'],
                                'message_type': '.'.join(STREAM_TYPES[stream])}
        if stream not in self._files:
            self._files[stream] = (self.directory / f'{stream}.jsonl').open('x', buffering=1)
        self._files[stream].write(json.dumps(record, allow_nan=False) + '\n')
        self.streams[stream].append(record)
        metadata['record_count'] += 1
        metadata['last_source_time'] = stamp
        return record

    def ready(self, required_streams):
        if not all(bool(self.streams.get(stream)) for stream in required_streams):
            return False
        max_age = self.context.get('readiness_max_age')
        if max_age is None:
            return True
        if self.clock is None:
            return False
        for stream in required_streams:
            stamp = self.streams[stream][-1]['source_time']
            if stamp is None or abs(self.clock - stamp) > max_age:
                return False
        return True

    def close(self):
        if self._closed:
            return
        try:
            for handle in [*self._files.values(), self._event_file]:
                handle.flush()
                handle.close()
        finally:
            self._closed = True
