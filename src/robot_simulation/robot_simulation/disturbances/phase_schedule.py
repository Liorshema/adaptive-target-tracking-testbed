"""Stateless source-time schedule with cubic transitions between parameter vectors."""
import numpy as np


class PhaseSchedule:
    """Hold first/last values outside sequence; blend at each later phase start."""

    def __init__(self, phases):
        """Validate explicit constructor parameters."""
        if not isinstance(phases, list) or not phases:
            raise ValueError('phases must be a nonempty list')
        self.phases = []
        self.starts = []
        start = 0.
        names = set()
        for phase in phases:
            expected = {'name', 'duration', 'transition', 'values'}
            if not isinstance(phase, dict) or set(phase) != expected:
                raise ValueError('phase requires name, duration, transition, values only')
            name = phase['name']
            if not isinstance(name, str) or not name.strip() or name in names:
                raise ValueError('phase names must be unique nonempty strings')
            names.add(name)
            duration, transition = phase['duration'], phase['transition']
            if any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not np.isfinite(v) for v in [duration, transition]):
                raise ValueError('phase timing must be finite numbers')
            if duration <= 0 or not 0 <= transition <= duration:
                raise ValueError('require duration>0 and 0<=transition<=duration')
            if (not self.phases and transition != 0) or (self.phases and transition <= 0):
                raise ValueError('first transition must be zero; subsequent transitions positive')
            raw = phase['values']
            if not isinstance(raw, list) or any(type(v) not in (int, float) for v in raw):
                raise ValueError('phase values must be a numeric list')
            values = np.asarray(raw, dtype=float)
            if values.shape != (3,) or not np.isfinite(values).all() or np.any(values[:2] < 0):
                raise ValueError('values require finite [rho>=0, height>=0, tangent]')
            self.starts.append(start)
            self.phases.append({**phase, 'values': values.copy()})
            start += duration
            if not np.isfinite(start):
                raise ValueError('total schedule duration must be finite')
        self.duration = start

    def index(self, time):
        """Return phase index using simulation source seconds (reset is stateless)."""
        if not np.isfinite(time):
            raise ValueError('source time must be finite')
        index = int(np.searchsorted(self.starts, time, side='right') - 1)
        return max(0, min(len(self.phases) - 1, index))

    def values(self, time):
        """Return preferred radius/height and tangential acceleration in SI units."""
        index = self.index(time)
        phase = self.phases[index]
        if index == 0:
            return phase['values'].copy()
        tau = np.clip((time-self.starts[index])/phase['transition'], 0., 1.)
        blend = tau*tau*(3.-2.*tau)
        return self.phases[index-1]['values']*(1.-blend)+phase['values']*blend
