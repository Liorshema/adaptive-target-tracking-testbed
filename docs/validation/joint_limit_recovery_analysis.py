"""Run-local deadlock evidence from full-step CSV and unchanged telemetry records."""
import csv
from collections import Counter
import json
from pathlib import Path


def control_continuity(path, start, end):
    """Expose sustained invalid control and missing coverage in a run window.

    The caller's one-second diagnostic gate is not a controller safety timeout.
    Source timestamps are used; startup and shutdown outside the window are excluded.
    """
    with Path(path).open() as stream:
        rows = [json.loads(line) for line in stream]
    rows = sorted((r for r in rows if r['source_time'] is not None
                   and start <= r['source_time'] <= end),
                  key=lambda r: r['source_time'])
    invalid_start = start
    longest = 0.
    previous = start
    gap = 0.
    valid_count = 0
    reasons = Counter()
    for row in rows:
        t = row['source_time']
        gap = max(gap, t - previous)
        previous = t
        if row['data']['valid']:
            valid_count += 1
            if invalid_start is not None:
                longest = max(longest, t - invalid_start)
                invalid_start = None
        else:
            reasons[row['data'].get('reason', '')] += 1
            if invalid_start is None:
                invalid_start = t
    if invalid_start is not None:
        longest = max(longest, end - invalid_start)
    return {'samples': len(rows), 'valid_samples': valid_count,
            'max_invalid_duration_s': longest,
            'max_sample_gap_s': max(gap, end - previous),
            'invalid_reasons': dict(reasons)}


def stalled_windows(path):
    """Measure contiguous inward-demand/stationary windows, not a sample count.

    The diagnostic thresholds (0.05 demand, 0.001 physical velocity) are the
    user's original failure filter, not new safety acceptance tolerances.
    """
    current = {}; windows = []; count = 0
    def finish(name):
        item=current.pop(name,None)
        if item is not None:
            item['duration_s']=item['end']-item['start']+item['dt']
            windows.append(item)
    with Path(path).open() as stream:
        for row in csv.DictReader(stream):
            count+=1
            name=row['name'];t=float(row['time']);q=float(row['q_post'])
            demand=float(row['target_velocity']);v=float(row['v_post'])
            target=float(row['position_reference']);dt=float(row['dt'])
            valid=(abs(demand)>.05 and abs(v)<.001 and demand*(target-q)>0
                   and row.get('valid_step','1')=='1')
            item=current.get(name)
            if item and (not valid or t-item['end']>dt*1.01+1e-9):
                finish(name);item=None
            if valid:
                if item is None:
                    item={'joint':name,'start':t,'end':t,'dt':dt,'samples':0,
                          'q_start':q,'q_end':q,'max_abs_demand':0.,'max_abs_velocity':0.}
                    current[name]=item
                item.update(end=t,q_end=q,samples=item['samples']+1,
                            max_abs_demand=max(item['max_abs_demand'],abs(demand)),
                            max_abs_velocity=max(item['max_abs_velocity'],abs(v)))
    for name in list(current):finish(name)
    windows.sort(key=lambda w:w['duration_s'],reverse=True)
    return {'rows':count,'max_contiguous_stall_s':windows[0]['duration_s'] if windows else 0.,
            'windows_over_one_second':[w for w in windows if w['duration_s']>=1.],
            'longest_windows':windows[:12]}


def telemetry_summary(directory, start=None, end=None):
    """Source-time inactive/recovery windows and actual command/feedback evidence."""
    directory=Path(directory);rows=[]
    with (directory/'actuation.jsonl').open() as stream:
        rows=[json.loads(line) for line in stream]
    rows=[r for r in rows if r['source_time'] is not None
          and (start is None or r['source_time'] >= start)
          and (end is None or r['source_time'] <= end)]
    intervals=[];current=None
    for r in rows:
        d=r['data'];t=r['source_time']
        if t is None:continue
        recovery=d['reason'].startswith('Position ')
        if recovery:
            if current is None:
                current={'start':t,'end':t,'samples':0,'feedback_valid_samples':0,
                         'reason_first':d['reason'],'timeouts_first':d['timeout_events']}
            current.update(end=t,samples=current['samples']+1,
                           feedback_valid_samples=current['feedback_valid_samples']+int(d['feedback_valid']),
                           timeouts_last=d['timeout_events'])
        elif current is not None:
            current.update(duration_s=t-current['start'],released_to_active=bool(d['active']),
                           next_reason=d['reason'],release_time=t)
            intervals.append(current);current=None
    if current is not None:
        current.update(duration_s=current['end']-current['start'],released_to_active=False,unfinished=True)
        intervals.append(current)
    return {'samples':len(rows),'active_samples':sum(bool(r['data']['active']) for r in rows),
            'feedback_valid_samples':sum(bool(r['data']['feedback_valid']) for r in rows),
            'recoveries':intervals,'last_actuation':rows[-1] if rows else None}
