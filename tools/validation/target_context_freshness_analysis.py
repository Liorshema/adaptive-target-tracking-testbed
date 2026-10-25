"""Per-decision source ages and conservative causal classification, with M03 scoring."""
import argparse
from collections import Counter
import json
import re
from pathlib import Path

import numpy as np
from m03_robustness_validation import audit


def analyze(directory):
    result = audit(directory)
    if 'failed_attempt' in result: return result
    read = lambda stream:[json.loads(s) for s in (directory/'telemetry'/f'{stream}.jsonl').read_text().splitlines()]
    controls = read('control'); diagnostics = [r['data']['parsed'] for r in read('diagnostics') if r['data'].get('parsed')]
    first=result['result']['first_valid_control_time']; end=result['end']
    intervals=[]; current=None; classified=Counter(); ages={'robot':[],'prediction':[]}
    for r in controls:
        t=r['source_time']; data=r['data']
        if t<0 or t>end:continue
        robot=data.get('robot_state_time');prediction=data.get('prediction_reference_time')
        a={'robot':None if robot is None or robot<0 else t-robot,
           'prediction':None if prediction is None or prediction<0 else t-prediction}
        for key,value in a.items():
            if value is not None:ages[key].append(value)
        if data['valid']:
            if current:
                current['end']=t;current['duration']=t-current['start']; intervals.append(current);current=None
            continue
        reason=data['reason']; category='unresolved'
        prior=[d for d in diagnostics if 0<=t-d['time']<=.3]
        health=prior[-1].get('localization') if prior else None
        if t<first:category='launch_initialization'
        elif 'prediction' in reason.lower():category='target_prediction_or_context'
        elif reason=='Robot state is stale' and a['robot'] is not None and a['robot']>.2:
            # Health is sampled. Never call this a precise trust transition.
            category=('robot_stale_with_sampled_untrusted_localization'
                      if health and not health.get('trusted',True) else 'robot_stale_unresolved_source_or_scheduling')
        elif 'Waiting for robot' in reason:category='robot_absent'
        elif reason=='Robot state is stale':category='robot_stale_historical_age_unavailable'
        classified[category]+=1
        failures=[]
        for key,limit in [('robot',.2),('prediction',.5)]:
            stamp=robot if key=='robot' else prediction
            if stamp is not None and stamp<0: failures.append(key+'_absent')
            elif a[key] is not None and a[key]>limit: failures.append(key+'_stale')
            elif a[key] is not None and a[key]<-.01: failures.append(key+'_future')
        row={'time':t,'reason':reason,'category':category,'input_ages':a,
             'observed_input_failures':failures,
             'controller_action':'inactive zero command, per tested ControlNode._stop contract',
             'sampled_localization_health':health}
        if current is None:current={'start':t,'decisions':[]}
        current['decisions'].append(row)
    if current:
        current['end']=None;current['duration_lower_bound']=end-current['start'];intervals.append(current)
    def summary(x):
        return {'count':len(x),'mean':float(np.mean(x)) if x else None,'max':max(x,default=None),
                'min':min(x,default=None),'p95':float(np.percentile(x,95)) if x else None}
    streams={}
    for key in ['odometry','joints','localization_measurement','robot_estimate','target_measurement','target_estimate','prediction']:
        records=read(key); stamps=[r['source_time'] for r in records if r['source_time'] is not None]
        unique=sorted(set(stamps));diff=np.diff(unique)
        streams[key]={'records':len(records),'source_start':min(stamps,default=None),'source_end':max(stamps,default=None),
                      'median_source_period':float(np.median(diff)) if len(diff) else None,
                      'max_source_gap':max(diff,default=None),'repeated_source_stamps':len(stamps)-len(unique),
                      'arrival_order_regressions':sum(b<a for a,b in zip(stamps,stamps[1:]))}
    result['decision_input_ages_s']={k:summary(v) for k,v in ages.items()}
    result['invalid_intervals']=intervals;result['invalid_decision_classification']=dict(classified)
    result['stream_timing']=streams
    log=(directory/'launch.log').read_text()
    events=[]; seen=set(); pending=None; waits=[]
    for line in log.splitlines():
        match=re.search(r'Target context (waiting|available): epoch=(\d+), state_time=([^,]+), clock=([0-9.]+)',line)
        if not match: continue
        key=match.groups()
        if key in seen: continue
        seen.add(key); mode,epoch,state,t=key
        row={'mode':mode,'epoch':int(epoch),'state_time':None if state=='None' else float(state),'clock':float(t)}
        events.append(row)
        if mode=='waiting':pending=row
        elif pending and pending['epoch']==row['epoch']:
            waits.append({'start':pending,'end':row,'duration_s':row['clock']-pending['clock']}); pending=None
    result['context_waits']={'events':events,'completed':waits,'open_wait':pending,
        'max_completed_duration_s':max((w['duration_s'] for w in waits),default=0.),
        'meaning':'Availability transitions; partial source-time progress may continue during a wait. Absence of the old exception text is not absence of waiting.'}
    result['context_errors']={'unavailable':log.count('Physical context unavailable'),
                            'terminal_gap':log.count('Irrecoverable target context/time gap'),
                            'integration_step_limit':log.count('integration step limit')}
    result['classification_limit']='Input source epochs are exact per decision when present. Trust is sampled at5Hz; initiating source absence versus callback delay remains unresolved unless separately instrumented. Historical messages lack the new input-epoch fields.'
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    results={str(d):analyze(d) for d in a.runs}
    with a.output.open('x') as f:json.dump(results,f,indent=2)
    for d,r in results.items():
        print(d, 'active=',r.get('control',{}).get('active',{}).get('valid_fraction'),r.get('invalid_decision_classification'),flush=True)
