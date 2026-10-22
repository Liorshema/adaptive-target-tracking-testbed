"""Recompute final M02 evidence and comparison plots from preserved trial files."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from robot_experiments.metrics.actuator_trace import analyze_trace
from robot_experiments.metrics.joint_diagnostics import causal_indices
from robot_experiments.reporting import write_json


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def summarize(path):
    result = json.loads((path/'result.json').read_text())
    context = json.loads((path/'resolved_configuration.json').read_text())['context']
    trace = path/'physics/actuator.csv'
    # Recheck every joint on a common complete iteration grid, not ROS delivery.
    result['physical'] = analyze_trace(trace,context,result['first_valid_control_time'])
    with trace.open() as stream:rows = list(csv.DictReader(stream))
    by_name = {n:[r for r in rows if r['name']==n] for n in context['joint_names']}
    commands = [r for r in records(path/'telemetry/command.jsonl')
                if r['data'].get('active') and r['source_time']>0]
    commands = sorted({r['source_time']:r for r in commands}.values(),key=lambda r:r['source_time'])
    ct = np.array([r['source_time'] for r in commands])
    for i,name in enumerate(context['joint_names']):
        a = by_name[name];t = np.array([float(r['time']) for r in a])
        v = np.array([float(r['v_post']) for r in a])
        index = causal_indices(ct,t,.15)
        valid = index>=0
        cmd = np.array([r['data']['velocity'][2+r['data']['joint_names'].index(name)]
                        for r in commands])
        errors = v[valid]-cmd[index[valid]]
        result['physical']['joints'][name]['dq_minus_published_command'] = {
            'alignment':'last nonfuture source stamp within 0.15s, physical grid samples',
            'samples':int(valid.sum()),'rms_rad_s':float(np.sqrt(np.mean(errors**2))),
            'max_abs_rad_s':float(np.max(np.abs(errors)))}
    base = records(path/'telemetry/base_truth.jsonl')
    points = np.array([r['data']['position'][:2] for r in base])
    result['base_activity'] = {'horizontal_sampled_travel_m':float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
        'max_horizontal_excursion_m':float(np.linalg.norm(points-points[0],axis=1).max())}
    return result,by_name,context


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before',type=Path,required=True)
    parser.add_argument('--after',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    evidence = {'before_root':str(args.before),'after_root':str(args.after),'cases':[]}
    for predictor in ['cv','imm']:
        for seed in [42,917]:
            before,brows,context = summarize(args.before/f'instrumented_pid_{predictor}_{seed}')
            after,arows,_ = summarize(args.after/f'velocity_servo_{predictor}_{seed}')
            case = {'predictor':predictor,'seed':seed,'before':before,'after':after}
            evidence['cases'].append(case)
            fig,axes = plt.subplots(len(context['joint_names']),4,figsize=(18,12),sharex=True)
            for i,name in enumerate(context['joint_names']):
                for label,data,color in [('PID',brows[name],'0.55'),('Governed motor',arows[name],'tab:blue')]:
                    a = lambda key:np.array([float(r[key]) for r in data])
                    t = a('time');q = a('q_post');v = a('v_post');reference = a('position_reference')
                    axes[i,0].plot(t,q,color=color,label=label)
                    axes[i,0].plot(t,reference,color=color,ls=':',lw=.7)
                    axes[i,1].plot(t,v,color=color,label=label,lw=.8)
                    axes[i,2].plot(t,reference-q,color=color,label=label,lw=.8)
                    if label=='PID':
                        axes[i,3].plot(t,a('requested_effort'),color='0.6',lw=.6,label='PID unsaturated request')
                        axes[i,3].plot(t,a('commanded_effort'),color='tab:orange',lw=.8,label='PID written effort')
                limits=list(context['description']['robot']['manipulator']['joints'].values())[i]
                for s in [-1,1]:
                    axes[i,1].axhline(s*limits['velocity'],color='red',ls='--',lw=.7)
                    axes[i,3].axhline(s*limits['effort'],color='red',ls='--',lw=.7)
                axes[i,3].set_yscale('symlog',linthresh=1.)
                axes[i,3].text(.98,.98,'Native motor torque: unavailable',
                    transform=axes[i,3].transAxes,fontsize=8,ha='right',va='top',
                    bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'})
                for j,unit in enumerate(['q / dotted reference (rad)','physical dq (rad/s)','reference error (rad)','effort command (Nm; symlog)']):
                    axes[i,j].set_ylabel(name+'\n'+unit,fontsize=8)
                    axes[i,j].grid(alpha=.2)
                    if i==0:axes[i,j].legend(fontsize=7)
            for ax in axes[-1]:ax.set_xlabel('Simulation source time (s), including startup')
            fig.suptitle(f'{predictor.upper()} seed {seed}: effort PID versus governed effort-limited motor')
            fig.tight_layout()
            file=args.output/f'{predictor}_{seed}_comparison.png'
            fig.savefig(file,dpi=140);plt.close(fig)
            case['plot']=str(file)
            print(predictor,seed,'safe=',after['physical']['accepted'],flush=True)
    write_json(args.output/'summary.json',evidence)


if __name__=='__main__':main()
