"""Recompute phase/physical evidence and figures from immutable M08 run artifacts."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from ament_index_python.packages import get_package_share_directory

from robot_configuration.yaml_loader import load_yaml
from robot_experiments.metrics.actuator_trace import analyze_trace
from robot_experiments.metrics.manipulation import analyze_phases
from robot_experiments.reporting import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', type=Path, required=True, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    settings = load_yaml(Path(get_package_share_directory('robot_experiments'))/
                         'config/manipulation_analysis.yaml')
    if set(settings) != {'max_gap','max_skew','overhead_region','phase_reference'}:
        raise ValueError('unknown/missing analysis settings')
    phase_ref = settings['phase_reference']
    if (not isinstance(phase_ref,dict) or set(phase_ref)!={'package','path'}
            or any(not isinstance(v,str) or not v.strip() for v in phase_ref.values())):
        raise ValueError('phase_reference requires only nonempty package/path strings')
    environment = load_yaml(Path(get_package_share_directory(phase_ref['package']))/phase_ref['path'])
    phases = next(f['parameters']['schedule'] for f in environment['environment']['acceleration_fields']
                  if f['type']=='base_relative_guidance')
    windows = []
    start = 0.
    for phase in phases:
        windows.append(dict(name=phase['name'],start=start,end=start+phase['duration']))
        start += phase['duration']
    result = {'studies':[str(p.resolve()) for p in args.study],'analysis':settings,'phases':phases,'runs':{}}
    run_paths = sorted(run for study in args.study for run in (study/'runs').iterdir())
    for run in run_paths:
        if not (run/'status.json').exists():
            continue
        status = json.loads((run/'status.json').read_text())
        if status['execution_status']!='completed':
            result.setdefault('incomplete_runs',[]).append({'path':str(run),
                'execution_status':status['execution_status'],'error':status.get('error')})
            continue
        resolved = json.loads((run/'resolved_configuration.json').read_text())
        streams = {}
        for path in (run/'telemetry').glob('*.jsonl'):
            if path.stem!='events':
                streams[path.stem]=[json.loads(line) for line in path.read_text().splitlines()]
        valid = [r['source_time'] for r in streams.get('control',[]) if r['data'].get('valid')]
        active = min(valid) if valid else status['window']['window_start']
        context = resolved['context']
        # Compare the actual scheduled experiment, not a readiness-dependent hold tail.
        analysis_end = min(start, status['window']['window_end'])
        analysis = analyze_phases(streams,context,windows,active,analysis_end,
            max_gap=settings['max_gap'], max_skew=settings['max_skew'],
            overhead_region=settings['overhead_region'])
        physics = run/'physics/actuator.csv'
        analysis['physical'] = analyze_trace(physics,context,active) if physics.exists() else None
        analysis['analysis_window_policy'] = 'first valid control through min(schedule end, recorded end)'
        analysis['m08'] = json.loads((run/'metrics.json').read_text())
        analysis['metadata'] = json.loads((run/'metadata.json').read_text())
        analysis['runtime_errors'] = [line for line in (run/'launch.log').read_text().splitlines()
                                      if '[ERROR]' in line or 'Traceback' in line]
        write_json(args.output/(run.name+'.json'),analysis)
        result['runs'][run.name] = {k:v for k,v in analysis.items() if k!='series'}
        plot(run.name, analysis, streams, context, args.output, phases)
        print(run.name,'physics=',(analysis.get('physical') or {}).get('accepted'),flush=True)
    write_json(args.output/'summary.json',result)


def plot(name, analysis, streams, context, output, phases):
    """Use ordinary scientific plots; raw source-time data stays independently readable."""
    series = analysis['series']
    if not series:
        return
    # Field names follow the pure metric record contract.
    t=np.array([r['time'] for r in series])
    positions=np.array([r.get('base_relative_position') or [np.nan]*3 for r in series])
    gaps=np.r_[False,np.diff(t)>analysis['thresholds']['max_gap_s']]
    positions[gaps]=np.nan
    fig=plt.figure(figsize=(8,6)); ax=fig.add_subplot(projection='3d')
    ax.plot(*positions.T,lw=1); ax.set(xlabel='world-aligned Δx [m]',ylabel='Δy [m]',zlabel='height above base [m]',title=name)
    fig.tight_layout();fig.savefig(output/(name+'_trajectory.png'),dpi=140);plt.close(fig)
    fig,axes=plt.subplots(4,2,figsize=(13,13),sharex=True)
    plots=[('rho_m','horizontal radius [m]'),('height_m','height above base [m]'),
           ('relative_azimuth_rad','relative azimuth [rad]'),('reach_m','shoulder-camera reach [m]'),
           ('base_yaw_unwrapped_rad','physical base yaw [rad]'),('arm_yaw_rad','arm yaw [rad]'),
           ('distance_error_m','absolute distance error [m]'),('pointing_error_rad','pointing error [rad]')]
    boundaries=np.cumsum([p['duration'] for p in phases])
    for ax,(key,label) in zip(axes.flat,plots):
        values=np.array([r.get(key) if r.get(key) is not None else np.nan for r in series])
        values[gaps]=np.nan
        ax.plot(t,values,lw=.9,label='measured');ax.set_ylabel(label);ax.grid(alpha=.2)
        if not name.startswith('A_') and key in ['rho_m','height_m']:
            from robot_simulation.disturbances.phase_schedule import PhaseSchedule
            schedule=PhaseSchedule(phases)
            column=0 if key=='rho_m' else 1
            ax.plot(t,[schedule.values(time)[column] for time in t],ls='--',lw=.8,
                    label='guidance preference')
            ax.legend(fontsize=7)
        if key=='relative_azimuth_rad':
            for bound in [-np.pi/2,np.pi/2]:ax.axhline(bound,color='red',ls=':',alpha=.4)
        for boundary in boundaries[:-1]:ax.axvline(boundary,color='gray',ls=':')
    for ax in axes[-1]:ax.set_xlabel('simulation source time [s]')
    fig.suptitle(name);fig.tight_layout();fig.savefig(output/(name+'_geometry_tracking.png'),dpi=140);plt.close(fig)
    from robot_experiments.metrics.joint_diagnostics import named_joint_series
    fig,axes=plt.subplots(4,2,figsize=(13,12),sharex=True)
    limits=context['description']['robot']['manipulator']['joints']
    for row,(joint,physical) in enumerate(zip(context['joint_names'],limits.values())):
        jt,q,dq,_=named_joint_series(streams['joints'],joint)
        use=jt <= analysis['windows']['active']['window_s'][1]
        jt,q,dq=jt[use],q[use],dq[use]
        q=q.copy();dq=dq.copy()
        missing=np.r_[False,np.diff(jt)>analysis['thresholds']['max_gap_s']]
        q[missing]=np.nan;dq[missing]=np.nan
        axes[row,0].plot(jt,q,lw=.8);axes[row,1].plot(jt,dq,lw=.8)
        for bound in [physical['lower'],physical['upper']]:axes[row,0].axhline(bound,color='r',ls=':')
        for bound in [-physical['velocity'],physical['velocity']]:axes[row,1].axhline(bound,color='r',ls=':')
        axes[row,0].set_ylabel(joint+' [rad]');axes[row,1].set_ylabel('[rad/s]')
        for ax in axes[row]:
            for boundary in boundaries[:-1]:ax.axvline(boundary,color='gray',ls=':')
            ax.grid(alpha=.2)
    axes[-1,0].set_xlabel('simulation source time [s]');axes[-1,1].set_xlabel('simulation source time [s]')
    fig.suptitle(name+' — raw named JointState feedback; red=description bounds')
    fig.tight_layout();fig.savefig(output/(name+'_joints.png'),dpi=140);plt.close(fig)
    physical=analysis.get('physical')
    if physical:
        with Path(physical['path']).open() as stream:
            raw=list(csv.DictReader(stream))
        fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
        for joint,bounds in zip(context['joint_names'],limits.values()):
            samples=[r for r in raw if r['name']==joint]
            pt=np.array([float(r['time']) for r in samples])
            pq=np.array([float(r['q_post']) for r in samples])
            pv=np.array([float(r['v_post']) for r in samples])
            axes[0].plot(pt,np.abs(pv)/bounds['velocity'],label=joint,lw=.6)
            excess=np.maximum(0.,np.maximum(bounds['lower']-pq,pq-bounds['upper']))
            axes[1].plot(pt,excess,label=joint,lw=.6)
        axes[0].axhline(1.,color='red',ls=':')
        axes[1].axhline(context['physical_numerical_tolerance'],color='red',ls=':')
        axes[0].set_ylabel('|physical qdot| / limit')
        axes[1].set(ylabel='physical position excess [rad]',xlabel='simulation source time [s]')
        axes[0].legend(fontsize=7);fig.suptitle(name+' — every recorded physics step')
        fig.tight_layout();fig.savefig(output/(name+'_physical_limits.png'),dpi=140);plt.close(fig)


if __name__=='__main__':
    main()
