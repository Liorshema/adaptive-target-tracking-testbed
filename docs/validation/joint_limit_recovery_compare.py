"""Build a compact comparison from immutable recovery and historical challenge runs."""
import argparse
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from robot_configuration.yaml_loader import load_yaml
from robot_experiments.metrics.manipulation import analyze_phases
from robot_experiments.reporting import write_json
from joint_limit_recovery_analysis import control_continuity


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    settings=load_yaml(Path(get_package_share_directory('robot_experiments'))/'config/manipulation_analysis.yaml')
    old_root=Path('results/reactive_challenge_verified_report')
    output={'studies':[str(p) for p in args.study],'runs':{},'excluded_attempts':[],
            'metric_policy':'physical FK, first-valid-control through 40s scheduled policy; independent launch origins, not identical trajectories'}
    for run in (p for study in args.study for p in sorted(study.iterdir())):
        if not (run/'recovery_result.json').exists():continue
        result=json.loads((run/'recovery_result.json').read_text())
        if 'window' not in result:
            output['excluded_attempts'].append({'path':str(run),'error':result.get('error')})
            continue
        window=result['window']
        continuity=control_continuity(run/'telemetry/control.jsonl',
            window['window_start'],window['window_end'])
        if (not continuity['valid_samples'] or continuity['max_invalid_duration_s']>=1.
                or continuity['max_sample_gap_s']>=1.):
            output['excluded_attempts'].append({'path':str(run),'control_continuity':continuity})
            continue
        resolved=json.loads((run/'resolved_configuration.json').read_text())
        streams={p.stem:[json.loads(line) for line in p.open()] for p in (run/'telemetry').glob('*.jsonl') if p.stem!='events'}
        end=result['window']['window_end'];active=result['first_valid_control_time']
        analysis=analyze_phases(streams,resolved['context'],
            [{'name':'scheduled','start':0.,'end':40.},{'name':'extended','start':40.,'end':end}],
            active,end,max_gap=settings['max_gap'],max_skew=settings['max_skew'],overhead_region=settings['overhead_region'])
        analysis.pop('series',None)
        old_path=old_root/(run.name+'.json')
        old=json.loads(old_path.read_text()) if old_path.exists() else None
        row={'path':str(run),'control_continuity':continuity,
             'before':old['windows']['active'] if old else None,'after':analysis,
             'before_timing':old.get('m08',{}).get('metrics',{}) if old else None,
             'physical':result.get('physical'),'deadlocks':result.get('deadlock_analysis'),
             'recoveries':result.get('recovery_analysis'),'timing':result['tracking'].get('metrics',{}),
             'window':result['window']}
        output['runs'][run.name]=row
        write_json(args.output/(run.name+'.json'),row)
    write_json(args.output/'summary.json',output)
    for name,row in output['runs'].items():
        scheduled=row['after']['windows']['scheduled']
        print(name, 'distance_mean_max_m', scheduled['tracking_distance'].get('mean'),
              scheduled['tracking_distance'].get('max'), 'pointing_mean_max_deg',
              scheduled['pointing_error'].get('mean',0)*180/math.pi,
              scheduled['pointing_error'].get('max',0)*180/math.pi, flush=True)


if __name__=='__main__':main()
