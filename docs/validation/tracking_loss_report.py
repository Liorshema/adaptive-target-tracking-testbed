"""Assemble only physically safe, sustained-control diagnostic comparisons."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from joint_limit_recovery_analysis import telemetry_summary

def main():
 p=argparse.ArgumentParser();p.add_argument('--case',nargs=3,action='append',metavar=('NAME','RUN','ANALYSIS'),required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);cases={}
 for name,run,analysis in a.case:
  run=Path(run);analysis=Path(analysis);r=json.loads((run/'diagnostic_result.json').read_text());s=json.loads((analysis/'summary.json').read_text());c=r.get('control_continuity',{})
  if r.get('error') or not r.get('physical',{}).get('accepted') or not c.get('valid_samples') or c.get('max_invalid_duration_s',float('inf'))>=1 or c.get('max_sample_gap_s',float('inf'))>=1:raise ValueError(f'{name}: incomplete evidence cannot be ranked')
  w=r['window'];health=telemetry_summary(run/'telemetry',w['window_start'],w['window_end']);audits=[]
  if (run/'oracle.jsonl').exists():
   with (run/'oracle.jsonl').open() as stream:
    for line in stream:
     x=json.loads(line)
     if x['kind']=='interface_audit':audits.append(x)
   if any(v['publishers']!=['tracking_loss_diagnostic_oracle'] or 'control_node' not in v['subscribers'] for row in audits for v in row['topics'].values()):raise ValueError('oracle topic ownership not established')
   if not audits:raise ValueError('oracle has no interface audits')
  cases[name]={'run':str(run),'analysis':str(analysis),'intervention':r['intervention'],'stages':s['stages'],'oscillation':s['oscillation'],'prediction_horizon_error_m':s['prediction_horizon_error_m'],'control':c,'physical':r['physical'],'health':health,'timing':r['tracking']['metrics'],'cleanup':r['cleanup'],'interface_audit_samples':len(audits)}
 summary={'policy':'Seed 42, Scenario D; matched initial/exogenous settings, reactive target paths differ. One completed replicate per intervention; no confidence intervals or algorithm ranking.','cases':cases}
 (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 names=list(cases);x=np.arange(len(names));fig,axes=plt.subplots(1,3,figsize=(13,4.5))
 for ax,key,label in [(axes[0],'physical_distance_error_m','Distance MAE (m)'),(axes[1],'physical_pointing_deg','Pointing MAE (deg)')]:
  ax.bar(x,[cases[n]['stages']['0-90'][key]['mean'] for n in names]);ax.set_ylabel(label)
 axes[2].bar(x,[cases[n]['stages']['0-90']['compliance']['valid_time_percent'] for n in names]);axes[2].set_ylabel('Joint distance + pointing compliance (%)');axes[2].set_ylim(0,100)
 for ax in axes:ax.set_xticks(x,names,rotation=30,ha='right');ax.grid(axis='y',alpha=.25)
 fig.suptitle('Scenario D / seed 42: diagnostic interventions, valid observed time\nTarget oracle uses current/past truth and zero lookahead; reactive paths differ')
 fig.tight_layout();fig.savefig(a.output/'interventions.png',dpi=180);fig.savefig(a.output/'interventions.svg');plt.close(fig)
 for n,c in cases.items():
  s=c['stages']['0-90'];print(n,'distance',s['physical_distance_error_m']['mean'],'pointing',s['physical_pointing_deg']['mean'],'compliance',s['compliance']['valid_time_percent'],'coverage',s['compliance']['coverage_percent'])
if __name__=='__main__':main()
