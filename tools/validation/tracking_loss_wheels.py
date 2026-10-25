"""Read-only encoder/ideal-odometry/physical-base twist consistency audit."""
import argparse
import csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from tracking_loss_geometry import History,wrap
from tracking_loss_analysis import read,stats

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();c=json.loads((a.run/'resolved_configuration.json').read_text())['context'];robot=c['robot'];wheel=c['description']['robot']['wheels'];radius=wheel['radius'];separation=2*wheel['y_offset'];drive=robot['differential_drive']
 joints=History(read(a.run/'telemetry/joints.jsonl'));odom=History(read(a.run/'telemetry/odometry.jsonl'));base=read(a.run/'telemetry/base_truth.jsonl');rows=[]
 for before,now in zip(base,base[1:]):
  t=now['source_time'];dt=t-before['source_time'];j=joints.at(t,.08);o=odom.at(t,.08)
  if not 0<dt<=.15 or j is None or o is None:continue
  names=j['data']['names'];v=np.array(j['data']['velocity']);left=np.mean([v[names.index(n)] for n in drive['left_joints']]);right=np.mean([v[names.index(n)] for n in drive['right_joints']]);enc_v=radius*(left+right)/2;enc_w=radius*(right-left)/separation
  R=Rotation.from_quat(now['data']['orientation']).as_matrix();Rp=Rotation.from_quat(before['data']['orientation']).as_matrix();delta=np.array(now['data']['position'])-np.array(before['data']['position']);body=R.T@delta/dt;physical_w=wrap(np.arctan2(R[1,0],R[0,0])-np.arctan2(Rp[1,0],Rp[0,0]))/dt
  rows.append({'t':t,'encoder_v':enc_v,'encoder_w':enc_w,'odom_v':o['data']['linear_velocity'][0],'odom_w':o['data']['angular_velocity'][2],'physical_fd_v':body[0],'physical_fd_lateral':body[1],'physical_fd_w':physical_w,'gt_twist_v':now['data']['linear_velocity'][0],'gt_twist_w':now['data']['angular_velocity'][2]})
 out={'radius_m':radius,'separation_m':separation,'policy':'Configured mean left/right wheel speeds, ideal differential-drive formula; physical forward/lateral/yaw velocities from adjacent world poses, ≤0.15s gaps; causal encoder/odom ≤0.08s. This measures rolling-constraint inconsistency, not individual contact forces.','stages':{}}
 for lo,hi in [(0,10),(10,20),(20,40),(40,90),(0,90)]:
  s=[r for r in rows if lo<=r['t']<hi];m={}
  for x,y in [('encoder_w','odom_w'),('encoder_w','physical_fd_w'),('encoder_v','physical_fd_v'),('gt_twist_w','physical_fd_w'),('gt_twist_v','physical_fd_v')]:m[x+'__'+y]=stats([abs(r[x]-r[y]) for r in s])
  m['physical_lateral_speed_abs']=stats([abs(r['physical_fd_lateral']) for r in s]);m['turning_samples']=int(sum(abs(r['encoder_w'])>.1 for r in s))
  out['stages'][f'{lo}-{hi}']=m
 a.output.mkdir(parents=True,exist_ok=False);(a.output/'summary.json').write_text(json.dumps(out,indent=2)+'\n')
 with (a.output/'series.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 print(json.dumps(out['stages']['0-90'],indent=2))
if __name__=='__main__':main()
