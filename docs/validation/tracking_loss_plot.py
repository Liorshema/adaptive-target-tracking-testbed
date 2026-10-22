"""Export standalone engineering plots from immutable diagnostic CSVs."""
import argparse
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def table(p):
 with p.open() as f:rows=list(csv.DictReader(f))
 return {k:np.array([float(r[k]=='True') if r[k] in ['True','False'] else (float(r[k]) if r[k] else np.nan) for r in rows]) for k in rows[0]}
def main():
 p=argparse.ArgumentParser();p.add_argument('--analysis',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 s=table(a.analysis/'series.csv');e=table(a.analysis/'robot_estimate_localization.csv');o=table(a.analysis/'odometry_localization.csv')
 fig,axes=plt.subplots(3,2,figsize=(14,11),sharex=True)
 ax=axes[0,0];ax.plot(o['t'],o['translation_m'],label='raw wheel odometry');ax.plot(e['t'],e['translation_m'],label='final robot estimate');ax.set_ylabel('Planar pose error (m)');ax.legend()
 ax=axes[0,1];ax.plot(o['t'],o['yaw_deg'],label='odometry');ax.plot(e['t'],e['yaw_deg'],label='estimate');ax.set_ylabel('Signed heading error (deg)');ax.legend()
 ax=axes[1,0];ax.plot(s['t'],s['physical_distance_error_m'],label='physical FK + target truth');ax.plot(s['t'],s['internal_distance_error_m'],label='controller internal');ax.set_ylabel('Signed radius error (m)');ax.legend()
 ax=axes[1,1];ax.plot(s['t'],s['physical_pointing_deg'],label='physical optical +X');ax.plot(s['t'],s['internal_pointing_deg'],label='controller internal');ax.set_ylabel('Pointing error (deg)');ax.legend()
 ax=axes[2,0];ax.plot(s['t'],s['command_0'],label='base v (m/s)');ax.plot(s['t'],s['command_1'],label='base yaw (rad/s)');ax.set_ylabel('Safe control command');ax.legend()
 ax=axes[2,1];ax.plot(s['t'],s['q_1'],label='measured shoulder');ax.plot(s['t'],s['q_3'],label='measured wrist');ax.set_ylabel('Physical joint position (rad)');ax.legend()
 for ax in axes.flat:
  ax.grid(alpha=.25);ax.set_xlim(0,90)
  for x in [10,20,40]:ax.axvline(x,color='gray',ls=':',lw=.8)
 for ax in axes[-1]:ax.set_xlabel('Simulation source time (s)')
 fig.suptitle('Scenario D, seed 42: physical tracking versus internal model\nCausal source alignment ≤80 ms; spawn transform applied; gaps not interpolated')
 fig.tight_layout();fig.savefig(a.output/'tracking_loss.png',dpi=180);fig.savefig(a.output/'tracking_loss.svg');plt.close(fig)
if __name__=='__main__':main()
