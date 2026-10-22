"""Replay accepted forecast epochs; verify output identity before attributing stages."""
import argparse,json
from pathlib import Path
import numpy as np
from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput
from tracking_loss_analysis import read,interpolate,stats
from tracking_loss_geometry import History

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();resolved=json.loads((a.run/'resolved_configuration.json').read_text())
 resources=resolved['owner_snapshot']['resources'];config_path=next(path for path,v in resources.items() if '/robot_prediction/' in path and 'prediction' in v.get('resolved',{}) and 'imm' in v.get('resolved',{}))
 system=PredictionSystemFactory(PredictionConfig(config_path)).create(predictor_type='imm',adaptation_type='residual',fusion_type='weighted_correction')
 estimates={round(r['source_time'],9):r for r in read(a.run/'telemetry/target_estimate.jsonl')};forecasts=read(a.run/'telemetry/prediction.jsonl');truth=History(read(a.run/'telemetry/target_truth.jsonl'));previous=None;delta=[];errors={name:{str(h):[] for h in [.1,.3,1.]} for name in ['imm','fused','cv']};missing=0;corrections=[]
 for r in forecasts:
  t=r['source_time'];e=estimates.get(round(t,9))
  if e is None:missing+=1;continue
  state=np.array(e['data']['mean']);d=r['data'];times=np.array(d['timestamps']);dt=times[1]-times[0]
  if previous is not None:system.update(state,observation_dt=t-previous,observation_timestamp=t)
  result=system.predict(PredictionInput(state=state,timestamp=t,dt=dt,horizon=len(times)));previous=t
  delta.append(np.max(np.abs(result.states-np.array(d['states']))));baseline=system._last_baseline_prediction
  corrections.append(np.linalg.norm(result.states[0,:3]-baseline.states[0,:3]))
  for h in [.1,.3,1.]:
   expected=interpolate(truth,t+h)
   if expected is None:continue
   for name,states in [('imm',baseline.states),('fused',result.states),('cv',state[:3]+(times-t)[:,None]*state[3:6])]:
    prediction=np.array([np.interp(t+h,times,states[:,k]) for k in range(3)]);errors[name][str(h)].append(np.linalg.norm(prediction-expected))
 out={'run':str(a.run),'config':config_path,'accepted_forecast_epochs':len(delta),'missing_estimate_epochs':missing,'recorded_vs_replay_max_abs_state_difference':max(delta,default=None),'output_identity_established':bool(delta) and bool(max(delta)<1e-8),'horizon_error_m':{name:{h:stats(v) for h,v in horizons.items()} for name,horizons in errors.items()},'position_correction_m':stats(corrections),'interpretation':'Only attribute recorded pipeline stages if output identity is established. CV/uncorrected IMM use same recorded target states; this is offline predictor comparison, not identical closed-loop policy.'}
 a.output.mkdir(parents=True,exist_ok=False);(a.output/'summary.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':main()
