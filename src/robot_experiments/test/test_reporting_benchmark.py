"""Immutable evidence directories and honest paired aggregation."""
import json
import pytest
from robot_experiments.reporting import create_study_directory, write_json
from robot_experiments.benchmarking import summarize

def test_collision_safe_directory_and_strict_atomic_json(tmp_path):
 a=create_study_directory(tmp_path,'test');b=create_study_directory(tmp_path,'test')
 assert a!=b and a.is_dir() and b.is_dir()
 write_json(a/'status.json',{'schema_version':1,'value':None})
 assert json.loads((a/'status.json').read_text())['value'] is None
 with pytest.raises(ValueError):write_json(a/'bad.json',{'x':float('nan')})
 assert not (a/'bad.json').exists()

def run(name,seed,value,accepted=True,contract='same'):
 return {'variant':name,'seed':seed,'execution_status':'completed','acceptance':accepted,
 'comparison_contract':contract,'metrics':{'metrics':{'tracking_distance':{'available':True,'mean':value,'unit':'m','sample_count':2}}},'artifacts':name+str(seed)}

def test_matched_seed_aggregation_safety_and_missing_values():
 s=summarize([run('cv',42,2),run('imm',42,1,False),run('cv',43,4),run('imm',43,3)],['cv','imm'],[42,43])
 assert s['comparable'] is True
 assert s['variants']['cv']['metrics']['tracking_distance']['mean']==3
 assert s['variants']['cv']['metrics']['tracking_distance']['sample_count']==2
 assert s['variants']['imm']['unconditional_winner_eligible'] is False
 assert 'winner' not in s

def test_incompatible_scenario_and_missing_pair_are_explicit():
 s=summarize([run('cv',42,1),run('imm',42,2,contract='different')],['cv','imm'],[42])
 assert not s['comparable'] and s['incomparability_reasons']
 s=summarize([run('cv',42,1)],['cv','imm'],[42])
 assert not s['comparable']

def test_prediction_aggregation_uses_rmse_and_preserves_horizons():
 r=run('cv',42,0)
 r['metrics']['metrics']['prediction_rmse']={'available':True,'mean':2.,'rmse':3.,'unit':'m',
 'horizons':{'0.1':{'available':True,'mean':1.,'rmse':1.5,'unit':'m','sample_count':5}}}
 s=summarize([r],['cv'],[42])
 assert s['variants']['cv']['metrics']['prediction_rmse']['mean']==3.
 assert s['variants']['cv']['metrics']['prediction_rmse']['source_statistic']=='rmse'
 assert s['variants']['cv']['prediction_horizons']['0.1']['mean']==1.5
