"""Composition and seed materialization contracts, independent of ROS."""
from pathlib import Path
import pytest
import yaml
from robot_experiments.configuration import load_experiment, materialize_run

SRC=Path(__file__).resolve().parents[2]
SHARE=lambda name:SRC/name

def document(tmp_path,**changes):
 data={'schema_version':1,'experiment':{'name':'paired','duration':2.,'settling_time':1.,'seeds':[42],
 'variants':[{'name':'imm','system_config':{'package':'robot_bringup','path':'config/system_nominal.yaml'}}],
 'metrics':['tracking_distance','pointing_error','prediction_rmse','joint_speed_violation','realtime_factor'],
 'output':{'directory':'results/experiments'},'runtime':{'enable_actuation':True}}}
 data['experiment'].update(changes)
 p=tmp_path/'experiment.yaml';p.write_text(yaml.safe_dump(data));return p

def test_real_owner_preflight_and_seed_reaches_measurement_config(tmp_path):
 spec=load_experiment(document(tmp_path),SHARE)
 run=tmp_path/'run';run.mkdir()
 result=materialize_run(spec['variants'][0],917,run,SHARE,spec['runtime'])
 assert result['plan']['identities']['predictor'].endswith('.IMMPredictor')
 context=result['context']
 assert context['topics']['base_truth']=='/simulation/robot/ground_truth'
 sim=yaml.safe_load(result['simulation_path'].read_text())
 measured=yaml.safe_load(Path(sim['simulation']['measurement_config']).read_text())
 assert measured['measurements']['target_position']['random_seed']==917
 assert yaml.safe_load((SRC/'robot_simulation/config/measurements.yaml').read_text())['measurements']['target_position']['random_seed']==42
 assert result['context']['description']==yaml.safe_load((SRC/'robot_description/config/robot/robot.yaml').read_text())
 assert result['plan']['runtime']['enable_actuation'] is True
 assert result['snapshot']['seed_consumers']
 assert result['plan']['description_path'].is_relative_to(run)

@pytest.mark.parametrize('change',[{'duration':True},{'duration':0},{'settling_time':-1},{'seeds':[1,1]},
 {'seeds':[True]},{'metrics':['imaginary']},{'variants':[{'name':'a','system_config':'missing.yaml'}]},
 {'runtime':{'enable_actuation':'yes'}},{'execution':{'wall_timeout':0}},{'typo':1}])
def test_invalid_configs_fail_early(tmp_path,change):
 with pytest.raises(ValueError):load_experiment(document(tmp_path,**change),SHARE)

def test_declaring_file_relative_resources_survive_inheritance(tmp_path):
 base=tmp_path/'parent';base.mkdir()
 source=document(base)
 source_data=yaml.safe_load(source.read_text())
 source_data['experiment']['variants'][0]['system_config']=str(SRC/'robot_bringup/config/system_cv.yaml')
 source.write_text(yaml.safe_dump(source_data))
 child=tmp_path/'child.yaml';child.write_text('extends: parent/experiment.yaml\nexperiment:\n  seeds: [43]\n')
 spec=load_experiment(child,SHARE)
 assert spec['seeds']==[43]
 assert spec['variants'][0]['plan']['identities']['predictor'].endswith('.ConstantVelocityPredictor')

def test_prediction_horizon_is_part_of_matched_comparison_contract(tmp_path):
 spec=load_experiment(document(tmp_path),SHARE)
 run=tmp_path/'run';run.mkdir()
 result=materialize_run(spec['variants'][0],917,run,SHARE,spec['runtime'])
 assert result['comparison_contract']['prediction_sampling']['horizon']>0
 assert result['comparison_contract']['prediction_sampling']['dt']>0
