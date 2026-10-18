"""Interrupted workflows keep partial evidence and restore private session state."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

from robot_experiments.configuration import load_experiment
from robot_experiments.execution import RunFailure
from robot_experiments import recording,runner

SRC=Path(__file__).resolve().parents[2]
SHARE=lambda name:SRC/name

def test_startup_failure_persists_status_cleanup_and_environment(tmp_path,monkeypatch):
 spec=load_experiment(SRC/'robot_experiments/config/nominal.yaml',SHARE)
 class Recorder:
  def __init__(self,context,path):
   self.streams={};self.availability={};self.events=[];self.closed=False
  def spin_once(self,timeout):pass
  def close(self):self.closed=True
 class Process:
  stopped=0
  def __init__(self,command,env,log):self.log=log
  def start(self):self.log.write_text('failed startup\n');return self
  def stop(self,grace):Process.stopped+=1;return {'survivors':[],'returncode':7}
 def fail(*args,**kwargs):raise RunFailure('readiness_timeout',{'clock':0})
 monkeypatch.setattr(recording,'Recorder',Recorder)
 monkeypatch.setattr(runner,'OwnedLaunch',Process)
 monkeypatch.setattr(runner,'run_window',fail)
 monkeypatch.setattr(runner,'_git',lambda:{'revision':'test'})
 before={k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','ROS_LOG_DIR']}
 result=runner.run_one(spec,spec['variants'][0],917,tmp_path/'run',SHARE)
 status=json.loads((tmp_path/'run/status.json').read_text())
 assert status['execution_status']=='failed' and status['error']['code']=='readiness_timeout'
 assert Process.stopped==1 and status['cleanup']['survivors']==[]
 assert status['acceptance'] is None
 assert (tmp_path/'run/resolved_configuration.json').exists()
 assert result['artifacts']=='runs/run'
 assert {k:os.environ.get(k) for k in before}==before

def test_compute_only_does_not_wait_for_unpublished_commands():
 plan={'runtime':{'publish_commands':False}}
 assert 'command' not in runner.required_streams(plan,False)
 assert 'actuation' not in runner.required_streams(plan,False)
 plan['runtime']['publish_commands']=True
 assert 'command' in runner.required_streams(plan,True)
 assert 'actuation' in runner.required_streams(plan,True)

def test_interrupt_aborts_study_even_with_continue_policy(tmp_path,monkeypatch):
 spec=load_experiment(SRC/'robot_experiments/config/paired_prediction.yaml',SHARE,tmp_path)
 calls=[]
 def interrupted(spec,variant,seed,path,share):
  calls.append(variant['name'])
  return {'variant':variant['name'],'seed':seed,'execution_status':'failed','acceptance':None,
  'comparison_contract':'same','metrics':{},'artifacts':'runs/failure',
  'status':{'error':{'type':'KeyboardInterrupt','code':'interrupted'}}}
 monkeypatch.setattr(runner,'run_one',interrupted)
 _,code=runner.run_experiment(spec,SHARE)
 assert code==1 and calls==['imm']
