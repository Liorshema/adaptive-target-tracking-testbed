"""Experiment influences freeze beside the same immutable Library model."""
from pathlib import Path
from copy import deepcopy
import pytest
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.dynamics_library import content_hash

@pytest.fixture
def backend(tmp_path):
    src=Path(__file__).parents[2]
    catalog=ExperimentCatalog(tmp_path/'catalog.yaml',share=lambda p:src/p);catalog.initialize()
    definition=deepcopy(catalog.get('EXP-NOMINAL')['definition'])
    variant=next(v for v in definition['experiment']['variants'] if v['name']=='imm')
    from robot_experiments.configuration import resolve_system_resource
    from robot_experiments.composition import materialize_resource
    from robot_configuration.yaml_loader import load_yaml
    manifest_path=resolve_system_resource(variant['system_config'],tmp_path,catalog.share)
    manifest=load_yaml(manifest_path)
    deployment=load_yaml(materialize_resource(manifest['simulation']['config'],manifest_path.parent,catalog.share))
    deployment['tracking']['pose_source']='physical'
    variant['system_config']={'base':variant['system_config'],'overrides':{
        'simulation':{'config':{'document':deployment}}}}
    from robot_experiments.authoring import _freeze
    variant['system_config']=_freeze(str(resolve_system_resource(variant['system_config'],tmp_path,catalog.share)))
    definition['experiment']['variants']=[variant]
    catalog.create('EXP-PHYSICAL','Physical tracking TF',definition)
    return ExperimentBackend(tmp_path/'workspace',catalog=catalog)

def draft(backend):
    d=backend.dynamics.templates()['attractor']['definition']
    backend.dynamics.create('DYN-ATTRACTOR','3D Attractor Dynamics',d)
    spec=backend.new_draft('EXP-ATTRACTOR','attractor','EXP-PHYSICAL',base_variant='imm')
    spec=backend.draft_with_dynamics(spec,'DYN-ATTRACTOR',revision=1)
    spec['experiment']['resources']={'environment':'environment_none'}
    spec['experiment']['execution']={'duration':3.,'settling_time':1.,'seeds':[42]}
    spec['experiment']['runtime']={'enable_actuation':False}
    return spec

def test_off_on_save_duplicate_and_frozen_profile_do_not_revise_library(backend,monkeypatch):
    spec=draft(backend);library=deepcopy(backend.dynamics.get('DYN-ATTRACTOR',1))
    a=backend.draft_with_robot_response(spec,False)
    b=backend.draft_with_robot_response(a,True)
    left=a['experiment']['parameters']['target_dynamics'];right=b['experiment']['parameters']['target_dynamics']
    assert left['model']==right['model']==library['model']
    assert a['experiment']['dynamics_source']==b['experiment']['dynamics_source']
    equivalent=deepcopy(right);equivalent['influences'][0]['enabled']=False
    assert left==equivalent
    saved=backend.save_experiment(a)
    copy=backend.new_draft('EXP-ATTRACTOR-RESPONSE','attractor response','EXP-ATTRACTOR')
    from robot_simulation.dynamics import derivative_influences as owner
    changed=owner.robot_response_profile();changed['parameters']['A_max']['value']=.9
    monkeypatch.setattr(owner,'robot_response_profile',lambda:changed)
    enabled=backend.draft_with_robot_response(copy,True)
    assert enabled['experiment']['parameters']['target_dynamics']['influences'][0]['parameters']['A_max']['value']==.3
    backend.save_experiment(enabled)
    resolved=backend.catalog.get('EXP-ATTRACTOR-RESPONSE')
    assert resolved['sha256']!=saved['sha256']
    assert backend.dynamics.get('DYN-ATTRACTOR',1)==library
    assert len(backend.dynamics.revisions('DYN-ATTRACTOR'))==1
    assert backend.catalog.get('EXP-ATTRACTOR',1)==saved

def test_unsupported_mapping_and_nonboolean_opt_in_are_rejected(backend):
    spec=draft(backend)
    for bad in [1,'yes',None]:
        with pytest.raises(ValueError):backend.draft_with_robot_response(spec,bad)
    model=deepcopy(spec['experiment']['parameters']['target_dynamics']['model'])
    model['outputs']['position']='2*[x,y,z]'
    detail=backend.describe_robot_response(model)
    assert not detail['available'] and 'position' in detail['reason'].lower()

def test_invalid_live_context_does_not_trap_saved_explicit_profile_on(backend):
    spec=backend.draft_with_robot_response(draft(backend),True)
    spec['experiment']['base_case']='EXP-NOMINAL'
    spec['experiment']['base_revision']=3
    with pytest.raises(ValueError,match='physical world tracking TF'):
        backend.preview_experiment(spec)
    off=backend.draft_with_robot_response(spec,False)
    assert off['experiment']['parameters']['target_dynamics']['influences'][0]['enabled'] is False
    assert backend.validate_experiment(off)['valid']
