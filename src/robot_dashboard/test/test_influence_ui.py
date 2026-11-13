"""Thin two-checkbox UI over frozen owner configuration."""
from copy import deepcopy
from pathlib import Path
import pytest
from robot_dashboard.app import component_cards
from robot_dashboard.dynamics_forms import hydrate_target_pin
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog

@pytest.fixture
def state_backend(tmp_path):
    src=Path(__file__).parents[2]
    c=ExperimentCatalog(tmp_path/'catalog.yaml',share=lambda p:src/p);c.initialize()
    definition=deepcopy(c.get('EXP-NOMINAL')['definition'])
    variant=next(v for v in definition['experiment']['variants'] if v['name']=='imm')
    from robot_experiments.configuration import resolve_system_resource
    from robot_experiments.composition import materialize_resource
    from robot_configuration.yaml_loader import load_yaml
    manifest_path=resolve_system_resource(variant['system_config'],tmp_path,c.share)
    manifest=load_yaml(manifest_path)
    deployment=load_yaml(materialize_resource(manifest['simulation']['config'],manifest_path.parent,c.share))
    deployment['tracking']['pose_source']='physical'
    variant['system_config']={'base':variant['system_config'],'overrides':{
        'simulation':{'config':{'document':deployment}}}}
    from robot_experiments.authoring import _freeze
    variant['system_config']=_freeze(str(resolve_system_resource(variant['system_config'],tmp_path,c.share)))
    definition['experiment']['variants']=[variant]
    c.create('EXP-PHYSICAL','Physical tracking TF',definition)
    b=ExperimentBackend(tmp_path/'data',catalog=c)
    b.dynamics.create('DYN-ATTRACTOR','3D Attractor Dynamics',b.dynamics.templates()['attractor']['definition'])
    d=b.draft_with_dynamics(b.new_draft('EXP-UI','attractor','EXP-PHYSICAL',base_variant='imm'),'DYN-ATTRACTOR',revision=1)
    d['experiment']['resources']={'environment':'environment_none'}
    p=b.preview_experiment(d)
    return {'spec':d,'preview':p,'components':p['components'],'inherited_components':deepcopy(p['components']),'stage':'Draft'},b

def walk(node):
    if isinstance(node,(list,tuple)):
        for child in node:yield from walk(child)
    elif hasattr(node,'to_plotly_json'):
        yield node
        yield from walk(getattr(node,'children',None))


def test_scaled_three_state_switch_hydrate_validate_save_reload_preserves_history(state_backend):
    """Actual Lab callbacks preserve an OFF incompatible frozen profile and its pin."""
    import yaml
    from robot_dashboard.app import create_app
    from robot_dashboard.dynamics_forms import choose_target_model
    from robot_dashboard.forms import begin_draft

    state, backend = state_backend
    state = hydrate_target_pin(state, backend)
    disabled = deepcopy(state['spec']['experiment']['parameters']['target_dynamics']['influences'])
    fixture = Path(__file__).parents[2] / 'robot_simulation/test/model_fixtures/halvorsen.yaml'
    model = yaml.safe_load(fixture.read_text())
    library = backend.dynamics.create('DYN-HALVORSEN', 'Halvorsen', model)
    old_library = deepcopy(backend.dynamics.get('DYN-ATTRACTOR', 1))
    source = deepcopy(backend.catalog.get('EXP-PHYSICAL', 1))
    state = choose_target_model(backend, state, 'library:DYN-HALVORSEN:1')
    # Reproduce the existing saved disabled snapshot, rather than removing it.
    state['spec']['experiment']['parameters']['target_dynamics']['influences'] = disabled
    state['preview'] = backend.preview_experiment(state['spec'])
    state['components'] = state['preview']['components']
    state = hydrate_target_pin(state, backend)
    state['expected_revision'] = None
    assert not state['robot_response']['available']
    assert not state['robot_response']['enabled']
    assert 'position' in state['robot_response']['reason'].lower()
    assert state['spec']['experiment']['parameters']['target_dynamics']['influences'] == disabled
    checkbox = next(c for c in walk(component_cards(backend, state))
                    if getattr(c, 'id', None) == {'type': 'influence', 'role': 'robot_response'})
    assert checkbox.value == [] and checkbox.options[0]['disabled']
    app = create_app(backend)
    key, callback = next((k, v) for k, v in app.callback_map.items()
                         if k.startswith('..builder-state.data'))

    def action(state, trigger):
        ex = state['spec']['experiment']
        inputs = [None] * 5 + [[], [], [], .4, '[42]', [], None, [[]]]
        states = [[], [], ex['base_case'], ex['base_revision'], 'create', ex['id'], ex['name'], state]
        response = app.server.test_client().post('/_dash-update-component', json={
            'output': key,
            'outputs': [{'id': o.component_id, 'property': o.component_property}
                        for o in callback['output']],
            'inputs': [{**m, 'value': v} for m, v in zip(callback['inputs'], inputs)],
            'state': [{**m, 'value': v} for m, v in zip(callback['state'], states)],
            'changedPropIds': [trigger + '.n_clicks']})
        assert response.status_code == 200, response.get_data(as_text=True)
        return response.get_json()['response']['builder-state']['data']

    state = action(state, 'validate-draft')
    assert state['stage'] == 'Validated', state['message']
    state = action(state, 'save-draft')
    assert state['stage'] == 'Saved', state['message']
    saved = deepcopy(backend.catalog.get('EXP-UI', 1))
    variant = saved['definition']['experiment']['variants'][0]['name']
    reloaded = hydrate_target_pin(begin_draft(backend, 'EXP-UI', 1, variant, 'revise'), backend)
    actual = reloaded['components']['target_dynamics']['effective']
    assert actual['model'] == model and actual['influences'] == disabled
    assert not reloaded['robot_response']['available']
    assert backend.dynamics.get('DYN-HALVORSEN', 1) == library
    assert backend.dynamics.get('DYN-ATTRACTOR', 1) == old_library
    assert backend.catalog.get('EXP-PHYSICAL', 1) == source
    with pytest.raises(ValueError, match='Unsupported influence projection'):
        backend.draft_with_robot_response(reloaded['spec'], True)
    assert backend.catalog.get('EXP-UI', 1) == saved
    assert len(backend.list_revisions('EXP-UI')) == 1

def test_initial_off_two_checkboxes_without_math_forms(state_backend):
    state,b=state_backend
    state=hydrate_target_pin(state,b)
    items=list(walk(component_cards(b,state)))
    robot=next(i for i in items if getattr(i,'id',None)=={'type':'influence','role':'robot_response'})
    env=next(i for i in items if getattr(i,'id',None)=='environmental-disturbance')
    assert robot.value==[] and not robot.options[0].get('disabled',False)
    assert env.value==[] and env.options[0]['disabled'] is True
    assert 'External Influences' in str(items)
    assert not any(isinstance(getattr(i,'id',None),dict) and i.id.get('type')=='param' and i.id.get('key')=='influences' for i in items)
    assert state['spec']['experiment']['parameters']['target_dynamics']['influences'][0]['enabled'] is False

def test_hydration_retains_on_snapshot_and_base_pin(state_backend):
    state,b=state_backend
    state['spec']=b.draft_with_robot_response(state['spec'],True)
    state['preview']=b.preview_experiment(state['spec']);state['components']=state['preview']['components']
    original=deepcopy(state['spec']['experiment'])
    state=hydrate_target_pin(state,b)
    assert state['spec']['experiment']['parameters']['target_dynamics']==original['parameters']['target_dynamics']
    assert state['spec']['experiment']['dynamics_source']==original['dynamics_source']
    assert state['robot_response']['enabled'] is True

@pytest.mark.parametrize('rejection', ['exception', 'preflight'])
def test_rejected_robot_response_redraws_the_authoritative_checkbox(state_backend, monkeypatch, rejection):
    """Exercise real Dash callback, not a separately simulated UI rollback."""
    import json
    from robot_dashboard.app import create_app
    from robot_dashboard.forms import fingerprint
    state,b=state_backend
    state=hydrate_target_pin(state,b)
    state['form_digest']=fingerprint(state['spec'])
    state['expected_revision']=None
    app=create_app(b)
    def reject(*args,**kwargs):raise ValueError('Physical world tracking TF required')
    if rejection == 'exception':
        monkeypatch.setattr(b,'draft_with_robot_response',reject)
    else:
        monkeypatch.setattr(b,'validate_experiment',lambda *a,**k:{
            'valid':False,'errors':['Physical world tracking TF required']})
    key,callback=next((k,v) for k,v in app.callback_map.items() if k.startswith('..builder-state.data'))
    inputs=[None]*5+[[],[],[],4.,'[42]',[],None,[['enabled']]]
    ex=state['spec']['experiment']
    states=[[],[],ex['base_case'],ex['base_revision'],'create',ex['id'],ex['name'],state]
    response=app.server.test_client().post('/_dash-update-component',json={
        'output':key,'outputs':[{'id':o.component_id,'property':o.component_property} for o in callback['output']],
        'inputs':[{**m,'value':v} for m,v in zip(callback['inputs'],inputs)],
        'state':[{**m,'value':v} for m,v in zip(callback['state'],states)],
        'changedPropIds':['{"role":"robot_response","type":"influence"}.value']})
    assert response.status_code==200
    result=response.get_json()['response']
    assert result['builder-state']['data']['stage']=='Invalid'
    assert 'Physical world tracking TF required' in result['builder-state']['data']['message']
    assert result['builder-state']['data']['spec']==state['spec']
    assert 'component-cards' in result
    def find(node):
        if isinstance(node,list):
            for child in node:yield from find(child)
        elif isinstance(node,dict):
            if node.get('props',{}).get('id')=={'type':'influence','role':'robot_response'}:yield node
            yield from find(node.get('props',{}).get('children'))
    checkbox=next(find(result['component-cards']['children']))
    assert checkbox['props']['value']==[]
