"""Unified model selection pins on intent, never on a library list refresh."""
import json
from pathlib import Path
from copy import deepcopy

import pytest
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog


@pytest.fixture
def backend(tmp_path):
    src=Path(__file__).resolve().parents[2]
    catalog=ExperimentCatalog(tmp_path/'catalog.yaml',share=lambda p:src/p)
    catalog.initialize()
    return ExperimentBackend(tmp_path/'data',catalog=catalog)


def test_target_dropdown_contains_registered_and_saved_preview_only_models(backend):
    from robot_dashboard.dynamics_forms import target_options
    d=backend.dynamics.templates()['blank_state_space']['definition']
    backend.dynamics.create('DYN-USER','User ODE',d)
    options=target_options(backend)
    assert any(o['value']=='fight_flight' for o in options)
    user=next(o for o in options if o['value'].startswith('library:DYN-USER:'))
    assert 'User ODE' in user['label'] and 'preview' in user['label'].lower()
    assert not user.get('disabled')


def test_single_selection_pins_latest_at_selection_and_restores_registered_edits(backend):
    from robot_dashboard.dynamics_forms import choose_target_model, target_options
    d=backend.dynamics.templates()['spiral']['definition']
    backend.dynamics.create('DYN-USER','User ODE',d)
    spec=backend.new_draft('EXP-UNIFIED','Unified','EXP-NOMINAL')
    spec['experiment']['components']={'target_dynamics':'fight_flight'}
    spec['experiment']['parameters']={'target_dynamics':{'damping':.27}}
    state={'spec':spec}
    pinned=choose_target_model(backend,state,'library:DYN-USER:1')
    assert pinned['spec']['experiment']['dynamics_source']['revision']==1
    before=deepcopy(pinned['spec'])
    d['base']['parameters']['damping']['value']=.3
    backend.dynamics.revise('DYN-USER',d,expected_revision=1)
    choices=target_options(backend,pinned['spec']['experiment']['dynamics_source'])
    assert any(c['value']=='library:DYN-USER:2' for c in choices)
    assert any(c['value']=='library:DYN-USER:1' for c in choices)
    assert pinned['spec']==before
    restored=choose_target_model(backend,pinned,'fight_flight')
    assert restored['spec']['experiment']==spec['experiment']


def test_loading_saved_experiment_retains_pin_without_resolving_mutable_library(backend):
    from robot_dashboard.forms import begin_draft
    from robot_dashboard.dynamics_forms import hydrate_target_pin
    d=backend.dynamics.templates()['spiral']['definition']
    backend.dynamics.create('DYN-HISTORY','Historical ODE',d)
    draft=backend.draft_with_dynamics(backend.new_draft('EXP-HISTORY','Historical','EXP-NOMINAL'),'DYN-HISTORY',revision=1)
    saved=backend.save_experiment(draft)
    backend.dynamics.revise('DYN-HISTORY',d,expected_revision=1)
    state=begin_draft(backend,'EXP-HISTORY',1,'imm','revise')
    hydrated=hydrate_target_pin(state)
    assert hydrated['spec']['experiment']['dynamics_source']['revision']==1
    assert json.loads(hydrated['dynamics_model_text'])==saved['overrides']['target']['target']['dynamics']['parameters']['model']
    # The frozen snapshot is sufficient even after the original library disappears.
    backend.dynamics.path.unlink()
    again=hydrate_target_pin(state)
    assert again['spec']==hydrated['spec']

@pytest.mark.parametrize('explicit', [False, True])
def test_studio_library_refresh_preserves_simple_labels_and_selection(explicit):
    """A Studio save must not restore old Lab labels or lose explicit selection."""
    from unittest.mock import Mock
    from dash import Dash
    from robot_dashboard.dynamics_studio import register_studio
    from robot_dashboard.forms import INHERIT
    backend=Mock()
    backend.list_component_options.return_value=[
        {'id':'fight_flight','deployable':True,'compatible':True},
        {'id':'constant_velocity','deployable':True,'compatible':True}]
    backend.dynamics.list.return_value=[]
    app=Dash(__name__);register_studio(app,backend)
    callback=next(v['callback'] for k,v in app.callback_map.items()
                  if k.endswith('"type":"component"}.options'))
    selected='fight_flight' if explicit else INHERIT
    state={'spec':{'experiment':{'components':{'target_dynamics':selected}}},
           'inherited_components':{'target_dynamics':{'id':'fight_flight'}}}
    choices=callback.__wrapped__({},state)
    assert [o['label'] for o in choices]==['Fight flight','Constant velocity']
    assert sum(o['value']==selected for o in choices)==1


def test_pinned_model_refresh_keeps_inherited_reset_semantics():
    """A pinned model's component ID is not an explicit registered selection."""
    from unittest.mock import Mock
    from robot_dashboard.dynamics_forms import target_options
    from robot_dashboard.forms import INHERIT
    backend=Mock()
    backend.list_component_options.return_value=[
        {'id':'equation_state_space','deployable':True,'compatible':True}]
    backend.dynamics.list.return_value=[]
    pin={'library_id':'DYN-PIN','revision':1}
    choices=target_options(backend,pin,inherited='equation_state_space',
                           selection='equation_state_space')
    assert choices[0]['value']==INHERIT
    assert any(o['value']=='library:DYN-PIN:1' for o in choices)
