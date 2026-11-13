"""Independent mathematical references for YAML acceleration injection."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_simulation.dynamics.equation_model import EquationDynamics
from robot_simulation.integration.rk4 import RK4Integrator

CONFIG=Path(__file__).parents[1]/'config'

def model():
    return yaml.safe_load((CONFIG/'dynamics_templates.yaml').read_text())['templates']['attractor']['definition']

def profile(enabled=True):
    p=yaml.safe_load((CONFIG/'derivative_influences.yaml').read_text())['profiles']['robot_response']
    p['enabled']=enabled
    return p


def halvorsen():
    return yaml.safe_load((Path(__file__).parent / 'model_fixtures/halvorsen.yaml').read_text())


def test_disabled_incompatible_profile_preserves_mapped_three_state_dynamics_exactly():
    original = halvorsen()
    before = deepcopy(original)
    frozen = profile(False)
    baseline = EquationDynamics(model=original, influences=[])
    baseline.validate_deployment('world')
    disabled = EquationDynamics(model=original, influences=[frozen])
    disabled.validate_deployment('world')
    assert disabled.definition == baseline.definition
    assert disabled.base_definition == original == before
    assert disabled.influences == [frozen] and not disabled.requires_tracking_context
    left, right = baseline.initial_state, disabled.initial_state
    rk = RK4Integrator()
    for step in range(200):
        t = step * .002
        np.testing.assert_array_equal(baseline.derivative(left, [], t, {}),
                                      disabled.derivative(right, [], t, {}))
        left = rk.step(baseline, left, [], t, .002, {})
        right = rk.step(disabled, right, [], t, .002, {})
        np.testing.assert_array_equal(left, right)
        for a, b in zip(baseline.physical_output(left, [], t + .002, {}),
                        disabled.physical_output(right, [], t + .002, {})):
            np.testing.assert_array_equal(a, b)
    with pytest.raises(ValueError, match='Unsupported influence projection'):
        EquationDynamics(model=original, influences=[profile(True)])


@pytest.mark.parametrize('change', ['version', 'enabled', 'frame', 'units',
                                  'expression', 'parameter', 'bindings', 'symbol'])
def test_disabled_profiles_still_validate_their_own_contract(change):
    frozen = profile(False)
    if change == 'version': frozen['schema_version'] = True
    elif change == 'enabled': frozen['enabled'] = 'off'
    elif change == 'frame': frozen['frame'] = 'map'
    elif change == 'units': frozen['parameters']['L']['unit'] = 's'
    elif change == 'expression': frozen['expression'] = "__import__('os')"
    elif change == 'parameter': frozen['parameters']['A_max']['value'] = -1.
    elif change == 'symbol': frozen['expression'] = (
        'A_max*influenceValidationVelocity/norm(influenceValidationVelocity)')
    else: frozen['bindings'] = {'vx': True}
    with pytest.raises(ValueError):
        EquationDynamics(model=halvorsen(), influences=[frozen])

def response(delta):
    distance=np.linalg.norm(delta)
    return .3*np.exp(-distance/.8)*delta/np.sqrt(distance**2+.05**2)

@pytest.mark.parametrize('delta',[[1,0,0],[-1,0,0],[.3,-.4,.2],[1e6,0,0],[1e-12,0,0],[0,0,0]])
def test_yaml_response_direction_decay_and_bound(delta):
    base=EquationDynamics(model=model());on=EquationDynamics(model=model(),influences=[profile()])
    x=base.initial_state;context={'frame_id':'world','tracking_position_world':x[:3]-delta}
    actual=on.derivative(x,[],0.,context)-base.derivative(x,[],0.,{})
    np.testing.assert_allclose(actual[:3],0,atol=0)
    np.testing.assert_allclose(actual[3:],response(np.array(delta)),atol=1e-15,rtol=1e-12)
    assert np.linalg.norm(actual[3:])<=.3+1e-15
    assert on.base_definition==model()

@pytest.mark.parametrize('name',['A_max','L','epsilon'])
@pytest.mark.parametrize('bad',[0.,-1.,float('nan'),float('inf'),True])
def test_invalid_parameters_fail_closed(name,bad):
    p=profile();p['parameters'][name]['value']=bad
    with pytest.raises(ValueError):EquationDynamics(model=model(),influences=[p])

def test_missing_nonfinite_and_wrong_frame_context_fail_closed():
    d=EquationDynamics(model=model(),influences=[profile()])
    for context in [{},{'tracking_position_world':[0,0,float('nan')]},
                    {'tracking_position_world':[0,0,0],'frame_id':'map'}]:
        with pytest.raises(ValueError):d.derivative(d.initial_state,[],0.,context)

def test_exact_off_and_independent_integrated_reference():
    original=model();before=deepcopy(original)
    base=EquationDynamics(model=original);off=EquationDynamics(model=original,influences=[profile(False)])
    on=EquationDynamics(model=original,influences=[profile()]);rk=RK4Integrator()
    a=b=c=base.initial_state;reference=base.initial_state
    def f(x,robot):
        p,v=x[:3],x[3:]
        return np.r_[v,-.2*(p-[0,0,1])-.1*v+np.cross([0,0,.4],v)+response(p-robot)]
    for i in range(200):
        t=i*.01;robot=np.array([-.2+.01*t,0,.5]);ctx={'tracking_position_world':robot}
        a=rk.step(base,a,[],t,.01,{})
        b=rk.step(off,b,[],t,.01,ctx)
        c=rk.step(on,c,[],t,.01,ctx)
        h=.01;q=reference;k1=f(q,robot);k2=f(q+h*k1/2,robot);k3=f(q+h*k2/2,robot);k4=f(q+h*k3,robot)
        reference=q+h*(k1+2*k2+2*k3+k4)/6
    np.testing.assert_array_equal(a,b)
    np.testing.assert_allclose(c,reference,atol=1e-12,rtol=0)
    assert c[0]>a[0]
    assert original==before
    pos,vel=on.physical_output(c,[],2.,ctx)
    np.testing.assert_array_equal(pos,c[:3]);np.testing.assert_array_equal(vel,c[3:])

def test_vector_binding_and_hidden_state_have_no_name_or_order_assumptions():
    d=model();d['state']=[{'name':'hidden','shape':[],'unit':'1','initial':7.},
        {'name':'where','shape':[3],'unit':'m','initial':[1,0,1]},
        {'name':'rate','shape':[3],'unit':'m/s','initial':[0,.2,0]}]
    d['equations']={'hidden':'0','where':'rate','rate':'-(k/m)*(where-g)-(c/m)*rate+cross(omega,rate)'}
    d['intermediates']={};d['outputs']['position']='where'
    p=profile();p['bindings']={'rate':[0,1,2]}
    e=EquationDynamics(model=d,influences=[p]);x=e.initial_state
    value=e.derivative(x,[],0.,{'tracking_position_world':[0,0,0]})
    np.testing.assert_allclose(value,np.r_[0,x[4:],[-.2,-.02,0]+np.cross([0,0,.4],x[4:])+response(x[1:4])])
    assert x[0]==7

@pytest.mark.parametrize('change',['position','missing_state','wrong_axis','duplicate_axis','nonlinear'])
def test_unsupported_physical_bindings_are_rejected(change):
    d=model();p=profile()
    if change=='position':p['bindings']={'x':0,'y':1,'z':2}
    elif change=='missing_state':p['bindings']['absent']=p['bindings'].pop('vx')
    elif change=='wrong_axis':p['bindings']['vx']=2
    elif change=='duplicate_axis':p['bindings']['vz']=1
    else:d['outputs']['position']='2*[x,y,z]'
    with pytest.raises(ValueError):EquationDynamics(model=d,influences=[p])


def test_influence_only_parameter_override_is_not_silently_discarded():
    values=EquationDynamics.prepare_parameters({'influences':[profile()]},{'model':model()})
    assert values['influences']==[profile()]
    EquationDynamics(**values)


def test_continuity_units_and_expression_validation():
    base=EquationDynamics(model=model());d=EquationDynamics(model=model(),influences=[profile()]);x=d.initial_state
    values=[]
    for epsilon in [-1e-10,0.,1e-10]:
        robot=x[:3]-[epsilon,0,0]
        values.append(d.derivative(x,[],0.,{'tracking_position_world':robot})-base.derivative(x,[],0.,{}))
    assert np.linalg.norm(values[2]-values[0])<2e-9
    for change in ['units','expression','frame','version']:
        p=profile()
        if change=='units':p['parameters']['L']['unit']='s'
        elif change=='expression':p['expression']="__import__('os')"
        elif change=='frame':p['frame']='map'
        else:p['schema_version']=True
        with pytest.raises(ValueError):EquationDynamics(model=model(),influences=[p])
