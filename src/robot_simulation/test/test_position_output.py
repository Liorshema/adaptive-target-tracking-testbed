"""Explicit mathematical output mapping never changes the integrated state contract."""
from copy import deepcopy

import numpy as np
import pytest

from robot_simulation.dynamics.equation_model import EquationDynamics
from robot_simulation.dynamics.motion_preview import preview_motion


def circular(names=('x','y','z')):
    a,b,c=names
    return {'id':'generic_motion','type':'equation_state_space','schema_version':1,'frame':'world',
            'state':[{'name':n,'shape':[],'unit':'m','initial':v} for n,v in zip(names,[1.,0.,1.])],
            'parameters':{'omega':{'shape':[],'unit':'1/s','value':.5}},
            'equations':{a:f'-omega*{b}',b:f'omega*{a}',c:'0'},
            'outputs':{'schema_version':1,'frame':'world','position':f'[{a},{b},{c}]'}}


@pytest.mark.parametrize('names',[('x','y','z'),('a','b','c')])
def test_scalar_world_output_and_chain_rule_follow_the_equations(names):
    definition=circular(names)
    m=EquationDynamics(model=definition)
    p,v=m.physical_output(m.initial_state,[],0,{})
    np.testing.assert_allclose(p,[1,0,1])
    np.testing.assert_allclose(v,[0,.5,0])
    assert m.capabilities['deployable']  # Prescribed mapping; full environment preflight remains required.
    r=preview_motion(definition,duration=2,dt=.01)
    actual=np.asarray(r['positions'])
    expected=np.column_stack([np.cos(.5*np.array(r['times'])),np.sin(.5*np.array(r['times'])),np.ones(len(r['times']))])
    np.testing.assert_allclose(actual,expected,atol=1e-9,rtol=0)
    np.testing.assert_allclose(np.linalg.norm(actual[:,:2],axis=1),1,atol=1e-9,rtol=0)
    np.testing.assert_allclose(r['velocities'][-1],[-.5*np.sin(1),.5*np.cos(1),0],atol=1e-9,rtol=0)
    assert len(r['states'][-1])==3
    assert r['output_mapping']['velocity_verified'] is True


def test_nonlinear_and_explicit_time_output_have_analytic_total_velocity():
    d=circular()
    d['parameters']['speed']={'shape':[],'unit':'m/s','value':2.}
    d['parameters']['length']={'shape':[],'unit':'m','value':1.}
    d['outputs']['position']='[x*x/length,y,z+speed*t]'
    m=EquationDynamics(model=d)
    p,v=m.physical_output([2,3,1],[],.5,{})
    np.testing.assert_allclose(p,[4,3,2])
    np.testing.assert_allclose(v,[-6,1,2])


def test_vector_output_keeps_full_internal_state_and_order():
    d={'id':'vector_model','schema_version':1,'type':'equation_state_space','frame':'world',
       'state':[{'name':'q','shape':[3],'unit':'m','initial':[1,2,3]}, {'name':'extra','shape':[2],'unit':'1','initial':[7,8]}],
       'parameters':{'drift':{'shape':[3],'unit':'m/s','value':[.1,.2,.3]}},
       'equations':{'q':'drift','extra':'[0,0]'},
       'outputs':{'schema_version':1,'position':'q','frame':'world'}}
    r=preview_motion(d,duration=.2,dt=.1)
    np.testing.assert_allclose(r['states'][-1],[1.02,2.04,3.06,7,8])
    np.testing.assert_allclose(r['velocities'][-1],[.1,.2,.3])
    assert r['state_series'][1]['indices']==[3,4]
    assert d['state'][0]['shape']==[3]


@pytest.mark.parametrize('change',[
    {'position':'[x,y]'}, {'position':'[omega,omega,omega]'}, {'frame':'map'},
    {'position':'__import__("os")'}, {'position':'[robot,robot,robot]'},
    {'position':'[x,y,z]','velocity':'[0,0,0]'}])
def test_invalid_mapping_fails_owner_validation(change):
    d=circular();d['outputs'].update(change)
    with pytest.raises(ValueError):EquationDynamics(model=d)


def test_dimensionless_lorenz_needs_explicit_length_scaling_for_meter_output():
    d={'id':'generic_nonlinear','schema_version':1,'type':'equation_state_space','frame':'world',
       'state':[{'name':n,'shape':[],'unit':'1','initial':v} for n,v in zip(['a','b','c'],[1,1,1])],
       'parameters':{n:{'shape':[],'unit':u,'value':v} for n,u,v in [('sigma','1',10),('rho','1',28),('beta','1',8/3),('rate','1/s',1),('length','m',.1)]},
       'equations':{'a':'rate*sigma*(b-a)','b':'rate*(a*(rho-c)-b)','c':'rate*(a*b-beta*c)'},
       'outputs':{'schema_version':1,'position':'length*[a,b,c]','frame':'world'}}
    m=EquationDynamics(model=d)
    p,v=m.physical_output(m.initial_state,[],0,{})
    np.testing.assert_allclose(p,[.1,.1,.1]);np.testing.assert_allclose(v,[0,2.6,-1/6])
    assert np.isfinite(preview_motion(d,duration=.1,dt=.01)['states']).all()
    d['outputs']['position']='[a,b,c]'
    with pytest.raises(ValueError,match='unit|Unit'):EquationDynamics(model=d)


def test_mapping_cannot_override_an_existing_live_cartesian_adapter():
    from robot_simulation.dynamics.motion_composition import dynamics_templates
    d=dynamics_templates()['constant_velocity']['definition']
    p=d['state'][0]['name']
    d['outputs']={'schema_version':1,'position':p,'frame':d['frame']}
    assert EquationDynamics(model=d).capabilities['deployable']
    d['outputs']['position']=f'2*{p}'
    m=EquationDynamics(model=d)
    assert not m.capabilities['deployable']
    with pytest.raises(ValueError):m.validate_deployment()


def test_robot_context_must_be_declared_and_cannot_be_a_hidden_position_mapping_input():
    d=circular()
    d['context']={'tracking_position_world':{'shape':[3],'unit':'m','frame':'world'}}
    d['parameters']['gain']={'shape':[],'unit':'1/s','value':.2}
    d['equations']['x']='gain*(tracking_position_world[0]-x)'
    m=EquationDynamics(model=d)
    with pytest.raises(ValueError,match='context'):m.physical_output(m.initial_state,[],0,{})
    _,v=m.physical_output(m.initial_state,[],0,{'tracking_position_world':[2,0,0]})
    assert v[0]==pytest.approx(.2)
    d['outputs']['position']='tracking_position_world'
    with pytest.raises(ValueError,match='state|input'):EquationDynamics(model=d)


def test_unused_parameter_is_identified_without_implied_motion_semantics():
    d=circular();d['parameters']['R']={'shape':[],'unit':'m','value':9}
    assert EquationDynamics(model=d).capabilities['unused_parameters']==['R']


def test_output_derivative_expansion_has_a_total_work_budget():
    d = circular()
    d['intermediates'] = {'q0':'(x+x)+(x+x)'}
    d['intermediates'].update({f'q{i}':f'(q{i-1}+q{i-1})+(q{i-1}+q{i-1})' for i in range(1,10)})
    d['outputs']['position'] = '[q9,y,z]'
    with pytest.raises(ValueError, match='work budget'):
        EquationDynamics(model=d)


def test_angle_state_maps_to_position_with_dimensionally_correct_velocity():
    d = {'id':'angular_state', 'type':'equation_state_space', 'schema_version':1, 'frame':'world',
         'state':[{'name':'theta','shape':[],'unit':'rad','initial':.4}],
         'parameters':{'omega':{'shape':[],'unit':'rad/s','value':.5},
                       'length':{'shape':[],'unit':'m','value':2}},
         'equations':{'theta':'omega'},
         'outputs':{'schema_version':1,'frame':'world',
                    'position':'[length*cos(theta),length*sin(theta),0]'}}
    m = EquationDynamics(model=d)
    p,v = m.physical_output([.4],[],0,{})
    np.testing.assert_allclose(p,[2*np.cos(.4),2*np.sin(.4),0])
    np.testing.assert_allclose(v,[-np.sin(.4),np.cos(.4),0])
    assert m.output_mapping['unit_symbols']


@pytest.mark.parametrize('expression',[
    '[length*sin(x/length),length*cos(y/length),length*tanh(z/length)]',
    '[length*exp(x/length),sqrt(y*length),z**2/length]',
    'length*cross([x/length,y/length,z/length],[1,2,3])',
    '[norm([x,y,z]),dot([x,y,z],[x,y,z])/length,z]',
])
def test_analytic_output_rules_match_independent_directional_difference(expression):
    d = circular()
    d['parameters']['length'] = {'shape':[],'unit':'m','value':2}
    d['outputs']['position'] = expression
    m = EquationDynamics(model=d)
    x = np.array([1.,2.,3.]); t = .3
    dx = m.derivative(x,[],t,{})
    p,v = m.physical_output(x,[],t,{})
    epsilon = 1e-6
    plus,_ = m.physical_output(x+epsilon*dx,[],t+epsilon,{})
    minus,_ = m.physical_output(x-epsilon*dx,[],t-epsilon,{})
    np.testing.assert_allclose(v,(plus-minus)/(2*epsilon),atol=2e-9,rtol=2e-8)
    assert p.shape == (3,)
