"""Typesetting is a safe view of actual generic equations, not another evaluator."""
import pytest


def model():
    return {'state':[{'name':'a','shape':[],'unit':'m','initial':1.0},{'name':'q','shape':[3],'unit':'m','initial':[1,2,3]}],
            'equations':{'a':'-omega*a+dot(q,q)/length/time','q':'cross(axis,q)*omega'},
            'parameters':{},'intermediates':{}}


def test_mathematics_formats_actual_scalar_vector_nonlinear_and_initial_values():
    from robot_dashboard.mathematics import model_tex
    result=model_tex(model())
    assert '\\omega' in result and '\\times' in result
    assert '\\mathbf{q}' in result and '\\begin{bmatrix}' in result
    assert '1.0' in result and '2' in result and '\\dot' in result
    assert 'circular' not in result


@pytest.mark.parametrize('bad',['__import__("os")','a.__class__','lambda: 1','[v for v in a]','\\href{evil}{x}'])
def test_typesetting_never_interprets_unrestricted_expressions_or_latex(bad):
    from robot_dashboard.mathematics import expression_tex
    with pytest.raises(ValueError):expression_tex(bad)


def test_greek_is_presentation_only_and_arbitrary_names_remain_generic():
    from robot_dashboard.mathematics import expression_tex
    assert expression_tex('-omega_x*b') != expression_tex('-omega_x*c')
    assert '\\omega' in expression_tex('-omega_x*b')
    assert 'robot' in expression_tex('robot')


def test_verified_velocity_presents_rates_and_radian_units_without_internal_ids():
    from robot_dashboard.mathematics import velocity_tex
    model = {'state':[{'name':'theta','shape':[]}]}
    mapping = {'velocity':'outputRate0/outputRadianUnit',
               'rate_symbols':{'outputRate0':'theta'},
               'unit_symbols':{'outputRadianUnit':{'value':1,'unit':'rad'}}}
    rendered = velocity_tex(mapping, model)
    assert r'\dot{\theta}' in rendered and r'\mathrm{rad}' in rendered
    assert 'outputRate' not in rendered and 'outputRadianUnit' not in rendered


def test_velocity_rate_bindings_replace_only_exact_ast_names():
    from robot_dashboard.mathematics import velocity_tex
    mapping = {'velocity':'outputRate0+outputRate0_extra',
               'rate_symbols':{'outputRate0':'theta'}, 'unit_symbols':{}}
    rendered = velocity_tex(mapping, {'state':[{'name':'theta','shape':[]}]})
    assert rendered.count(r'\dot{\theta}') == 1
    assert r'\mathrm{outputRate0}_{\mathrm{extra}}' in rendered
