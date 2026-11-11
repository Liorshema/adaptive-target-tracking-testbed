"""Legacy references and independent composition checks use real data-only templates."""

from copy import deepcopy
import numpy as np
import pytest

from robot_simulation.dynamics.motion_composition import compile_definition, dynamics_templates
from robot_simulation.dynamics.equation_model import EquationDynamics
from robot_simulation.dynamics.target.fight_flight import (
    FightFlightDynamics,
    FightFlightParameters,
    SaturatedFightFlightParameters,
    TangentialInteractionParameters,
)
from robot_simulation.integration.rk4 import RK4Integrator


@pytest.mark.parametrize('name', ['fight_flight', 'saturated_fight_flight'])
@pytest.mark.parametrize('tangential', [False, True])
@pytest.mark.parametrize(
    'delta',
    [[0.0, 0.0, 0.0], [1e-10, 0.0, 0.0], [0.0, 0.0, 2.0], [1.0, 0.2, 0.7], [-2.0, 1.0, 0.3]],
)
def test_legacy_radial_drag_regularization_and_tangent_equivalence(name, tangential, delta):
    source = dynamics_templates()[name]['definition']
    source['interactions'][0]['enabled'] = tangential
    params = {k: v['value'] for k, v in source['base']['parameters'].items()}
    params['distance_epsilon'] = params.pop('epsilon')
    radial = (FightFlightParameters if name == 'fight_flight' else SaturatedFightFlightParameters)(
        **params
    )
    tangent = source['interactions'][0]['parameters']
    circulation = (
        TangentialInteractionParameters(tangent['acceleration']['value'], tangent['axis']['value'])
        if tangential
        else None
    )

    class Field:
        def acceleration(self, p, t):
            return np.array([0.1, -0.2, 0.03 * t])

    old = FightFlightDynamics(radial, Field(), circulation)
    new = EquationDynamics(model=compile_definition(source)['model'], environment_field=Field())
    tracking = np.array([0.3, 0.2, 0.1])
    x = np.r_[tracking + delta, [0.1, -0.3, 0.2]]
    context = {'tracking_position_world': tracking}
    np.testing.assert_allclose(
        new.derivative(x, [], 0.7, context),
        old.derivative(x, [], 0.7, context),
        atol=1e-13,
        rtol=1e-13,
    )
    a = x.copy()
    b = x.copy()
    for step in range(25):
        a = RK4Integrator().step(new, a, [], step * 0.002, 0.002, context)
        b = RK4Integrator().step(old, b, [], step * 0.002, 0.002, context)
    np.testing.assert_allclose(a, b, atol=1e-12, rtol=1e-12)


def test_templates_are_detached_and_remain_mathematical_starting_points():
    data = dynamics_templates()
    assert {
        'constant_velocity',
        'turning',
        'spiral',
        'spring_damper',
        'saturated_fight_flight',
        'fight_flight',
        'blank_state_space',
        'attractor',
    } == set(data)
    assert not data['blank_state_space']['capabilities']['deployable']
    data['spiral']['definition']['base']['parameters']['stiffness']['value'] = 99.0
    assert (
        dynamics_templates()['spiral']['definition']['base']['parameters']['stiffness']['value']
        == 0.2
    )


def test_interaction_only_parameter_change_and_exact_disabled_base():
    source = dynamics_templates()['spiral']['definition']
    source['interactions'] = [
        deepcopy(dynamics_templates()['saturated_fight_flight']['definition']['base'])
    ]
    source['interactions'][0]['id'] = 'robot_response'
    source['interactions'][0]['parameters']['damping']['value'] = 0.0
    source['base']['enabled'] = False
    x = np.array([1.0, 0.2, 0.7, 0.1, -0.3, 0.2])
    context = {'tracking_position_world': np.zeros(3)}
    d = EquationDynamics(model=compile_definition(source)['model'])
    delta = x[:3]
    r = np.linalg.norm(delta)
    expected = 0.15 * np.tanh((r - 1.0) / 0.15) * delta / np.sqrt(r * r + 1e-12)
    np.testing.assert_allclose(
        d.derivative(x, [], 0.0, context), np.r_[x[3:], expected], atol=1e-14
    )
    source['base']['enabled'] = True
    first = EquationDynamics(model=compile_definition(source)['model']).derivative(
        x, [], 0.0, context
    )
    source['base']['parameters']['stiffness']['value'] = 0.7
    second = EquationDynamics(model=compile_definition(source)['model']).derivative(
        x, [], 0.0, context
    )
    np.testing.assert_allclose(second - first, np.r_[np.zeros(3), -0.5 * x[:3]], atol=1e-14)
    source['interactions'][0].update(enabled=False, expression='unknown')
    with pytest.raises(ValueError):
        compile_definition(source)


def test_bad_context_collision_and_circular_local_dependency_reject():
    source = dynamics_templates()['spiral']['definition']
    other = deepcopy(dynamics_templates()['saturated_fight_flight']['definition']['base'])
    source['interactions'] = [other]
    source['base']['context'] = {
        'tracking_position_world': {'unit': 'm/s', 'shape': [3], 'frame': 'world'}
    }
    with pytest.raises(ValueError):
        compile_definition(source)
    source['base'].pop('context')
    source['base']['intermediates'] = {'a': 'b', 'b': 'a'}
    with pytest.raises(ValueError):
        compile_definition(source)


def test_existing_factory_composite_environment_is_added_once():
    from pathlib import Path
    import yaml
    from robot_simulation.factory.simulation_factory import SimulationFactory

    source = dynamics_templates()['spiral']['definition']
    canonical = compile_definition(source)['model']
    environment = {
        'environment': {
            'acceleration_fields': [
                {
                    'type': 'linear_restoring',
                    'parameters': {
                        'center': [0.0, 0.0, 1.0],
                        'gain_matrix': [[0.3, 0.0, 0.0], [0.0, 0.3, 0.0], [0.0, 0.0, 0.3]],
                    },
                }
            ]
        }
    }
    integration = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/simulation.yaml').read_text()
    )
    engine, _ = SimulationFactory.with_defaults().create(
        {
            'target': {
                'dynamics': {'type': 'equation_state_space', 'parameters': {'model': canonical}}
            }
        },
        environment,
        integration,
    )
    x = np.array([1.0, 0.2, 0.7, 0.1, -0.3, 0.2])
    expected = (
        -0.2 * x[:3] - 0.1 * x[3:] + 0.4 * np.cross([0, 0, 1], x[3:]) - 0.3 * (x[:3] - [0, 0, 1])
    )
    np.testing.assert_allclose(
        engine.dynamics.derivative(x, [], 0.0, {}), np.r_[x[3:], expected], atol=1e-14
    )


def test_template_contribution_parameters_have_no_shared_mutable_aliases():
    source = dynamics_templates()['saturated_fight_flight']['definition']
    source['base']['parameters']['epsilon']['value'] = 0.01
    assert source['interactions'][0]['parameters']['epsilon']['value'] == 1e-6
