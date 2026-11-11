"""Bounded model-only preview uses the same D3/RK4 numerical contract."""

from copy import deepcopy

import numpy as np
import pytest

from robot_simulation.dynamics.motion_composition import dynamics_templates


def preview(*args, **kwargs):
    from robot_simulation.dynamics.motion_preview import preview_motion

    return preview_motion(*args, **kwargs)


def test_preview_uses_declared_initials_and_full_state_order():
    model = dynamics_templates()['constant_velocity']['definition']
    result = preview(model, duration=0.25, dt=0.1)
    assert result['times'] == [0.0, 0.1, 0.2, 0.25]
    initial = np.concatenate([s['initial'] for s in model['state']])
    np.testing.assert_allclose(result['states'][0], initial)
    np.testing.assert_allclose(result['states'][-1][:3], initial[:3] + 0.25 * initial[3:])
    assert result['position']['indices'] == [0, 1, 2]
    assert result['environment_included'] is False


def test_scalar_preview_is_not_invented_cartesian_output():
    model = dynamics_templates()['blank_state_space']['definition']
    model['parameters']['rate']['value'] = 2.0
    result = preview(model, duration=0.2, dt=0.1)
    assert result['states'][-1] == pytest.approx([0.4])
    assert result['position'] is None
    assert result['state_series'] == [{'name': 'q', 'unit': '1', 'indices': [0]}]


def test_preview_requires_explicit_fixed_context_and_reports_domain_stage():
    model = dynamics_templates()['saturated_fight_flight']['definition']
    with pytest.raises(ValueError, match='tracking_position_world'):
        preview(model, duration=0.2, dt=0.1)
    value = preview(model, duration=0.2, dt=0.1, context={'tracking_position_world': [0, 0, 0]})
    assert value['fixed_context'] == {'tracking_position_world': [0, 0, 0]}
    with pytest.raises(ValueError, match='undeclared'):
        preview(
            model,
            duration=0.2,
            dt=0.1,
            context={'tracking_position_world': [0, 0, 0], 'unknown': [1, 2, 3]},
        )
    bad = dynamics_templates()['blank_state_space']['definition']
    bad['state'][0]['initial'] = 0.0
    bad['equations']['q'] = 'rate/q'
    with pytest.raises(ValueError, match='step 1.*t=0'):
        preview(bad, duration=0.2, dt=0.1)


@pytest.mark.parametrize(
    'options',
    [
        dict(duration=0, dt=0.1),
        dict(duration=61, dt=0.1),
        dict(duration=1, dt=float('nan')),
        dict(duration=True, dt=0.1),
        dict(duration=60, dt=0.0001),
    ],
)
def test_preview_rejects_invalid_or_excessive_resource_requests(options):
    with pytest.raises(ValueError):
        preview(dynamics_templates()['spiral']['definition'], **options)


def test_preview_declarative_control_is_explicit_and_definition_unchanged():
    model = dynamics_templates()['blank_state_space']['definition']
    model['control'] = [{'name': 'u', 'shape': [], 'unit': '1/s', 'frame': 'world'}]
    model['equations']['q'] = 'u'
    saved = deepcopy(model)
    with pytest.raises(ValueError, match='control'):
        preview(model, duration=0.2, dt=0.1)
    result = preview(model, duration=0.2, dt=0.1, control=[3.0])
    assert result['states'][-1] == pytest.approx([0.6])
    assert model == saved


def test_roundoff_in_step_count_does_not_add_zero_length_final_step():
    result = preview(
        dynamics_templates()['constant_velocity']['definition'], duration=0.07, dt=0.01
    )
    assert result['times'][-1] == 0.07
    assert len(result['times']) == 8


def test_extremely_small_step_is_rejected_as_resource_error_not_overflow():
    with pytest.raises(ValueError, match='2000'):
        preview(dynamics_templates()['spiral']['definition'], duration=1.0, dt=1.e-320)
