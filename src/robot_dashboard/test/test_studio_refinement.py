"""D4.5 Studio presentation keeps the complete generic mathematical draft."""

from copy import deepcopy
from pathlib import Path

import pytest

from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog


@pytest.fixture
def backend(tmp_path):
    """Use the real library owner with an isolated workspace."""
    src = Path(__file__).resolve().parents[2]
    catalog = ExperimentCatalog(
        tmp_path / 'catalog.yaml', share=lambda p: src / p
    )
    catalog.initialize()
    return ExperimentBackend(tmp_path / 'data', catalog=catalog)


def components(node):
    """Walk mounted Dash components, including closed disclosures."""
    if isinstance(node, (list, tuple)):
        for child in node:
            yield from components(child)
    elif hasattr(node, 'to_plotly_json'):
        yield node
        yield from components(getattr(node, 'children', None))


def mapped_model():
    """Build a six-state generic ODE with seven physical coefficients."""
    return {
        'schema_version': 1,
        'type': 'equation_state_space',
        'id': 'six_state_system',
        'frame': 'world',
        'state': [
            {'name': name, 'shape': [], 'unit': unit, 'initial': initial}
            for name, unit, initial in [
                ('x', 'm', 1.0), ('y', 'm', 0.0), ('z', 'm', 1.0),
                ('vx', 'm/s', 0.0), ('vy', 'm/s', 0.5), ('vz', 'm/s', 0.0),
            ]
        ],
        'parameters': {
            'kx': {'shape': [], 'unit': '1/s^2', 'value': 0.25},
            'ky': {'shape': [], 'unit': '1/s^2', 'value': 0.25},
            'kz': {'shape': [], 'unit': '1/s^2', 'value': 0.25},
            'damping': {'shape': [], 'unit': '1/s', 'value': 0.1},
            'ax': {'shape': [], 'unit': 'm/s^2', 'value': 0.0},
            'ay': {'shape': [], 'unit': 'm/s^2', 'value': 0.0},
            'az': {'shape': [], 'unit': 'm/s^2', 'value': 0.25},
        },
        'equations': {
            'x': 'vx', 'y': 'vy', 'z': 'vz',
            'vx': '-kx*x-damping*vx+ax',
            'vy': '-ky*y-damping*vy+ay',
            'vz': '-kz*z-damping*vz+az',
        },
        'outputs': {
            'schema_version': 1, 'frame': 'world', 'position': '[x,y,z]'
        },
    }


def test_default_feedback_is_empty_before_and_after_initial_callback(backend):
    """Initial presentation must not reinsert the removed description."""
    from dash import Dash
    from robot_dashboard.dynamics_studio import (
        new_state, register_studio, studio_layout,
    )

    layout = studio_layout(backend)
    feedback = next(
        c for c in components(layout)
        if getattr(c, 'id', None) == 'dyn-feedback'
    )
    assert not feedback.children
    app = Dash(__name__)
    app.layout = layout
    register_studio(app, backend)
    callback = next(v['callback'] for k, v in app.callback_map.items()
                    if k.startswith('..dyn-feedback.children'))
    presentation = callback.__wrapped__(new_state(backend), '3d')
    assert presentation[0] == ''


def test_output_mapping_starts_closed_with_complete_mounted_controls(backend):
    """Collapsing the output must keep the editable mapping mounted."""
    from robot_dashboard.dynamics_studio import editor

    source = mapped_model()
    before = deepcopy(source)
    rendered = list(components(editor(source)))
    panel = next(c for c in rendered
                 if 'studio-output' in getattr(c, 'className', '').split())
    props = panel.to_plotly_json()
    assert props['type'] == 'Details'
    assert props['props'].get('open', False) is False
    children = list(components(panel))
    assert children[1].to_plotly_json()['type'] == 'Summary'
    assert children[1].children == 'Physical Output Mapping'
    position = next(c for c in children if getattr(c, 'id', None)
                    == {'type': 'dyn-field', 'key': 'output_position'})
    assert position.value == '[x,y,z]'
    assert any(
        getattr(c, 'id', None) == 'dyn-output-velocity' for c in children
    )
    # Native expansion changes visibility without rewriting the draft.
    panel.open = True
    panel.open = False
    assert position.value == '[x,y,z]'
    assert source == before


def test_closed_mapping_keeps_full_editor_save_reload_and_preview(backend):
    """Preserve every mounted row/output through persistence and preview."""
    from robot_dashboard.dynamics_forms import (
        load_model, read_editor, save_model,
    )
    from robot_dashboard.dynamics_studio import editor, new_state

    source = mapped_model()
    rendered = list(components(editor(source)))
    fields = {
        c.id['key']: c.value for c in rendered
        if isinstance(getattr(c, 'id', None), dict)
        and c.id['type'] == 'dyn-field'
    }
    tables = {
        c.id['key']: c.data for c in rendered
        if isinstance(getattr(c, 'id', None), dict)
        and c.id['type'] == 'dyn-table'
    }
    assert len(tables['state']) == 6
    assert len(tables['parameters']) == 7
    edited = read_editor(source, 0, fields, tables)
    assert edited == source
    state = new_state(backend)
    saved = save_model(
        backend, state, 'DYN-SIX-STATE', 'Six scalar states', edited
    )
    restored = load_model(backend, 'DYN-SIX-STATE', saved['expected_revision'])
    assert restored['definition'] == source
    result = backend.dynamics.preview(
        restored['definition'], duration=0.1, dt=0.02
    )
    assert len(result['states'][0]) == 6
    assert result['positions'][0] == [1.0, 0.0, 1.0]
    assert result['velocities'][0] == [0.0, 0.5, 0.0]
