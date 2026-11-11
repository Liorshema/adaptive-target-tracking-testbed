"""Detached structured Studio forms; mathematical checks remain in D3/D4.0."""

import json
from copy import deepcopy

BOUNDS = ('minimum', 'maximum', 'exclusiveMinimum', 'exclusiveMaximum')


def text(value):
    """Represent individual numeric cells, not an editable model document."""
    return json.dumps(value, allow_nan=False, separators=(',', ':'))


def numeric(value, label):
    """Parse a finite JSON numeric cell; the owner checks its physical shape/domain."""
    try:
        parsed = json.loads(value) if isinstance(value, str) else value
        text(parsed)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{label}: enter a finite number or numeric array') from error
    return parsed


def _unique(rows, label):
    result = {}
    for row in rows:
        value = row.get('name', '')
        if not isinstance(value, str):
            raise ValueError(f'{label}: names must be text')
        name = value.strip()
        if not name:
            raise ValueError(f'{label}: every row needs a name (delete unused rows)')
        if name in result:
            raise ValueError(f'Duplicate {label} name: {name}')
        result[name] = row
    return result


def editor_data(definition, index=0):
    """Project one complete model/contribution into editable fields and row tables."""
    d = deepcopy(definition)
    composed = d['type'] == 'cartesian_composition'
    source = [d['base'], *d.get('interactions', [])][index] if composed else d
    fields = {
        'model_id': d['id'],
        'frame': d['frame'],
        'output_position': d.get('outputs', {}).get('position', ''),
        'output_frame': d.get('outputs', {}).get('frame', d['frame']),
    }
    if composed:
        fields.update(
            source_id=source['id'],
            contribution_frame=source['frame'],
            expression=source['expression'],
            enabled=source.get('enabled', True),
            output_shape=text(source['output']['shape']),
            output_unit=source['output']['unit'],
        )
    else:
        deployment = d.get('deployment', {})
        fields.update(
            deployment=deployment.get('adapter', 'none'),
            position=deployment.get('position', ''),
            velocity=deployment.get('velocity', ''),
        )
    tables = {
        'state': [
            {
                **s,
                'shape': text(s['shape']),
                'initial': text(s['initial']),
                'derivative': d.get('equations', {}).get(s['name'], ''),
            }
            for s in d['state']
        ],
        'parameters': [
            {'name': name, **spec, 'shape': text(spec['shape']), 'value': text(spec['value'])}
            for name, spec in source['parameters'].items()
        ],
        'intermediates': [
            {'name': k, 'expression': v} for k, v in source.get('intermediates', {}).items()
        ],
        'context': [
            {'name': k, **v, 'shape': text(v['shape'])}
            for k, v in source.get('context', {}).items()
        ],
        'control': [{**v, 'shape': text(v['shape'])} for v in d.get('control', [])],
    }
    return fields, tables


def read_editor(definition, index, fields, tables):
    """Read structured cells without silently dropping declarations or duplicate names."""
    d = deepcopy(definition)
    d.update(id=fields['model_id'], frame=fields['frame'])
    state_rows = _unique(tables['state'], 'state')
    d['state'] = [
        {
            'name': name,
            'shape': numeric(r['shape'], f'{name}.shape'),
            'unit': r['unit'],
            'initial': numeric(r['initial'], f'{name}.initial'),
        }
        for name, r in state_rows.items()
    ]
    composed = d['type'] == 'cartesian_composition'
    source = [d['base'], *d.get('interactions', [])][index] if composed else d
    source['parameters'] = {}
    for name, r in _unique(tables['parameters'], 'parameter').items():
        spec = {
            'shape': numeric(r['shape'], f'{name}.shape'),
            'unit': r['unit'],
            'value': numeric(r['value'], f'{name}.value'),
        }
        for bound in BOUNDS:
            if r.get(bound) not in ('', None):
                spec[bound] = numeric(r[bound], f'{name}.{bound}')
        source['parameters'][name] = spec
    for key in ('context', 'control'):
        rows = tables.get(key, [])
        if composed and key == 'control':
            continue
        if rows or key in source:
            declarations = [
                {
                    'name': name,
                    'shape': numeric(r['shape'], f'{name}.shape'),
                    'unit': r['unit'],
                    'frame': r['frame'],
                }
                for name, r in _unique(rows, key).items()
            ]
            source[key] = (
                declarations if key == 'control' else {s.pop('name'): s for s in declarations}
            )
    intermediates = {
        k: r['expression'] for k, r in _unique(tables['intermediates'], 'intermediate').items()
    }
    if intermediates or 'intermediates' in source:
        source['intermediates'] = intermediates
    if composed:
        source.update(
            id=fields['source_id'],
            frame=fields['contribution_frame'],
            expression=fields['expression'],
            output={
                'shape': numeric(fields['output_shape'], 'output.shape'),
                'unit': fields['output_unit'],
            },
        )
        if not fields['enabled'] or 'enabled' in source:
            source['enabled'] = fields['enabled']
    else:
        d['equations'] = {name: r['derivative'] for name, r in state_rows.items()}
        if fields['deployment'] == 'none':
            d.pop('deployment', None)
        else:
            d['deployment'] = {
                'adapter': fields['deployment'],
                'position': fields['position'],
                'velocity': fields['velocity'],
            }
    if fields.get('output_position', '').strip():
        d['outputs'] = {
            'schema_version': 1,
            'frame': fields['output_frame'],
            'position': fields['output_position'],
        }
    else:
        d.pop('outputs', None)
    return d


def add_interaction(definition, templates, template, *, zero_drag):
    """Copy an influence into a draft; explicit opt-in controls additional drag."""
    d = deepcopy(definition)
    if d['type'] != 'cartesian_composition':
        raise ValueError('Interactions require Composed Cartesian Motion')
    source = (
        deepcopy(templates[template]['definition']['base'])
        if template != 'custom'
        else {
            'id': 'influence',
            'frame': d['frame'],
            'output': {'shape': [3], 'unit': 'm/s^2'},
            'parameters': {},
            'expression': '[0,0,0]',
        }
    )
    used = {s['id'] for s in [d['base'], *d.get('interactions', [])]}
    stem, n = source['id'], 1
    while source['id'] in used:
        source['id'] = f'{stem}_{n}'
        n += 1
    if zero_drag and 'damping' in source['parameters']:
        source['parameters']['damping']['value'] = 0.0
    d.setdefault('interactions', []).append(source)
    return d


def load_model(backend, identifier, revision):
    """Read a detached pinned source; its selected revision is the CAS expectation."""
    if revision is None:
        raise ValueError('Select an explicit immutable revision before opening')
    row = backend.dynamics.get(identifier, revision)
    label = next(r['name'] for r in backend.dynamics.list() if r['id'] == identifier)
    return {
        'id': identifier,
        'name': label,
        'expected_revision': row['revision'],
        'definition': row['definition'],
        'saved_hash': row['sha256'],
        'dirty': False,
        'selected': 0,
        'stage': 'Saved',
        'preview': None,
        'checked': backend.dynamics.validate(row['definition']),
    }


def save_model(backend, state, identifier, name, definition):
    """Create/revise explicitly; failed validation and CAS leave stored history intact."""
    if state.get('id'):
        if identifier != state['id']:
            raise ValueError(
                'Saved model ID is immutable; use New from template for a new identity'
            )
        if (
            name != state['name']
            and definition
            == backend.dynamics.get(identifier, state['expected_revision'])['definition']
        ):
            backend.dynamics.rename(
                identifier,
                name,
                expected_revision=state['expected_revision'],
                expected_name=state['name'],
            )
            saved = load_model(backend, identifier, state['expected_revision'])
            saved['selected'] = state.get('selected', 0)
            return saved
        record = backend.dynamics.revise(
            identifier,
            definition,
            expected_revision=state['expected_revision'],
            created_by='Dynamics Studio',
            name=name,
            expected_name=state['name'],
        )
    else:
        record = backend.dynamics.create(
            identifier, name, definition, created_by='Dynamics Studio'
        )
    saved = load_model(backend, identifier, record['revision'])
    saved['selected'] = state.get('selected', 0)
    return saved


def preview_inputs(model):
    """Declare editable fixed assumptions; never pretend these are live robot measurements."""
    result = []
    for kind, declarations in [
        ('context', [{'name': k, **v} for k, v in model.get('context', {}).items()]),
        ('control', model.get('control', [])),
    ]:
        for spec in declarations:
            result.append(
                {
                    'kind': kind,
                    'name': spec['name'],
                    'shape': text(spec['shape']),
                    'unit': spec['unit'],
                    'fixed': '',
                }
            )
    return result


def restore_library_model(specification, model_text):
    """Restore exact frozen numeric representations after JavaScript JSON transport.

    JavaScript has one numeric type: 1.0 returns as 1. Preserve the canonical
    JSON text while checking exact numeric/structural equality of the visible
    object. This restores original bytes/types, never weakens owner hash checks.
    """
    from robot_experiments.dynamics_library import validate_dynamics_source

    spec = deepcopy(specification)
    ex = spec['experiment']
    if 'dynamics_source' not in ex:
        return spec
    if not model_text:
        raise ValueError(
            'Reload and apply the explicit dynamics revision to preserve its canonical definition'
        )
    frozen = json.loads(model_text)
    current = ex.get('parameters', {}).get('target_dynamics', {}).get('model')

    def same(a, b):
        if type(a) in (int, float) and type(b) in (int, float):
            return a == b
        if type(a) is not type(b):
            return False
        if isinstance(a, dict):
            return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
        if isinstance(a, list):
            return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
        return a == b

    if not same(current, frozen):
        raise ValueError('Pinned model changed; reapply an explicit library revision')
    validate_dynamics_source(ex['dynamics_source'], frozen)
    ex['parameters']['target_dynamics']['model'] = frozen
    return spec


def select_target_source(backend, state, source, identifier=None, revision=None):
    """Switch source without losing detached registered edits or canonical JSON types."""
    state = deepcopy(state)
    spec = state['spec']
    ex = spec['experiment']
    if source == 'library':
        if 'dynamics_source' not in ex:
            state['registered_dynamics'] = {
                'selection': ex.get('components', {}).get('target_dynamics'),
                'parameters': deepcopy(ex.get('parameters', {}).get('target_dynamics')),
            }
        spec = backend.draft_with_dynamics(spec, identifier, revision=revision)
        state['dynamics_model_text'] = json.dumps(
            spec['experiment']['parameters']['target_dynamics']['model'], allow_nan=False
        )
        state['spec'] = spec
    elif source == 'registered':
        if 'dynamics_source' in ex:
            previous = state.get('registered_dynamics', {})
            ex.pop('dynamics_source')
            for group, saved in [('components', 'selection'), ('parameters', 'parameters')]:
                ex.get(group, {}).pop('target_dynamics', None)
                if previous.get(saved) is not None:
                    ex.setdefault(group, {})['target_dynamics'] = deepcopy(previous[saved])
                if group in ex and not ex[group]:
                    ex.pop(group)
            state.pop('dynamics_model_text', None)
    else:
        raise ValueError('Select registered dynamics or a saved library revision')
    return state


def target_options(backend, pin=None, *, inherited=None, selection=None):
    """Unify labels while retaining explicit immutable library revisions in values."""
    result = [
        {
            'label': o['id'].replace('_', ' ').capitalize(),
            'value': o['id'],
            'disabled': not o['deployable'] or o['compatible'] is False,
        }
        for o in backend.list_component_options('target_dynamics')
    ]
    try:
        rows = backend.dynamics.list()
        for row in rows:
            caps = backend.dynamics.capabilities(row['id'], row['revision'])
            result.append(
                {
                    'label': row['name'] + (' · preview only' if not caps['deployable'] else ''),
                    'value': f"library:{row['id']}:{row['revision']}",
                }
            )
        if pin and not any(
            o['value'] == f"library:{pin['library_id']}:{pin['revision']}" for o in result
        ):
            label = next(
                (r['name'] for r in rows if r['id'] == pin['library_id']), 'Saved equation model'
            )
            result.append(
                {
                    'label': label + ' · current saved version',
                    'value': f"library:{pin['library_id']}:{pin['revision']}",
                }
            )
    except (ValueError, OSError):
        if pin:
            result.append(
                {
                    'label': 'Saved equation model · frozen version',
                    'value': f"library:{pin['library_id']}:{pin['revision']}",
                }
            )
    if inherited is not None:
        from .forms import INHERIT
        result = [
            {'label': inherited.replace('_', ' ').capitalize(),
             'value': inherited if pin is None and selection == inherited else INHERIT},
            *(o for o in result if o['value'] != inherited),
        ]
    return result


def choose_target_model(backend, state, selection):
    """Apply one dropdown intent, pinning the option's actual immutable revision."""
    if selection.startswith('library:'):
        _, identifier, number = selection.split(':')
        return select_target_source(backend, state, 'library', identifier, int(number))
    restored = select_target_source(backend, state, 'registered')
    ex = restored['spec']['experiment']
    previous = ex.get('components', {}).get('target_dynamics')
    if selection == '__inherit__':
        if previous is not None:
            ex.get('components', {}).pop('target_dynamics', None)
            ex.get('parameters', {}).pop('target_dynamics', None)
    elif selection != previous:
        ex.setdefault('components', {})['target_dynamics'] = selection
        ex.get('parameters', {}).pop('target_dynamics', None)
    return restored


def hydrate_target_pin(state, backend=None):
    """Use frozen Author provenance and effective model, never mutable library latest."""
    from robot_experiments.dynamics_library import validate_dynamics_source

    state = deepcopy(state)
    preview = state.get('preview') or {}
    source = preview.get('provenance', {}).get('dynamics_library')
    model = (
        preview.get('components', {}).get('target_dynamics', {}).get('effective', {}).get('model')
    )
    if source is not None and model is not None:
        validate_dynamics_source(source, model)
        ex = state['spec']['experiment']
        ex.setdefault('components', {})['target_dynamics'] = 'equation_state_space'
        values = preview['components']['target_dynamics']['effective']
        parameters = {'model': deepcopy(model)}
        if 'influences' in values:
            parameters['influences'] = deepcopy(values['influences'])
        if backend is not None:
            detail = backend.describe_robot_response(model, values.get('influences'))
            state['robot_response'] = detail
            if detail['available']:
                parameters['influences'] = deepcopy(detail['influences'])
        ex.setdefault('parameters', {})['target_dynamics'] = parameters
        ex['dynamics_source'] = deepcopy(source)
        state['dynamics_model_text'] = json.dumps(model, allow_nan=False)
        from .forms import fingerprint
        state['form_digest'] = fingerprint(state['spec'])
    return state
