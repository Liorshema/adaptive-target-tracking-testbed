"""Translate detached browser forms to the existing one-file Author contract."""

import hashlib
import json
import math
from copy import deepcopy

INHERIT = '__inherit__'


def fingerprint(specification):
    """Identify exactly the form that was validated, independently of UI status."""
    return hashlib.sha256(
        json.dumps(specification, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def begin_draft(backend, source, revision, variant, mode, identifier=None, name=None):
    """Pin a supported base; a historical selection may create, never overwrite."""
    latest = backend.list_revisions(source)[-1]['revision']
    if mode == 'revise':
        if revision != latest:
            raise ValueError(
                'revision: select the latest revision to edit; '
                'use Create new to branch from history'
            )
        row = next(r for r in backend.list_experiments() if r['id'] == source)
        identifier, name = source, row['name']
    elif mode == 'create':
        identifier = backend.validate_experiment_id(identifier)
    else:
        raise ValueError('Choose Create new or Revise selected')
    spec = backend.new_draft(
        identifier, name, source, base_revision=revision, base_variant=variant
    )
    if mode == 'create':
        # Copy the selected configuration, not its scientific comparison contract.
        # Explicit null tells Author to remove the policy inherited from the base.
        spec['experiment']['comparison'] = None
    preview = backend.preview_experiment(spec)
    return {
        'spec': spec,
        'expected_revision': revision if mode == 'revise' else None,
        'preview': preview,
        'components': preview['components'],
        'stage': 'Draft',
        'validated_digest': None,
        'form_digest': fingerprint(spec),
        'message': 'Draft loaded. Changes stay in this browser until you explicitly save.',
    }


def parameter_rows(detail):
    """Keep default/effective/explicit provenance separate; never infer leaf ancestry."""
    properties = detail.get('schema', {}).get('properties', {})
    defaults, effective = detail.get('defaults', {}), detail.get('effective', {})
    overrides = detail.get('overrides', {})
    rows = []
    for key in dict.fromkeys([*properties, *effective, *defaults]):
        schema = properties.get(key, {})
        source = detail.get('parameter_sources', {}).get(key, {}).get('kind')
        rows.append(
            {
                'key': key,
                'schema': schema,
                'kind': schema.get('type', 'unknown'),
                'unit': schema.get('unit', 'not specified'),
                'default': defaults.get(key),
                'has_default': key in defaults,
                'value': effective.get(key, defaults.get(key)),
                'explicit': key in overrides,
                'editable': key in properties,
                'source': (
                    'Explicit experiment override'
                    if source == 'experiment_override'
                    else (
                        'Owner default; effective value pending preflight'
                        if detail.get('pending')
                        else 'Resolved / inherited; precise parameter origin unknown'
                    )
                ),
            }
        )
    return rows


def _value(value, kind, label):
    if kind in ('array', 'object', 'unknown'):
        try:
            value = json.loads(
                value,
                parse_constant=lambda v: (_ for _ in ()).throw(
                    ValueError(f'non-finite JSON constant {v}')
                ),
            )
        except (ValueError, TypeError) as error:
            raise ValueError(f'{label}: enter valid finite JSON ({error})') from error
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        raise ValueError(f'{label}: a finite value is required for an explicit override')
    return value


def assemble_draft(specification, component_values, component_ids, values, fields, enabled):
    """Apply form edits only; scientific schema and compatibility checks stay in Author."""
    spec = deepcopy(specification)
    ex = spec['experiment']
    components = ex.setdefault('components', {})
    parameters = ex.setdefault('parameters', {})
    changed = set()
    for value, field in zip(component_values, component_ids):
        role = field['role']
        old = components.get(role, INHERIT)
        if value != old:
            changed.add(role)
            parameters.pop(role, None)
        if value == INHERIT:
            components.pop(role, None)
        else:
            components[role] = value
    for value, field, switches in zip(values, fields, enabled):
        role, key = field['role'], field['key']
        if role in changed:
            continue
        if 'override' in (switches or []):
            parameters.setdefault(role, {})[key] = _value(
                value, field['kind'], f'experiment.parameters.{role}.{key}'
            )
        elif role in parameters:
            parameters[role].pop(key, None)
        if role in parameters and not parameters[role]:
            parameters.pop(role)
    if not components:
        ex.pop('components', None)
    if not parameters:
        ex.pop('parameters', None)
    return spec


def apply_execution(specification, duration, seeds):
    """Apply author-owned execution fields; full domain validation stays in Author."""
    if type(duration) not in (int, float) or not math.isfinite(duration):
        raise ValueError('experiment.execution.duration: enter finite simulation seconds')
    try:
        parsed = json.loads(seeds)
    except (TypeError, ValueError) as error:
        raise ValueError('experiment.execution.seeds: enter a JSON integer array') from error
    if not isinstance(parsed, list) or any(type(seed) is not int for seed in parsed):
        raise ValueError('experiment.execution.seeds: enter a JSON integer array')
    spec = deepcopy(specification)
    spec['experiment'].setdefault('execution', {}).update(duration=duration, seeds=parsed)
    return spec


def apply_comparison(specification, path, models, values, options):
    """Declare exact variables; full resolved validation remains in the Author."""
    from robot_experiments.comparison import validate_policy
    spec = deepcopy(specification)
    if path in ('__legacy__', '__preserve__'):
        return spec
    if path in (None, '__strict__'):
        spec['experiment']['comparison'] = None
        return spec
    detail = next((option for option in options if option['path'] == path), None)
    if detail is None:
        raise ValueError('comparison variable: select a registered exact path')
    existing = (spec['experiment'].get('comparison') or {}).get('variables', [])
    name = existing[0]['name'] if len(existing) == 1 and existing[0]['path'] == path else path.replace('.', '_')
    variable = {'name': name, 'path': path}
    if detail['kind'] == 'model':
        variable['values'] = models
    elif values and values.strip():
        variable['values'] = _value(values, 'array', 'comparison permitted values')
    spec['experiment']['comparison'] = validate_policy({
        'schema_version': 2, 'domain': detail['domain'], 'variables': [variable]})
    return spec
