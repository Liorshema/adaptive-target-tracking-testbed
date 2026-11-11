"""Deterministic Cartesian authoring-to-D3 transformation; no numerical execution."""

import ast
import json
from copy import deepcopy

from robot_configuration.resources import package_config_path
from robot_configuration.yaml_loader import load_yaml, load_yaml_group

from .equation_expression import parse_expression
from .equation_model import EquationDynamics, _mapping, _name
from .equation_units import parse_unit


def definition_data(value):
    """Detach finite YAML/JSON values, including aliases, without changing numerical data."""
    pending, visited = [value], set()
    while pending:
        item = pending.pop()
        if isinstance(item, (dict, list, tuple)):
            if id(item) in visited:
                continue
            visited.add(id(item))
            if isinstance(item, dict):
                if any(not isinstance(key, str) for key in item):
                    raise ValueError('Dynamics definition keys must be strings')
                pending.extend(item.values())
            else:
                pending.extend(item)
    try:
        return json.loads(json.dumps(value, allow_nan=False))
    except (TypeError, ValueError) as error:
        raise ValueError('Dynamics definitions must contain finite JSON/YAML data') from error


def compile_definition(definition):
    """Return a complete validated canonical model and contribution symbol provenance."""
    if not isinstance(definition, dict):
        raise ValueError('Dynamics definition must be a mapping')
    if definition.get('type') == 'equation_state_space':
        return {'model': EquationDynamics(model=definition).definition, 'symbol_map': {}}
    _mapping(
        definition,
        {'schema_version', 'type', 'id', 'frame', 'state', 'base', 'interactions', 'outputs'},
        {'schema_version', 'type', 'id', 'frame', 'state', 'base'},
        'composition',
    )
    if (
        definition['type'] != 'cartesian_composition'
        or type(definition['schema_version']) is not int
        or definition['schema_version'] != 1
    ):
        raise ValueError('Expected cartesian_composition schema_version 1')
    _name(definition['id'])
    states = deepcopy(definition['state'])
    if (
        not isinstance(states, list)
        or len(states) != 2
        or any(not isinstance(s, dict) or 'name' not in s for s in states)
    ):
        raise ValueError(
            'Cartesian composition requires ordered position and velocity declarations'
        )
    position, velocity = [s['name'] for s in states]
    _name(position)
    _name(velocity)
    interactions = definition.get('interactions', [])
    if not isinstance(interactions, list) or len(interactions) > 31:
        raise ValueError('interactions must be an ordered list of at most 31 contributions')
    contributions = [definition['base'], *interactions]
    used = {position, velocity, 't', 'pi'}
    # Shared context is kept under its actual runtime key, not a fictitious namespace.
    for contribution in contributions:
        if isinstance(contribution, dict) and isinstance(contribution.get('context', {}), dict):
            used.update(contribution.get('context', {}))

    def allocate(stem):
        name = stem
        suffix = 0
        while name in used:
            suffix += 1
            name = f'{stem}_{suffix}'
        used.add(name)
        return name

    canonical = {
        'id': definition['id'],
        'type': 'equation_state_space',
        'schema_version': 1,
        'frame': definition['frame'],
        'state': states,
        'parameters': {},
        'context': {},
        'intermediates': {},
        'equations': {},
        'deployment': {'adapter': 'cartesian_target', 'position': position, 'velocity': velocity},
    }
    if 'outputs' in definition:
        canonical['outputs'] = deepcopy(definition['outputs'])
    mapping, terms = {}, []
    for index, source in enumerate(contributions):
        _mapping(
            source,
            {
                'id',
                'frame',
                'output',
                'parameters',
                'context',
                'intermediates',
                'expression',
                'enabled',
            },
            {'id', 'frame', 'output', 'parameters', 'expression'},
            'contribution',
        )
        identifier = _name(source['id'])
        if identifier in mapping:
            raise ValueError(f'Duplicate contribution identity: {identifier}')
        if source['frame'] != definition['frame']:
            raise ValueError('Contribution and target frames differ')
        enabled = source.get('enabled', True)
        if type(enabled) is not bool:
            raise ValueError('Contribution enabled must be boolean')
        output = _mapping(
            source['output'], {'shape', 'unit'}, {'shape', 'unit'}, 'contribution output'
        )
        if output['shape'] != [3] or parse_unit(output['unit']) != parse_unit('m/s^2'):
            raise ValueError('Contribution output must be a 3-vector acceleration in m/s^2')
        # Validate locally before renaming: unknown symbols cannot accidentally
        # resolve to a generated symbol owned by a different contribution.
        probe = {
            **deepcopy(canonical),
            'parameters': deepcopy(source['parameters']),
            'context': deepcopy(source.get('context', {})),
            'intermediates': deepcopy(source.get('intermediates', {})),
            'equations': {position: velocity, velocity: source['expression']},
        }
        probe.pop('outputs', None)
        EquationDynamics(model=probe)
        parameters = {
            name: allocate(f'c{index}_p{i}') for i, name in enumerate(sorted(source['parameters']))
        }
        intermediate = {
            name: allocate(f'c{index}_i{i}')
            for i, name in enumerate(sorted(source.get('intermediates', {})))
        }
        rename = {**parameters, **intermediate}

        class Rename(ast.NodeTransformer):
            def visit_Name(self, node):
                return ast.copy_location(
                    ast.Name(id=rename.get(node.id, node.id), ctx=node.ctx), node
                )

        def expression(text):
            return ast.unparse(Rename().visit(parse_expression(text)))

        output_name = allocate(f'c{index}_a')
        mapping[identifier] = {
            'order': index,
            'role': 'base' if index == 0 else 'interaction',
            'included': enabled,
            'parameters': parameters,
            'intermediates': intermediate,
            'context': {name: name for name in source.get('context', {})},
            'output': output_name,
        }
        if not enabled:
            continue
        for name, declaration in source.get('context', {}).items():
            previous = canonical['context'].get(name)
            if previous is not None and previous != declaration:
                raise ValueError(f'Incompatible shared context declarations: {name}')
            canonical['context'][name] = deepcopy(declaration)
        for name, value in source['parameters'].items():
            canonical['parameters'][parameters[name]] = deepcopy(value)
        for name, text in source.get('intermediates', {}).items():
            canonical['intermediates'][intermediate[name]] = expression(text)
        canonical['intermediates'][output_name] = expression(source['expression'])
        terms.append(output_name)
    canonical['equations'] = {
        position: velocity,
        velocity: ' + '.join(terms) if terms else '[0,0,0]',
    }
    return {'model': EquationDynamics(model=canonical).definition, 'symbol_map': mapping}


def dynamics_templates():
    """Read detached mathematical starting points from the installed owner data."""
    path = package_config_path(__file__, 'robot_simulation', 'dynamics_templates.yaml')
    data = load_yaml(path)
    _mapping(data, {'schema_version', 'templates'}, {'schema_version', 'templates'}, 'templates')
    if (
        type(data['schema_version']) is not int
        or data['schema_version'] != 1
        or not isinstance(data['templates'], dict)
    ):
        raise ValueError('Unsupported dynamics templates schema')
    result = {}
    for identifier, item in data['templates'].items():
        _mapping(item, {'name', 'definition', 'source'}, {'name'}, 'template')
        if ('definition' in item) == ('source' in item):
            raise ValueError('Template must supply one definition or owner source')
        if 'source' in item:
            source = _mapping(
                item['source'], {'file', 'group'}, {'file', 'group'}, 'template source'
            )
            if (
                not isinstance(source['file'], str)
                or '/' in source['file']
                or source['file'] in ('.', '..')
            ):
                raise ValueError('Template source must name an owner config file')
            definition = load_yaml_group(path.parent / source['file'], source['group'])
        else:
            definition = item['definition']
        definition = definition_data(definition)
        compiled = compile_definition(definition)
        result[identifier] = {
            'name': item['name'],
            'definition': definition,
            'capabilities': EquationDynamics(model=compiled['model']).capabilities,
        }
    return result
