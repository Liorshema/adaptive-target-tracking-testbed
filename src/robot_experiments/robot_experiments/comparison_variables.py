"""Typed component variables projected from immutable owner-resolved evidence.

Only exact scalar/array leaves are projected. Component owners remain responsible
for registered identities, parameter schemas, defaults and deployment preflight.
"""

from copy import deepcopy
import json
import re

from robot_configuration.components import validate_schema


_ROLES = frozenset({'predictor', 'controller', 'robot_estimator', 'target_estimator'})
_DOMAINS = frozenset({'component_models', 'component_parameters', 'target_dynamics'})
_PROTECTED_ESTIMATION = frozenset({
    'joint_count', 'target_spatial_dimension', 'joint_bounds_enabled',
    'joint_bounds_numerical_tolerance', 'max_joint_state_age',
})
_FLAT_CONTROLLER_PARAMETERS = {'gains': 'controller_task_gains', 'damping': 'controller_damping'}


def _registry(role):
    # Keep component discovery lazy: configuration imports the comparison engine.
    from .components import component_registries

    registries = component_registries()
    return registries['dynamics' if role == 'target_dynamics' else role]


def _metadata(role, identifier):
    try:
        result = _registry(role).describe(identifier)
    except (KeyError, TypeError) as error:
        raise ValueError(f'comparison {role}: unregistered component {identifier!r}') from error
    if result['capabilities'].get('deployable', True) is not True:
        raise ValueError(f'comparison {role}: component {identifier!r} is not deployable')
    return result


def _finite(value):
    try:
        return json.dumps(value, sort_keys=True, allow_nan=False)
    except (ValueError, TypeError) as error:
        raise ValueError('comparison values must be finite JSON') from error


def _parse(path, domain):
    if (not isinstance(path, str)
            or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*(?:\.[A-Za-z][A-Za-z0-9_]*)+', path)):
        raise ValueError(f'comparison path {path!r}: expected an exact component or parameter path')
    keys = path.split('.')
    kind, role = keys[:2]
    allowed_roles = {'target_dynamics'} if domain == 'target_dynamics' else _ROLES
    if role not in allowed_roles:
        raise ValueError(f'comparison path {path!r}: unsupported role for {domain}')
    if kind == 'components' and len(keys) == 2 and domain != 'component_parameters':
        return kind, role, ()
    if kind == 'parameters' and len(keys) >= 3 and domain != 'component_models':
        return kind, role, tuple(keys[2:])
    raise ValueError(f'comparison path {path!r}: unsupported variable category for {domain}')


def _leaf_schema(schema, keys):
    for key in keys:
        if schema.get('type') != 'object' or key not in schema.get('properties', {}):
            raise ValueError('not a registered parameter leaf')
        schema = schema['properties'][key]
    if schema.get('type') == 'object':
        raise ValueError('parameter containers cannot be masked')
    return schema


def _permitted_parameter(role, keys):
    if role.endswith('_estimator'):
        name = keys[0]
        if (name in _PROTECTED_ESTIMATION or name.startswith('localization_')
                or name.endswith(('_process_model', '_measurement_model',
                                  '_deployment_adapter', '_algorithm_parameters'))):
            raise ValueError(f'comparison {role}.{name}: protected binding, timing or safety field')


def _parameter_schema(role, keys, metadata):
    _permitted_parameter(role, keys)
    try:
        return _leaf_schema(metadata['schema'], keys)
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f'comparison parameters.{role}.{".".join(keys)}: '
                         'not a registered scalar/array parameter leaf') from error


def validate_component_policy(policy):
    """Validate v2 categories, registered role choices and exact owner-schema leaves."""
    if not isinstance(policy, dict) or set(policy) != {'schema_version', 'domain', 'variables'}:
        raise ValueError('comparison requires schema_version, domain and variables only')
    if type(policy['schema_version']) is not int or policy['schema_version'] != 2:
        raise ValueError('component comparison.schema_version: supported integer 2')
    domain = policy['domain']
    if not isinstance(domain, str) or domain not in _DOMAINS:
        raise ValueError('component comparison.domain: unsupported typed comparison category')
    if not isinstance(policy['variables'], list) or not policy['variables']:
        raise ValueError('comparison.variables: expected nonempty list')
    names, paths = set(), set()
    for variable in policy['variables']:
        if (not isinstance(variable, dict) or not {'name', 'path'} <= variable.keys()
                or variable.keys() - {'name', 'path', 'values'}):
            raise ValueError('comparison variable requires name/path and optional values only')
        name, path = variable['name'], variable['path']
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,79}', name):
            raise ValueError('comparison variable name: expected descriptive identifier')
        kind, role, keys = _parse(path, domain)
        if name in names or path in paths:
            raise ValueError('comparison variables: duplicate name or path')
        names.add(name)
        paths.add(path)
        values = variable.get('values')
        if 'values' in variable or kind == 'components':
            if not isinstance(values, list) or not values:
                raise ValueError(f'comparison variable {name}: values must be a nonempty list')
            encoded = [_finite(value) for value in values]
            if len(set(encoded)) != len(encoded):
                raise ValueError(f'comparison variable {name}: duplicate permitted values')
        if kind == 'components':
            for identifier in values:
                if not isinstance(identifier, str):
                    raise ValueError(f'comparison variable {name}: model values must be registered IDs')
                _metadata(role, identifier)
            continue
        schemas = []
        for item in _registry(role).describe_all():
            if item['capabilities'].get('deployable', True) is not True:
                continue
            try:
                schemas.append(_parameter_schema(role, keys, item))
            except ValueError:
                pass
        if not schemas:
            raise ValueError(f'comparison path {path!r}: protected or unsupported parameter leaf')
        for value in values or []:
            for schema in schemas:
                try:
                    validate_schema(value, schema, path)
                    break
                except (ValueError, TypeError):
                    pass
            else:
                raise ValueError(f'comparison {path}: permitted value violates registered parameter schema')
    return deepcopy(policy)


def _get(mapping, keys, label):
    value = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            raise ValueError(f'comparison {label}: required mirror/parameter absent at {".".join(keys)}')
        value = value[key]
    return value


def _remove(mapping, keys, label):
    parent = _get(mapping, keys[:-1], label)
    _get(parent, keys[-1:], label)
    del parent[keys[-1]]


def _selected(role, original, conditions):
    if role == 'target_dynamics':
        return _get(original, ('target_dynamics', 'target', 'dynamics', 'type'), role)
    if role == 'controller':
        identifier = _get(conditions, ('controller_parameters', 'controller_type'), role)
        strategy = _get(conditions, ('controller_strategy',), role)
        if strategy is not None:
            mirrored = _get(strategy, ('controller', 'type'), role)
            if mirrored != identifier:
                raise ValueError('comparison controller: conflicting controller identity mirrors')
        return identifier
    return _get(conditions, ('component_selection', role + '_type'), role)


def _parameter_location(role, identifier, conditions):
    if role == 'target_dynamics':
        return 'original', ('target_dynamics', 'target', 'dynamics', 'parameters')
    if role == 'predictor':
        return 'conditions', ('prediction_configuration', 'predictors', identifier, 'parameters')
    if role == 'controller':
        if conditions.get('controller_strategy') is not None:
            return 'conditions', ('controller_strategy', 'controller', 'parameters')
        return 'conditions', ('controller_parameters',)
    return 'conditions', ('estimator_parameters',)


def _intrinsic(conditions, role, metadata):
    from robot_estimation.factory.algorithm_parameters import decode_algorithm_parameters
    from .components import component_registries

    prefix = role.removesuffix('_estimator')
    algorithm = metadata['capabilities']['algorithm']
    encoded = _get(conditions, ('estimator_parameters', prefix + '_algorithm_parameters'), role)
    try:
        parameters = decode_algorithm_parameters(encoded, algorithm, component_registries()['estimator'])
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f'comparison {role}: invalid frozen algorithm parameters: {error}') from error
    # Explicit process/measurement/adapter selections remain protected. Validate
    # their registration without replacing them by the selected algorithm defaults.
    for field, family in [('process_model', 'estimation_process'),
                          ('measurement_model', 'estimation_measurement'),
                          ('deployment_adapter', 'estimation_adapter')]:
        selected = conditions['estimator_parameters'].get(prefix + '_' + field)
        if selected is not None:
            try:
                component_registries()[family].describe(selected)
            except (KeyError, TypeError) as error:
                raise ValueError(f'comparison {role}: invalid {field} binding') from error
    return prefix + '_algorithm_parameters', {'algorithm': algorithm, 'parameters': parameters}


def _schema_leaves(schema, prefix=()):
    if schema.get('type') != 'object':
        return {prefix}
    return {path for key, child in schema.get('properties', {}).items()
            for path in _schema_leaves(child, prefix + (key,))}


def comparison_options():
    """Discover detached v2 UI choices from deployable owners and exact schemas."""
    from .components import component_registries

    registries, options = component_registries(), []
    for role in sorted(_ROLES | {'target_dynamics'}):
        registry = registries['dynamics' if role == 'target_dynamics' else role]
        models = [item for item in registry.describe_all()
                  if item['capabilities'].get('deployable', True) is True]
        if not models:
            continue
        domain = 'target_dynamics' if role == 'target_dynamics' else 'component_models'
        path = 'components.' + role
        _parse(path, domain)
        options.append({'path': path, 'domain': domain, 'kind': 'model', 'role': role,
                        'model_values': [item['id'] for item in models]})
        parameters = {}
        for metadata in models:
            for keys in _schema_leaves(metadata['schema']):
                try:
                    _parameter_schema(role, keys, metadata)
                except (ValueError, IndexError):
                    continue
                path = 'parameters.' + role + '.' + '.'.join(keys)
                domain = ('target_dynamics' if role == 'target_dynamics'
                          else 'component_parameters')
                _parse(path, domain)
                parameters.setdefault(path, {'path': path, 'domain': domain,
                                            'kind': 'parameter', 'role': role,
                                            'model_values': []})['model_values'].append(metadata['id'])
        options.extend(parameters.values())
    return deepcopy(sorted(options, key=lambda item: item['path']))


def _model_defaults(role, identifier, variable, source_parameters, invariant_parameters,
                    *, parameter_group=()):
    """Project only model-specific, present leaves equal to their owner defaults.

    Shared leaves (including optional shared settings) remain protected even when
    their model defaults differ. New values therefore cannot hide behind selection.
    """
    metadata = [_metadata(role, name) for name in variable['values']]
    for item in metadata:
        for group in parameter_group:
            item['schema'] = item['schema']['properties'][group]
            item['defaults'] = item['defaults'][group]
    shared = set.intersection(*[_schema_leaves(item['schema']) for item in metadata])
    selected = next(item for item in metadata if item['id'] == identifier)
    for keys in _schema_leaves(selected['schema']) - shared:
        try:
            effective = _get(source_parameters, keys, role)
        except ValueError:
            continue  # Optional absent owner leaves carry no evidence to project.
        try:
            default = _get(selected['defaults'], keys, role)
        except ValueError as error:
            raise ValueError(f'comparison {role}.{".".join(keys)}: '
                             'undeclared model-specific parameter has no owner default') from error
        validate_schema(effective, _leaf_schema(selected['schema'], keys), role)
        if _finite(effective) != _finite(default):
            raise ValueError(f'comparison {role}.{".".join(keys)}: '
                             'undeclared model-specific parameter differs from owner default')
        _remove(invariant_parameters, keys, role)


def project_components(original, conditions, policy):
    """Return detached invariant contracts and exact effective variable evidence.

    Original contracts/hashes stay with the caller. This function never mutates
    frozen evidence and never projects physics, task, sensor or safety containers.
    """
    declaration = validate_component_policy(policy)
    if not isinstance(original, dict) or not isinstance(conditions, dict):
        raise ValueError('comparison requires resolved contract and condition mappings')
    invariant_original, invariant_conditions = deepcopy(original), deepcopy(conditions)
    variables, intrinsic_documents = {}, {}
    for variable in declaration['variables']:
        path = variable['path']
        kind, role, keys = _parse(path, declaration['domain'])
        identifier = _selected(role, original, conditions)
        metadata = _metadata(role, identifier)
        if kind == 'components':
            if identifier not in variable['values']:
                raise ValueError(f'comparison variable {variable["name"]}: effective model not permitted')
            effective = identifier
            if role == 'target_dynamics':
                _remove(invariant_original, ('target_dynamics', 'target', 'dynamics', 'type'), path)
            elif role == 'controller':
                _remove(invariant_conditions, ('controller_parameters', 'controller_type'), path)
                if conditions['controller_strategy'] is not None:
                    _remove(invariant_conditions, ('controller_strategy', 'controller', 'type'), path)
            else:
                _remove(invariant_conditions, ('component_selection', role + '_type'), path)
            if role == 'predictor':
                _, location = _parameter_location(role, identifier, conditions)
                validate_schema(_get(conditions, location, path), metadata['schema'], path)
            elif role.endswith('_estimator'):
                field, document = _intrinsic(conditions, role, metadata)
                projected_document = deepcopy(document)
                _model_defaults(role, identifier, variable, document['parameters'],
                                projected_document['parameters'],
                                parameter_group=('algorithm_parameters',))
                # Normalize the canonical algorithm mirror and exact default-owned
                # intrinsic leaves. Shared numerical values and bindings remain.
                document = projected_document
                del document['algorithm']
                invariant_conditions['estimator_parameters'][field] = _finite(document)
            elif role in {'target_dynamics', 'controller'}:
                owner, location = _parameter_location(role, identifier, conditions)
                source = original if owner == 'original' else conditions
                target = invariant_original if owner == 'original' else invariant_conditions
                parameters = _get(source, location, path)
                projected = _get(target, location, path)
                if role == 'controller' and conditions['controller_strategy'] is None:
                    # Flat ROS controller fields are transport aliases, not a map
                    # of arbitrary model-owned values. Shared task/bounds stay.
                    for name, alias in _FLAT_CONTROLLER_PARAMETERS.items():
                        if name in metadata['schema'].get('properties', {}):
                            validate_schema(_get(parameters, (alias,), path),
                                            metadata['schema']['properties'][name], path)
                else:
                    validate_schema(parameters, metadata['schema'], path)
                    _model_defaults(role, identifier, variable, parameters, projected)
        else:
            schema = _parameter_schema(role, keys, metadata)
            if role.endswith('_estimator') and keys[0] == 'algorithm_parameters':
                field, document = _intrinsic(conditions, role, metadata)
                effective = _get(document['parameters'], keys[1:], path)
                validate_schema(effective, schema, path)
                projected_document = intrinsic_documents.setdefault(role, deepcopy(document))
                _remove(projected_document['parameters'], keys[1:], path)
                invariant_conditions['estimator_parameters'][field] = _finite(projected_document)
            else:
                owner, location = _parameter_location(role, identifier, conditions)
                transported = keys
                if role == 'controller' and conditions['controller_strategy'] is None:
                    if len(keys) != 1 or keys[0] not in _FLAT_CONTROLLER_PARAMETERS:
                        raise ValueError(f'comparison {path}: no exact flat controller parameter transport')
                    transported = (_FLAT_CONTROLLER_PARAMETERS[keys[0]],)
                source = original if owner == 'original' else conditions
                target = invariant_original if owner == 'original' else invariant_conditions
                effective = _get(source, location + transported, path)
                validate_schema(effective, schema, path)
                _remove(target, location + transported, path)
            if 'values' in variable and _finite(effective) not in [_finite(v) for v in variable['values']]:
                raise ValueError(f'comparison variable {variable["name"]}: effective value not permitted')
        variables[variable['name']] = {'path': path, 'value': deepcopy(effective)}
    return invariant_original, invariant_conditions, variables
