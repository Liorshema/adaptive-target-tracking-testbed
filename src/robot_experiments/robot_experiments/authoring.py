"""Compile a small scientific specification into the existing revisioned catalog."""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from robot_configuration.yaml_loader import _merge_mappings
from robot_configuration.yaml_loader import load_yaml

from .catalog import definition_hash
from .catalog import ExperimentCatalog
from .components import describe_component
from .components import list_components
from .components import validate_component
from .composition import load_resource
from .composition import materialize_resource
from .composition import resource_label
from .configuration import DEFAULT_EXECUTION
from .configuration import resolve_system_resource

# Public slot -> registry family. Numerical ownership stays with those registries.
SLOTS = {
    name: name
    for name in (
        'robot_estimator',
        'target_estimator',
        'predictor',
        'adaptation',
        'fusion',
        'controller',
        'task',
        'reference_policy',
        'reference_geometry',
    )
}
SLOTS['target_dynamics'] = 'dynamics'
PREDICTION_GROUPS = {'predictor': 'predictors', 'adaptation': 'adaptations', 'fusion': 'fusions'}
PARAMETER_OWNERS = {
    'robot_estimator': ('estimation',),
    'target_estimator': ('estimation',),
    'target_dynamics': ('target', 'target', 'dynamics', 'parameters'),
    'controller': ('strategy', 'controller', 'parameters'),
    'task': ('strategy', 'controller', 'task', 'parameters'),
}


def _owner_parameters(owners, slot, identifier, *, create=True):
    path = (
        ('prediction', PREDICTION_GROUPS[slot], identifier, 'parameters')
        if slot in PREDICTION_GROUPS
        else PARAMETER_OWNERS[slot]
    )
    parent = owners
    for key in path:
        if not create and key not in parent:
            return None
        parent = parent.setdefault(key, {})
    return parent


def _edit_parameters(owners, slot, identifier, edits):
    parameters = _owner_parameters(owners, slot, identifier)
    parameters.update(_merge_mappings(parameters, edits))


def _mapping(value, allowed, label, required=()):
    if not isinstance(value, dict):
        raise ValueError(f'{label}: expected mapping')
    unknown, missing = set(value) - set(allowed), set(required) - set(value)
    if unknown or missing:
        where = label + '.' + sorted(unknown or missing)[0]
        raise ValueError(f'{where}: unknown {sorted(unknown)}; missing {sorted(missing)}')
    return value


def _freeze(value):
    """Detach generated resources as ordinary inline documents, never persist /tmp paths."""
    if isinstance(value, dict):
        return {k: _freeze(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_freeze(v) for v in value]
    if isinstance(value, (str, Path)) and resource_label(str(value)) is not None:
        return {'document': _freeze(load_yaml(value))}
    return deepcopy(value)


class ExperimentAuthor:
    """One-file authoring; actual owner construction and persistence belong to Catalog."""

    def __init__(self, catalog=None):
        self.catalog = catalog if catalog is not None else ExperimentCatalog()

    def discover(self):
        """Expose live deployable registry metadata and existing cases/resources."""
        state = load_yaml(self.catalog.builtin)
        families = list_components()
        return {
            'base_cases': self.catalog.list(),
            'components': {
                slot: [d for d in families[family] if d['capabilities'].get('deployable', True)]
                for slot, family in SLOTS.items()
            },
            'scenarios': sorted(state.get('direct_launch', {}).get('cases', {})),
            'resources': sorted(state.get('resources', {})),
        }

    def _read(self, source):
        if isinstance(source, (str, Path)):
            path = Path(source).expanduser().resolve()
            raw = path.read_bytes()
            document = load_yaml(path)
            origin = str(path)
        else:
            document = deepcopy(source)
            raw = json.dumps(document, sort_keys=True, allow_nan=False).encode()
            origin = '<mapping>'
        _mapping(
            document,
            {'schema_version', 'experiment'},
            'authoring',
            {'schema_version', 'experiment'},
        )
        if type(document['schema_version']) is not int or document['schema_version'] != 1:
            raise ValueError('authoring.schema_version: expected integer 1')
        ex = _mapping(
            document['experiment'],
            {
                'id',
                'name',
                'base_case',
                'base_revision',
                'base_variant',
                'scenario',
                'components',
                'estimation_bindings',
                'parameters',
                'resources',
                'execution',
                'runtime',
                'advanced',
            },
            'experiment',
            {'id', 'name', 'base_case'},
        )
        self.catalog._identifier(ex['id'])
        if not isinstance(ex['name'], str) or not ex['name'].strip():
            raise ValueError('experiment.name: expected nonempty display name')
        return ex, {'source': origin, 'sha256': hashlib.sha256(raw).hexdigest()}

    def _resource(self, reference):
        if isinstance(reference, str):
            state = load_yaml(self.catalog.builtin)
            if reference not in state.get('resources', {}):
                raise ValueError(f'experiment.resources: unknown named resource {reference!r}')
            return {
                'package': 'robot_experiments',
                'path': 'config/experiments.yaml',
                'group': 'resources.' + reference,
            }
        return deepcopy(reference)

    def compile(self, source):  # noqa: A003 - public compiler API.
        """Return a detached canonical definition/owner overrides, without writing anything."""
        ex, provenance = self._read(source)
        try:
            base = self.catalog.get(ex['base_case'], ex.get('base_revision'))
            base_spec = self.catalog.prepare(ex['base_case'], base['revision'])
        except (KeyError, ValueError, TypeError) as error:
            raise ValueError(f'experiment.base_case: {error}') from error
        variants = base_spec['variants']
        chosen = ex.get('base_variant')
        matches = [v for v in variants if chosen is None or v['name'] == chosen]
        if len(matches) != 1:
            raise ValueError('experiment.base_variant: choose one existing base-case variant')
        variant = matches[0]
        canonical = deepcopy(base['definition'])
        if canonical['schema_version'] != 1:
            raise ValueError(
                'experiment.base_case: choose a schema-1 single-condition study; '
                'schema-2 comparisons remain available through canonical CLI'
            )
        study = canonical['experiment']
        raw_variant = next(v for v in study['variants'] if v['name'] == variant['name'])
        reference = deepcopy(raw_variant['system_config'])
        if 'scenario' in ex:
            cases = load_yaml(self.catalog.builtin).get('direct_launch', {}).get('cases', {})
            if not isinstance(ex['scenario'], str) or ex['scenario'] not in cases:
                raise ValueError('experiment.scenario: unknown deployed scenario')
            reference = deepcopy(cases[ex['scenario']])
        manifest_path = resolve_system_resource(
            reference, self.catalog.path.parent, self.catalog.share
        )
        manifest = load_yaml(manifest_path)
        deployment = load_yaml(
            materialize_resource(
                manifest['simulation']['config'], manifest_path.parent, self.catalog.share
            )
        )
        target_path = materialize_resource(
            deployment['target']['config'], manifest_path.parent, self.catalog.share
        )
        target = load_yaml(target_path)
        # A scenario can have different task/geometry/parameters from the base study.
        from robot_bringup.factory.system_configuration import load_system_configuration

        plan = load_system_configuration(manifest_path, self.catalog.share)
        strategy = deepcopy(plan['strategy'])
        selected = {
            **{k.removesuffix('_type'): v for k, v in plan['selection'].items()},
            'controller': plan['control_parameters']['controller_type'],
            'target_dynamics': target['target']['dynamics']['type'],
            'reference_policy': manifest['control']['reference']['policy'],
            'reference_geometry': manifest['control']['reference']['geometry'],
        }
        if strategy and 'task' in strategy['controller']:
            selected['task'] = strategy['controller']['task']['type']
        else:
            candidates = [
                d['id']
                for d in list_components('task')
                if d['capabilities'].get('selection_parameter')
            ]
            if len(candidates) != 1:
                raise ValueError('experiment.components.task: explicit typed task required')
            selected['task'] = candidates[0]
        components = _mapping(ex.get('components', {}), SLOTS, 'experiment.components')
        parameters = _mapping(ex.get('parameters', {}), SLOTS, 'experiment.parameters')
        inherited = deepcopy(selected)
        selected.update(components)
        changed = {slot for slot in components if selected[slot] != inherited[slot]}
        edited = {slot for slot, value in parameters.items() if value}
        details = {}
        for slot in SLOTS:
            label = 'experiment.components.' + slot
            try:
                if not isinstance(selected[slot], str):
                    raise ValueError('expected registered string ID; use the ID "none" to disable')
                metadata = describe_component(SLOTS[slot], selected[slot])
                if not metadata['capabilities'].get('deployable', True):
                    raise ValueError('library-only component is not deployable')
                values = _mapping(
                    parameters.get(slot, {}),
                    metadata['schema'].get('properties', {}),
                    'experiment.parameters.' + slot,
                )
                if slot.endswith('_estimator') and any(
                    key.endswith('_algorithm_parameters') for key in values
                ):
                    raise ValueError(
                        'frozen algorithm transport is reserved; use algorithm_parameters')
                validation = validate_component(SLOTS[slot], selected[slot], values)
                if not validation['valid']:
                    raise ValueError('; '.join(validation['errors']))
            except (ValueError, KeyError, TypeError) as error:
                location = 'experiment.parameters.' + slot if slot in parameters else label
                raise ValueError(f'{location}: {error}') from error
            details[slot] = {
                'id': selected[slot],
                'defaults': metadata['defaults'],
                'overrides': deepcopy(values),
                'advanced_owner_overrides_present': bool(ex.get('advanced')),
                'selection_source': (
                    {'kind': 'experiment_selection', 'source': provenance['source']}
                    if slot in components else
                    {'kind': 'scenario', 'name': ex['scenario'], 'source': provenance['source']}
                    if 'scenario' in ex else
                    {'kind': 'inherited_base', 'id': base['id'],
                     'revision': base['revision'], 'sha256': base['sha256']}
                ),
            }
        changes, owners = {}, deepcopy(base['overrides'])
        for slot in changed & (PARAMETER_OWNERS.keys() | PREDICTION_GROUPS.keys()):
            previous = _owner_parameters(owners, slot, inherited[slot], create=False)
            if previous is not None and slot.endswith('_estimator'):
                old = describe_component(SLOTS[slot], inherited[slot])['capabilities']
                new = describe_component(SLOTS[slot], selected[slot])['capabilities']
                if old.get('binding') is not None and all(
                    old.get(key) == new.get(key)
                    for key in (
                        'binding',
                        'process_model',
                        'measurement_model',
                        'deployment_adapter',
                    )
                ):
                    continue  # Algorithm changes preserve compatible inherited physical models.
            if previous is not None:
                # Estimator roles share one flat owner map; clear only that role's keys.
                keys = (
                    describe_component(SLOTS[slot], inherited[slot])['schema']['properties']
                    if slot.endswith('_estimator')
                    else list(previous)
                )
                for key in keys:
                    previous.pop(key, None)
        # Selections use existing manifest contracts; parameter owners remain unchanged.
        bindings = _mapping(
            ex.get('estimation_bindings', {}),
            {'robot', 'target'},
            'experiment.estimation_bindings',
        )
        for role, value in bindings.items():
            _mapping(
                value,
                {'process_model', 'measurement_model', 'adapter'},
                'experiment.estimation_bindings.' + role,
                {'process_model', 'measurement_model'},
            )
            for key, identifier in value.items():
                if not isinstance(identifier, str) or not identifier.strip():
                    raise ValueError(
                        'experiment.estimation_bindings: expected registered model IDs'
                    )
                field = 'deployment_adapter' if key == 'adapter' else key
                changes.setdefault('estimation', {}).setdefault(role, {}).setdefault(
                    'parameters', {}
                )[role + '_' + field] = identifier
        from robot_estimation.factory.algorithm_parameters import (
            decode_algorithm_parameters, encode_algorithm_parameters,
        )
        from robot_estimation.factory.estimator_factory import create_estimation_registries

        registries = create_estimation_registries()
        for slot, role in [('robot_estimator', 'robot'), ('target_estimator', 'target')]:
            metadata = registries[slot].describe(selected[slot])
            algorithm = metadata['capabilities']['algorithm']
            previous_algorithm = registries[slot].describe(inherited[slot])['capabilities'][
                'algorithm']
            key = role + '_algorithm_parameters'
            intrinsic = {}
            if algorithm == previous_algorithm:
                encoded = owners.get('estimation', {}).get(
                    key, plan['estimation_parameters'][key])
                intrinsic = decode_algorithm_parameters(
                    encoded, algorithm, registries['estimator'])
            edits = deepcopy(parameters.get(slot, {}))
            intrinsic = registries['estimator'].validate(
                algorithm, _merge_mappings(intrinsic, edits.pop('algorithm_parameters', {})))
            _edit_parameters(owners, slot, selected[slot], {
                key: encode_algorithm_parameters(algorithm, intrinsic)})
            if slot in changed or slot in edited:
                changes.setdefault('estimation', {}).setdefault(role, {})['type'] = selected[slot]
                if slot in parameters:
                    values = edits
                    for field in ('process_model', 'measurement_model', 'deployment_adapter'):
                        key = role + '_' + field
                        if key in values:
                            target = changes['estimation'][role].setdefault('parameters', {})
                            if key in target and target[key] != values[key]:
                                raise ValueError(
                                    'experiment.estimation_bindings: conflicting parameter binding'
                                )
                            target[key] = values.pop(key)
                    _edit_parameters(owners, slot, selected[slot], values)
        if any(s in changed or s in edited for s in PREDICTION_GROUPS):
            changes['prediction'] = {
                k: selected[s]
                for s, k in [
                    ('predictor', 'type'),
                    ('adaptation', 'adaptation'),
                    ('fusion', 'fusion'),
                ]
            }
            if any(s in edited for s in PREDICTION_GROUPS):
                prediction = load_resource(
                    manifest['prediction']['config'],
                    base=manifest_path.parent,
                    share=self.catalog.share,
                )
                for slot, group in PREDICTION_GROUPS.items():
                    if slot in edited:
                        # Existing owner entries supply the runtime pathway. Do not pretend
                        # that a newly registered family automatically has a ROS adapter.
                        if selected[slot] not in prediction[group]:
                            raise ValueError(
                                f'experiment.parameters.{slot}: selected component '
                                'has no deployed owner parameter block'
                            )
                        _edit_parameters(owners, slot, selected[slot], parameters[slot])
        control_slots = {'controller', 'task'}
        if control_slots & (changed | edited):
            current = deepcopy(strategy) if strategy is not None else {'controller': {}}
            controller = current['controller']
            if 'controller' in changed:
                controller['parameters'] = deepcopy(details['controller']['defaults'])
                controller['uses_safety_bounds'] = (
                    'safety'
                    in describe_component('controller', selected['controller'])['dependencies']
                )
            controller['type'] = selected['controller']
            controller.setdefault('parameters', deepcopy(details['controller']['defaults']))
            if 'task' in changed or 'task' not in controller:
                controller['task'] = {
                    'type': selected['task'],
                    'parameters': deepcopy(details['task']['defaults']),
                }
            task_parameters = deepcopy(parameters.get('task', {}))
            metadata = describe_component('controller', selected['controller'])
            if metadata['capabilities'].get('task_binding') == 'selection_matrix':
                task_metadata = describe_component('task', selected['task'])
                selection_key = task_metadata['capabilities'].get('selection_parameter')
                if selection_key is None:
                    raise ValueError(
                        'experiment.components.task: incompatible controller task binding'
                    )
                from robot_control.factory.project_configuration import configured_control

                description = load_yaml(plan['description_path'])
                baseline = configured_control(plan['control_parameters'], description)
                controller['task_selection'] = baseline['controller']['task_selection']
                if selection_key in task_parameters:
                    owners.setdefault('strategy', {}).setdefault('controller', {})[
                        'task_selection'
                    ] = task_parameters.pop(selection_key)
                del controller['task']
            if 'controller' in parameters:
                _edit_parameters(
                    owners, 'controller', selected['controller'], parameters['controller']
                )
            if task_parameters:
                _edit_parameters(owners, 'task', selected['task'], task_parameters)
            changes['control'] = {
                'type': selected['controller'],
                'strategy': {'document': _freeze(current)},
            }
        for slot, key in [('reference_policy', 'policy'), ('reference_geometry', 'geometry')]:
            if slot in changed:
                changes.setdefault('control', {}).setdefault('reference', {})[key] = selected[slot]
            if parameters.get(slot):
                raise ValueError(
                    f'experiment.parameters.{slot}: deployment currently supports '
                    'ID selection only; use advanced owner overrides for shared tracking'
                )
        if 'target_dynamics' in changed:
            target['target']['dynamics'] = {
                'type': selected['target_dynamics'],
                'parameters': deepcopy(details['target_dynamics']['defaults']),
            }
            deployment['target']['config'] = {'document': _freeze(target)}
        if 'target_dynamics' in parameters:
            _edit_parameters(
                owners,
                'target_dynamics',
                selected['target_dynamics'],
                parameters['target_dynamics'],
            )
        resources = _mapping(
            ex.get('resources', {}), {'robot', 'environment', 'sensing'}, 'experiment.resources'
        )
        if 'robot' in resources:
            robot = _mapping(
                resources['robot'],
                {'description', 'deployment'},
                'experiment.resources.robot',
                {'description', 'deployment'},
            )
            changes['robot'] = {'description': self._resource(robot['description'])}
            gazebo = self._resource(robot['deployment'])
            changes.setdefault('simulation', {})['robot'] = gazebo
            deployment['robot']['config'] = gazebo
        if 'environment' in resources:
            deployment['simulation']['environment_config'] = self._resource(
                resources['environment']
            )
        if 'sensing' in resources:
            sensing = _mapping(
                resources['sensing'],
                {'measurements', 'localization'},
                'experiment.resources.sensing',
            )
            if 'measurements' in sensing:
                deployment['simulation']['measurement_config'] = self._resource(
                    sensing['measurements']
                )
            if 'localization' in sensing:
                value = sensing['localization']
                if value is None:
                    deployment.pop('localization_sensor_config', None)
                else:
                    value = self._resource(value)
                    deployment['localization_sensor_config'] = value
                changes.setdefault('simulation', {})['localization_sensor'] = value
        if resources or 'target_dynamics' in changed:
            changes.setdefault('simulation', {})['config'] = {'document': _freeze(deployment)}
        # Freeze the already resolved base, including historical Catalog aliases.
        # Re-embedding a removed legacy filename would fail inside a nested recipe.
        recipe = {'base': _freeze(str(manifest_path)), 'overrides': changes}
        if changes.get('control', {}).get('strategy') is not None:
            stale = [
                key
                for key in ('strategy_group', 'strategy_overrides')
                if key in manifest['control']
            ]
            if stale:
                recipe['omit'] = ['control.' + key for key in stale]
        study['variants'] = [{'name': variant['name'], 'system_config': recipe}]
        execution = _mapping(
            ex.get('execution', {}),
            {*DEFAULT_EXECUTION, 'duration', 'settling_time', 'seeds'},
            'experiment.execution',
        )
        for key, value in execution.items():
            if key in {'duration', 'settling_time', 'seeds'}:
                study[key] = deepcopy(value)
            else:
                study.setdefault('execution', {})[key] = deepcopy(value)
        if 'runtime' in ex:
            study['runtime'] = _merge_mappings(
                study.get('runtime', {}),
                _mapping(ex['runtime'], {'enable_actuation', 'gui'}, 'experiment.runtime'),
            )
        advanced = _mapping(ex.get('advanced', {}), {'overrides'}, 'experiment.advanced')
        if 'overrides' in advanced:
            owners = _merge_mappings(
                owners,
                _mapping(
                    advanced['overrides'],
                    {
                        'prediction',
                        'estimation',
                        'control',
                        'strategy',
                        'target',
                        'environment',
                        'integration',
                        'measurements',
                        'localization_sensor',
                    },
                    'experiment.advanced.overrides',
                ),
            )
        provenance['base'] = {key: base[key] for key in ('id', 'revision', 'sha256')}
        study['description'] = (
            study.get('description', '')
            + '\nAuthoring provenance: '
            + json.dumps(provenance, sort_keys=True)
        )
        return {
            'id': ex['id'],
            'name': ex['name'],
            'definition': canonical,
            'overrides': owners,
            'provenance': provenance,
            'components': details,
        }

    def validate(self, compiled):
        """Use exactly the catalog's detached preparation and owner validation path."""
        record = {
            **compiled,
            'revision': 1,
            'sha256': definition_hash(compiled['definition'], compiled['overrides']),
        }
        try:
            return self.catalog._prepare(record)
        except (ValueError, TypeError, KeyError, OSError) as error:
            paths = [
                'experiment.parameters.' + slot
                for slot, detail in compiled['components'].items()
                if detail['overrides']
            ]
            raise ValueError(
                ' / '.join(
                    [
                        'experiment.components',
                        *paths,
                        'experiment.resources',
                        'experiment.estimation_bindings',
                        'experiment.execution',
                    ]
                )
                + f': {error}'
            ) from error

    def preview(self, source):
        """Construct selected owners and show inherited defaults versus authored changes."""
        compiled = self.compile(source)
        spec = self.validate(compiled)
        from .explain import explain_experiment

        effective = explain_experiment(spec, self.catalog.share)
        details = compiled['components']
        # The explanation is the seeded, owner-validated source of effective values.
        run = effective['runs'][0]
        details['target_dynamics']['effective'] = run['scenario']['target_dynamics']['target'][
            'dynamics'
        ]['parameters']
        for slot, group in PREDICTION_GROUPS.items():
            details[slot]['effective'] = run['prediction']['configuration'][group].get(
                details[slot]['id'], {'parameters': details[slot]['defaults']}
            )['parameters']
        for slot in ('robot_estimator', 'target_estimator'):
            schema = describe_component(slot, details[slot]['id'])['schema']['properties']
            details[slot]['effective'] = {
                k: v for k, v in run['estimation']['parameters'].items() if k in schema
            }
            key = slot.removesuffix('_estimator') + '_algorithm_parameters'
            encoded = details[slot]['effective'].pop(key)
            details[slot]['effective']['algorithm_parameters'] = json.loads(encoded)['parameters']
        from robot_control.factory.project_configuration import configured_control
        from .reporting import plain

        control = configured_control(
            run['control']['parameters'],
            load_yaml(spec['variants'][0]['plan']['description_path']),
            run['control']['strategy'],
        )
        details['controller']['effective'] = plain(control['controller']['parameters'])
        details['task']['effective'] = plain(
            control['controller']
            .get('task', {})
            .get('parameters', {'selection_matrix': control['controller'].get('task_selection')})
        )
        for slot, key in [('reference_policy', 'policy'), ('reference_geometry', 'geometry')]:
            details[slot]['effective'] = plain(control['tracking'][key].get('parameters', {}))
        return {
            'id': compiled['id'],
            'name': compiled['name'],
            'valid': True,
            'provenance': compiled['provenance'],
            'components': details,
            'identities': spec['variants'][0]['plan']['identities'],
            'summary': (
                f"{compiled['id']}: {compiled['name']}\n"
                f"Base {compiled['provenance']['base']['id']} revision "
                f"{compiled['provenance']['base']['revision']}; "
                f"duration {spec['duration']} s; seeds {spec['seeds']}; "
                f"actuation {spec['runtime']['enable_actuation']}\n"
                + '\n'.join(f"{slot}: {detail['id']}" for slot, detail in details.items())
            ),
            'effective': effective,
        }

    def save(self, source, *, expected_revision=None):
        """Create or compare-and-swap a revision using existing Catalog CRUD."""
        compiled = self.compile(source)
        self.validate(compiled)
        if expected_revision is None:
            return self.catalog.create(
                compiled['id'], compiled['name'], compiled['definition'], compiled['overrides']
            )
        return self.catalog.revise(
            compiled['id'],
            compiled['definition'],
            compiled['overrides'],
            expected_revision=expected_revision,
        )


def main(argv, *, share=None):
    """Headless discovery/preview/save; execution remains the existing experiment CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['discover', 'preview', 'validate', 'save'])
    parser.add_argument('source', type=Path, nargs='?')
    parser.add_argument('--catalog', type=Path)
    parser.add_argument('--expected-revision', type=int)
    parser.add_argument(
        '--details', action='store_true', help='Include full seeded backend explanation'
    )
    args = parser.parse_args(argv)
    try:
        api = ExperimentAuthor(ExperimentCatalog(args.catalog, share))
        if args.action == 'discover':
            result = api.discover()
        else:
            if args.source is None:
                raise ValueError('source: provide one simple experiment YAML file')
            if args.action == 'save':
                result = api.save(args.source, expected_revision=args.expected_revision)
            elif args.action == 'validate':
                compiled = api.compile(args.source)
                spec = api.validate(compiled)
                result = {
                    'valid': True,
                    'id': compiled['id'],
                    'identities': spec['variants'][0]['plan']['identities'],
                }
            else:
                result = api.preview(args.source)
                if not args.details:
                    result.pop('effective')
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    except (ValueError, TypeError, KeyError, OSError) as error:
        print(json.dumps({'error': str(error)}))
        return 2
