"""Inspect seeded, validated run inputs without creating a runtime session."""

from pathlib import Path
from tempfile import TemporaryDirectory

from .comparison import comparison_reasons
from .configuration import materialize_run
from .preparation import complete_preparation
from .reporting import plain


def _display(value, directory: Path):
    """Label transient paths only in display metadata; never rewrite comparison views."""
    if isinstance(value, Path):
        value = str(value)
    if isinstance(value, str):
        from .composition import resource_label
        label = resource_label(value)
        if label is not None:
            return label
        prefix = str(directory)
        if value == prefix or value.startswith(prefix + '/'):
            return '<materialized>' + value[len(prefix):]
        return value
    if isinstance(value, dict):
        return {_display(k, directory): _display(v, directory) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_display(v, directory) for v in value]
    return plain(value)


def _variables(view: dict, resources: dict, selected: dict, assignments: dict) -> dict:
    """Relate effective leaves to selected captured owner documents, not per-key ancestry."""
    declarations = {v['name']: v for v in (view['policy'] or {}).get('variables', [])}
    result = {}
    for name, variable in view['variables'].items():
        owners = []
        for source in selected.values():
            if source not in resources:
                continue
            value = resources[source]['resolved']
            try:
                # Contract's outer key names the captured owner document.
                for key in variable['path'].split('.')[1:]:
                    value = value[key]
                if value == variable['value']:
                    owners.append(source)
            except (KeyError, TypeError):
                continue
        result[name] = {**variable, 'declared_values': declarations[name].get('values'),
                        'owner_resources': sorted(set(owners)),
                        'expectation': 'permitted values, not a per-variant assignment'}
        if name in assignments:
            result[name]['assignment'] = assignments[name]
            result[name]['owner_resources'] = [assignments[name]['owner_source']]
            result[name]['expectation'] = 'permitted values; explicit assignment shown separately'
    return result


def _describe(spec: dict, variant: dict, seed: int, prepared: dict, directory: Path) -> dict:
    """Project existing effective dictionaries; selection is never inferred from filenames."""
    plan, snapshot = prepared['plan'], prepared['snapshot']
    source_plan = variant['plan']
    view = prepared['comparison']
    contract, conditions = view['original_contract'], view['protected_conditions']
    selected = plain(source_plan['sensor_inventory']['configuration_resources'])
    resources = snapshot['resources']
    result = {
        'variant': variant['name'], 'seed': seed,
        'system': {'source': str(variant['system_config']),
                   'profile_source': str(source_plan['profile_path']),
                   'selection': plan['selection'], 'identities': plan['identities']},
        'scenario': {
            'source': str(source_plan['profile']['simulation_config']),
            'target_dynamics': contract['target_dynamics'], 'environment': contract['environment'],
            'initial_conditions': snapshot['initial_conditions'], 'tracking': contract['tracking'],
            'physical_context': contract['physical_context'],
            'integration': contract['integration'],
            'physics': conditions['physics'],
            'gazebo_transport': conditions['gazebo_transport'],
            'terrain': contract['terrain'], 'world_sha256': contract['world_sha256']},
        'robot': {'description_source': str(source_plan['description_path']),
                  'description': contract['description'], 'deployment': contract['robot'],
                  'holding': contract['holding']},
        'estimation': {'implementations': {key: plan['identities'][key]
                                           for key in ['robot_estimator', 'target_estimator']},
                       'parameters': plan['estimation_parameters']},
        'prediction': {'source': str(source_plan['prediction_path']),
                       'implementations': {key: plan['identities'][key]
                                           for key in ['predictor', 'adaptation', 'fusion']},
                       'configuration': conditions['prediction_configuration']},
        'control': {'implementations': {key: plan['identities'][key]
                                        for key in ['controller', 'model', 'safety']},
                    'parameters': plan['control_parameters'],
                    'strategy_source': (str(source_plan['strategy_path'])
                                        if source_plan['strategy_path'] else None),
                    'strategy': conditions['controller_strategy'],
                    'actuation': contract['actuation'], 'runtime': plan['runtime']},
        'sensors': plan['sensor_inventory'],
        'seed_consumers': snapshot['seed_consumers'],
        'runtime': {'settings': plan['runtime'], 'execution': spec['execution'],
                    'window': {'duration': spec['duration'],
                               'settling_time': spec['settling_time'],
                               'unit': 's', 'start': None, 'end': None,
                               'basis': 'readiness-relative simulation source epoch'}},
        'sources': {
            'selected_resources': selected,
            'resources': {source: {'sha256': record['sha256'],
                                   'parents': record.get('parents', [])}
                          for source, record in resources.items()},
            'scope': ('Selected resources, inheritance ancestry and final M07 calibration owners; '
                      'not full per-key declaring ancestry. <materialized> labels deleted '
                      'temporary input paths, not persistent files.'),
            'overrides': {'launch_runtime': plan['runtime'],
                          'owner_parameters': variant.get('owner_overrides', {}),
                          'seed_consumers': snapshot['seed_consumers'],
                          'parameter_assignments': snapshot.get(
                              'assignments', 'not supported in schema 1')}},
        'experimental_variables': _variables(
            view, resources, selected, snapshot.get('assignments', {})),
        'configuration_validation': {'passed': True, 'basis': 'materialized M07 owner preflight'},
    }
    if 'composition' in source_plan:
        result['system']['composition'] = source_plan['composition']
    result = _display(result, directory)
    # Original contracts and hashes remain byte-for-byte factual, not display-normalized.
    result['comparison'] = {**view, 'runtime_eligibility': None,
                            'runtime_status': 'not evaluated; no experiment was launched'}
    return result


def explain_experiment(spec: dict, share) -> dict:
    """Prepare all inputs, then return one detached document after deterministic cleanup."""
    from .runner import _git
    source_sha256 = _git().get('worktree_source_sha256')
    runs, checks = [], []
    variants = [variant['name'] for variant in spec['variants']]
    with TemporaryDirectory(prefix='robot_experiment_explain_') as temporary:
        for seed in spec['seeds']:
            for variant in spec['variants']:
                directory = Path(temporary) / f'{variant["name"]}_seed{seed}'
                directory.mkdir()
                prepared = materialize_run(
                    variant, seed, directory, share, spec['runtime'],
                    record_actuator_trace=spec['execution']['record_actuator_trace'])
                complete_preparation(spec, prepared, source_sha256)
                runs.append(_describe(spec, variant, seed, prepared, directory))
                checks.append({'variant': variant['name'], 'seed': seed,
                               'comparison_contract': prepared['comparison']['original_hash'],
                               'comparison': prepared['comparison']})
        reasons = comparison_reasons(checks, variants, spec['seeds'], spec.get('comparison'),
                                     configuration_only=True)
        for run in runs:
            paired = [check for check in checks if check['seed'] == run['seed']]
            rejected = comparison_reasons(paired, variants, [run['seed']], spec.get('comparison'),
                                          configuration_only=True)
            run['comparison']['configuration_compatibility'] = {
                'compatible': not rejected, 'rejection_reasons': rejected}
        result = {
            'schema_version': 1, 'mode': 'explain',
            'experiment': {key: plain(value) for key, value in spec.items() if key != 'variants'},
            'runs': runs,
            'comparison': {'policy': spec.get('comparison'),
                           'configuration_compatibility': {
                               'compatible': not reasons, 'rejection_reasons': reasons},
                           'runtime_eligibility': None,
                           'runtime_status': ('pending measurements, completion, metrics '
                                              'and coverage')},
        }
    return result
