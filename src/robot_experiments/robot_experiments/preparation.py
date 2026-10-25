"""Finish materialized run preparation before any runtime ownership is acquired."""

import hashlib
from pathlib import Path

from .comparison import METRIC_NAMES, comparison_views
from .metrics import evaluate
from .reporting import plain


def _metric_definition_hash():
    """Identify installed metric definitions without depending on a Git checkout."""
    root = Path(evaluate.__code__.co_filename).parent
    digest = hashlib.sha256()
    for path in sorted(root.rglob('*.py')):
        digest.update(str(path.relative_to(root)).encode() + b'\0' + path.read_bytes() + b'\0')
    return digest.hexdigest()


def complete_preparation(spec: dict, prepared: dict, source_sha256: str | None) -> None:
    """Attach the same scoring/comparison inputs for execution and explanation.

    Mutate the materializer's detached result, retaining partial contract evidence
    if comparison validation raises. No processes, clocks or measurements are used.
    """
    context, plan = prepared['context'], prepared['plan']
    context['readiness_max_age'] = spec['execution']['max_skew']
    prepared['comparison_contract'] = {
        **prepared['comparison_contract'], 'duration': spec['duration'],
        'settling_time': spec['settling_time'], 'max_skew': spec['execution']['max_skew'],
        'physical_tolerance': context['physical_numerical_tolerance'],
        'enable_actuation': spec['runtime']['enable_actuation']}
    conditions = {
        **prepared.get('scientific_conditions', {}),
        'experiment_setup': {'name': spec['name'], 'seeds': spec['seeds'],
                             'execution': spec['execution'], 'runtime': spec['runtime']},
        'evaluation': {
            'requested_metrics': spec['metrics'],
            'required_metrics': [metric for name in spec['metrics']
                                 for metric in METRIC_NAMES.get(name, [name])],
            'physical_coverage': 'existing complete joint observation coverage',
            'definition_sha256': _metric_definition_hash(),
            'source_sha256': source_sha256, 'metric_schema_version': 1}}
    prepared['comparison'] = comparison_views(
        plain(prepared['comparison_contract']), spec.get('comparison'), plain(conditions))
    prepared['configuration_identity'] = {
        'system_manifest': str(plan['source']), 'component_types': plan['identities'],
        'resolved_configuration': 'resolved_configuration.json'}
