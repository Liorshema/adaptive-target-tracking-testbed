"""Translate only imports when executing immutable archived estimator evidence.

No legacy package or runtime alias is installed. Archived file bytes, equations
and provenance remain unchanged; validation loaders call this before compilation.
"""

import re

IMPORTS = {
    'robot_estimation.common.estimator_interface': 'robot_estimation.interfaces.estimator_interface',
    'robot_estimation.common.measurement': 'robot_estimation.measurement',
    'robot_estimation.common.state': 'robot_estimation.state',
    'robot_estimation.common.timestamp_buffer': 'robot_estimation.timestamp_buffer',
    'robot_estimation.filters.filter_interface': 'robot_estimation.interfaces.filter_interface',
}


def canonicalize_imports(source):
    """Translate qualified from/import statements without changing numerical code."""
    for old, new in IMPORTS.items():
        pattern = r'(?m)^(\s*(?:from|import)\s+)' + re.escape(old) + r'(?=[\s.])'
        source = re.sub(pattern, lambda match: match[1] + new, source)
    return source
