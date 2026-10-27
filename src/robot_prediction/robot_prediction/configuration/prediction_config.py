"""Canonical predictor-family defaults with one historical document projection."""

from copy import deepcopy
from pathlib import Path

from robot_configuration.components import schema_for
from robot_configuration.components import validate_schema
from robot_configuration.yaml_loader import load_yaml

# This is a structural translation, never a second table of numerical defaults.
LEGACY_PATHS = {
    'prediction': ('prediction', 'parameters'),
    'history': ('monitoring', 'parameters'),
    'confidence': ('monitoring', 'parameters'),
    'imm': ('predictors', 'imm', 'parameters'),
    'coordinated_turn': ('predictors', 'coordinated_turn', 'parameters'),
    'fusion': ('fusions', 'baseline', 'parameters'),
}


PARAMETER_SCHEMAS = {
    ('prediction',): {
        'dt': {'type': 'number', 'exclusiveMinimum': 0.0},
        'horizon': {'type': 'integer', 'minimum': 1},
    },
    ('monitoring',): {
        'window_duration': {'type': 'number', 'exclusiveMinimum': 0.0},
        'residual_scale': {'type': 'number', 'exclusiveMinimum': 0.0},
    },
    ('predictors', 'constant_velocity'): {},
    ('predictors', 'constant_acceleration'): {},
    ('predictors', 'coordinated_turn'): {
        'turn_rate_epsilon': {'type': 'number', 'exclusiveMinimum': 0.0}
    },
    ('predictors', 'imm'): {
        'model_probabilities': {
            'type': 'array',
            'minItems': 3,
            'maxItems': 3,
            'items': {'type': 'number', 'minimum': 0.0},
        },
        'transition_matrix': {
            'type': 'array',
            'minItems': 3,
            'maxItems': 3,
            'items': {
                'type': 'array',
                'minItems': 3,
                'maxItems': 3,
                'items': {'type': 'number', 'minimum': 0.0},
            },
        },
        'likelihood_variance': {'type': 'number', 'exclusiveMinimum': 0.0},
    },
    ('adaptations', 'residual'): {'correction_gain': {'type': 'number', 'minimum': 0.0}},
    ('adaptations', 'rls'): {
        'forgetting_factor': {'type': 'number', 'exclusiveMinimum': 0.0, 'maximum': 1.0},
        'initial_covariance': {'type': 'number', 'exclusiveMinimum': 0.0},
    },
    ('fusions', 'weighted_correction'): {},
    ('fusions', 'baseline'): {
        'default_weights': {
            'type': 'array',
            'minItems': 1,
            'items': {'type': 'number', 'minimum': 0.0},
        }
    },
}


def normalize_prediction_overrides(overrides):
    """Translate historical partial edits without supplying any default values."""
    if not isinstance(overrides, dict):
        raise ValueError('prediction overrides must be a mapping')
    result = {}
    for name, value in overrides.items():
        if name in {'monitoring', 'predictors', 'adaptations', 'fusions'} or (
            name == 'prediction' and isinstance(value, dict) and 'parameters' in value
        ):
            if name in result:
                raise ValueError('mixed/conflicting prediction override paths')
            result[name] = deepcopy(value)
            continue
        if name == 'adaptation':
            if 'adaptations' in overrides:
                raise ValueError('mixed/conflicting adaptation override paths')
            result['adaptations'] = {key: {'parameters': deepcopy(v)} for key, v in value.items()}
            continue
        if name not in LEGACY_PATHS:
            raise ValueError(f'unknown prediction override section {name!r}')
        path = LEGACY_PATHS[name]
        if path[0] in overrides and path[0] != name:
            raise ValueError('mixed/conflicting prediction override paths')
        parent = result
        for key in path[:-1]:
            parent = parent.setdefault(key, {})
        parent.setdefault(path[-1], {}).update(deepcopy(value))
    return result


def canonical_prediction(document):
    """Read canonical data or explicitly translate a complete legacy snapshot."""
    data = deepcopy(document)
    if 'predictors' not in data:
        data = normalize_prediction_overrides(data)
        for name in ['constant_velocity', 'constant_acceleration']:
            data['predictors'][name] = {'parameters': {}}
        data['fusions']['weighted_correction'] = {'parameters': {}}
    return data


class PredictionConfig:
    """Validate all declared families and expose detached canonical/default views."""

    def __init__(self, config_path):
        self._config_path = Path(config_path)
        self._config = canonical_prediction(load_yaml(self._config_path))
        self._validate()

    def _validate(self):
        data = self._config
        required = {'prediction', 'monitoring', 'predictors', 'adaptations', 'fusions'}
        if set(data) != required:
            raise ValueError(
                f'{self._config_path}: missing/unknown prediction sections {set(data) ^ required}'
            )

        for path, properties in PARAMETER_SCHEMAS.items():
            section = data
            for key in path:
                if key not in section:
                    raise ValueError(f"{self._config_path}: missing {'.'.join(path)}")
                section = section[key]
            label = '.'.join(path)
            if not isinstance(section, dict) or set(section) != {'parameters'}:
                raise ValueError(f'{label}: expected only a parameters mapping')
            validate_schema(
                section['parameters'],
                {
                    'type': 'object',
                    'properties': properties,
                    'required': list(properties),
                    'additionalProperties': False,
                },
                label,
            )
        # Extra registered component families may provide their own schema at construction.
        for family in ['predictors', 'adaptations', 'fusions']:
            for name, entry in data[family].items():
                if not isinstance(entry, dict) or set(entry) != {'parameters'}:
                    raise ValueError(f'{family}.{name}: expected parameters mapping')
                validate_schema(
                    entry['parameters'], schema_for(entry['parameters']), family + '.' + name
                )
        import numpy as np

        imm = self.get('imm')
        if sum(imm['model_probabilities']) <= 0:
            raise ValueError('imm.model_probabilities: positive total required')
        if not np.allclose(np.sum(imm['transition_matrix'], axis=1), 1.0):
            raise ValueError('imm.transition_matrix: rows must sum to one')
        if sum(self.get('fusion')['default_weights']) <= 0:
            raise ValueError('fusion.default_weights: positive total required')

    def get(self, section):
        """Preserve the historical node/preflight view without duplicating defaults."""
        if section == 'adaptation':
            return {
                key: deepcopy(value['parameters'])
                for key, value in self._config['adaptations'].items()
            }
        if section in LEGACY_PATHS:
            value = self._config
            for key in LEGACY_PATHS[section]:
                value = value[key]
            if section == 'history':
                value = {'window_duration': value['window_duration']}
            if section == 'confidence':
                value = {'residual_scale': value['residual_scale']}
            return deepcopy(value)
        if section not in self._config:
            raise KeyError(f'missing configuration section: {section}')
        return deepcopy(self._config[section])

    @property
    def data(self):
        """Return canonical immutable-owner values as a detached mapping."""
        return deepcopy(self._config)

    @property
    def legacy_data(self):
        """Return the existing persisted metric/runtime document representation."""
        return {key: self.get(key) for key in [*LEGACY_PATHS, 'adaptation']}
