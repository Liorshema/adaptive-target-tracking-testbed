from pathlib import Path

from robot_configuration.yaml_loader import load_yaml
from robot_configuration.parameter_validation import validate_parameters
import math


class PredictionConfig:
    """Load and provide access to prediction configuration."""

    def __init__(
        self,
        config_path: str | Path,
    ):
        self._config_path = Path(config_path)
        self._config = self._load()

    def _load(self) -> dict:
        """Load configuration from YAML."""
        if not self._config_path.exists():
            raise FileNotFoundError(
                f'configuration file not found: '
                f'{self._config_path}'
            )

        config = load_yaml(self._config_path)
        allowed = {
            'prediction': {'dt', 'horizon', 'type'},
            'history': {'window_duration'}, 'confidence': {'residual_scale'},
            'imm': {'model_probabilities', 'transition_matrix', 'likelihood_variance'},
            'coordinated_turn': {'turn_rate_epsilon'},
            'fusion': {'type', 'default_weights'},
            'adaptation': {'type', 'residual', 'rls'},
        }
        if set(config) - set(allowed):
            raise ValueError(f'{self._config_path}: unknown prediction sections {set(config)-set(allowed)}')
        for section, value in config.items():
            if not isinstance(value, dict) or set(value) - allowed[section]:
                raise ValueError(f'{self._config_path}:{section}: unsupported keys or non-mapping section')
        for section, keys in [('residual', {'correction_gain'}),
                              ('rls', {'forgetting_factor', 'initial_covariance'})]:
            value = config.get('adaptation', {}).get(section, {})
            if not isinstance(value, dict) or set(value) - keys:
                raise ValueError(f'{self._config_path}:adaptation.{section}: unsupported keys')
        validate_parameters(config.get('prediction'),
                            {'dt': 'DOUBLE', 'horizon': 'INTEGER', 'type': 'STRING'},
                            required={'dt', 'horizon'}, path=f'{self._config_path}:prediction')
        def finite(value):
            if isinstance(value, dict):
                return all(finite(v) for v in value.values())
            if isinstance(value, list):
                return all(finite(v) for v in value)
            return not isinstance(value, (int, float)) or math.isfinite(value)
        if not finite(config):
            raise ValueError(f'{self._config_path}: prediction parameters must be finite')
        return config

    def get(
        self,
        section: str,
    ) -> dict:
        """Return a configuration section."""
        if section not in self._config:
            raise KeyError(
                f'missing configuration section: '
                f'{section}'
            )

        value = self._config[section]

        if not isinstance(value, dict):
            raise ValueError(
                f'configuration section '
                f'{section} must be a mapping.'
            )

        return value

    @property
    def data(
        self,
    ) -> dict:
        """Return complete configuration data."""
        return self._config.copy()
