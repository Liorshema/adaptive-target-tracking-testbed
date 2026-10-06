from pathlib import Path

import yaml


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

        with self._config_path.open(
            'r',
            encoding='utf-8',
        ) as file:
            config = yaml.safe_load(file)

        if config is None:
            raise ValueError(
                'configuration file is empty.'
            )

        if not isinstance(config, dict):
            raise ValueError(
                'configuration root must be a mapping.'
            )

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
