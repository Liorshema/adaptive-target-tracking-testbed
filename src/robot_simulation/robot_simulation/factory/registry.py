"""Simulation's configuration-mapping adapter over trusted component registrations."""

from collections.abc import Mapping

from robot_configuration.components import ComponentRegistry as RegisteredComponents


class ComponentRegistry(RegisteredComponents):
    """Preserve the public simulation factory API without another registry engine."""

    def create(self, config, **dependencies):
        if not isinstance(config, Mapping) or set(config) - {'type', 'parameters', 'enabled'}:
            raise ValueError('Component specification must contain type and parameters only')
        if 'enabled' in config and type(config['enabled']) is not bool:
            raise ValueError('Component enabled must be boolean')
        try:
            entry = self._entry(config['type'])
            if entry['schema'] is None and dependencies:
                parameters = config.get('parameters', {})
                if not isinstance(parameters, Mapping):
                    raise ValueError('Component parameters must be a mapping')
                if set(parameters) & set(dependencies):
                    raise ValueError('Component dependencies cannot shadow parameters')
                return super().create(config['type'], {**parameters, **dependencies})
            return super().create(config['type'], config.get('parameters', {}), **dependencies)
        except KeyError as error:
            raise ValueError(str(error)) from error

    @property
    def registered_types(self):
        """Retain insertion-ordered names for existing simulation callers."""
        return tuple(self._entries)
