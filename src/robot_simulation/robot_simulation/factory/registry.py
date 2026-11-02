"""Simulation's configuration-mapping adapter over trusted component registrations."""

from collections.abc import Mapping

from robot_configuration.components import ComponentRegistry as RegisteredComponents


class ComponentRegistry(RegisteredComponents):
    """Preserve the public simulation factory API without another registry engine."""

    def validate(self, name, parameters=None, context=None, *, use_defaults=True):
        """Delegate coupled/atomic parameter preparation to an optional owner hook."""
        entry = self._entry(name)
        prepare = getattr(entry['builder'], 'prepare_parameters', None)
        if prepare is not None:
            parameters = prepare(
                {} if parameters is None else parameters, entry['defaults'],
                use_defaults=use_defaults)
            use_defaults = False
        return super().validate(name, parameters, context, use_defaults=use_defaults)

    def apply_overrides(self, name, parameters, overrides):
        """Resolve existing owner values with their atomic/domain override semantics."""
        from robot_configuration.parameter_validation import override_parameters

        prepare = getattr(self._entry(name)['builder'], 'prepare_parameters', None)
        values = (prepare(overrides, parameters) if prepare is not None
                  else override_parameters(parameters, overrides))
        return self.validate(name, values, use_defaults=False)

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
