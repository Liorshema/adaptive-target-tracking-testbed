"""Retain the control constructor API over the shared trusted registry."""

from collections.abc import Mapping

from robot_configuration.components import ComponentRegistry as RegisteredComponents


class ComponentRegistry(RegisteredComponents):
    """Separate runtime task/safety dependencies from validated model parameters."""

    def create(self, name, parameters=None, *, context=None, **dependencies):
        if parameters is not None and not isinstance(parameters, Mapping):
            raise TypeError('Component parameters must be a mapping')
        parameters = dict(parameters or {})
        for key in self._entry(name)['dependencies']:
            if key in parameters:
                if key in dependencies:
                    raise ValueError(f'{name}: shadowed dependency {key}')
                dependencies[key] = parameters.pop(key)
        return super().create(name, parameters, context=context, **dependencies)
