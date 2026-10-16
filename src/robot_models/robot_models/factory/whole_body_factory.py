"""Compose injected model components and an optional fixed tracking frame."""

from collections.abc import Callable, Mapping
from typing import Any

import numpy as np

from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.manipulator.fixed_mount.model import FixedMountModel
from robot_models.whole_body.composer import WholeBodyComposer


def build_configured_model(
    configuration: Mapping[str, Any],
    component_factory: Callable[[str, Mapping[str, Any]], Any],
) -> WholeBodyComposer:
    """
    Build a whole-body model through externally supplied component construction.

    The factory receives ``base`` and ``manipulator`` categories and their
    unmodified specification mappings. An optional ``tracking_frame`` transform
    is applied once to the returned manipulator's end frame. Its optional parent
    label is deployment metadata; these interfaces do not expose named links.
    No model registry, description file, or ROS runtime is required here.
    """
    if not isinstance(configuration, Mapping):
        raise TypeError('configuration must be a mapping')
    for category in ['base', 'manipulator']:
        if not isinstance(configuration.get(category), Mapping):
            raise ValueError(f'{category} configuration must be a mapping')
    base = component_factory('base', configuration['base'])
    if not isinstance(base, BaseModel):
        raise TypeError('base component must implement BaseModel')
    manipulator = component_factory('manipulator', configuration['manipulator'])
    if not isinstance(manipulator, ManipulatorModel):
        raise TypeError('manipulator component must implement ManipulatorModel')
    if 'tracking_frame' in configuration:
        mount = configuration['tracking_frame']
        if not isinstance(mount, Mapping):
            raise ValueError('tracking_frame configuration must be a mapping')
        transform = {}
        for field in ['translation', 'rotation_rpy']:
            value = np.asarray(mount.get(field), dtype=float)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f'tracking_frame {field} must be a finite 3-vector')
            transform[field] = value
        manipulator = FixedMountModel(parent=manipulator, **transform)
    return WholeBodyComposer(base=base, manipulator=manipulator)
