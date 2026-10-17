"""Construct the actuator's existing physical guard and startup reference."""

from dataclasses import dataclass

import numpy as np

from robot_simulation.actuation.position_guard import PositionGuard
from robot_simulation.actuation.position_reference import PositionReference


@dataclass
class ActuationComponents:
    """Stateful components and limits shared by preflight and runtime assembly."""

    names: list[str]
    reference: PositionReference
    guard: PositionGuard
    lower: np.ndarray
    upper: np.ndarray


def create_actuation_components(config, robot, physical, holding, base_lower, base_upper, *, enabled):
    """Apply the existing adapter startup rules without altering servo behavior."""
    allowed = {'command_topic', 'rate_hz', 'command_max_age', 'feedback_max_age', 'wall_timeout',
               'future_tolerance', 'max_position_lead', 'max_integration_dt', 'position_recovery',
               'publish_diagnostics', 'diagnostic_rate_hz', 'diagnostic_topic'}
    unknown = set(config) - allowed
    if unknown:
        raise ValueError(f'Unknown actuation settings: {sorted(unknown)}')
    names = [robot['joint_name_template'].format(key=key) for key in physical]
    if not names or len(set(names)) != len(names):
        raise ValueError('Actuation joint names must be unique')
    reference = PositionReference(
        [j['lower'] for j in physical.values()], [j['upper'] for j in physical.values()],
        [j['velocity'] for j in physical.values()], config['max_position_lead'], config['max_integration_dt'])
    guard = PositionGuard(reference.lower, reference.upper, reference.velocity_limit, **config['position_recovery'])
    if guard.inset > reference.max_lead:
        raise ValueError('Recovery inset must not exceed max_position_lead')
    if holding is not None:
        if enabled and not holding['enabled']:
            raise ValueError('Position actuation requires the configured holding PIDs')
        if set(holding['positions']) != set(physical):
            raise ValueError('Startup reference must cover the authoritative joints')
        initial = np.array([holding['positions'][key] for key in physical], dtype=float)
        if np.any(initial < reference.lower) or np.any(initial > reference.upper):
            raise ValueError('Startup reference violates physical joint limits')
        reference.initialize(initial)
    if len(base_lower) != 2 or len(base_upper) != 2:
        raise ValueError('Base limits must be explicit [v, omega] arrays')
    lower = np.r_[base_lower, -reference.velocity_limit]
    upper = np.r_[base_upper, reference.velocity_limit]
    if (not np.all(np.isfinite(np.r_[lower, upper])) or np.any(lower >= upper) or
            np.any(lower > 0) or np.any(upper < 0)):
        raise ValueError('Actuation limits must be finite and contain a safe zero')
    for name in ['rate_hz', 'command_max_age', 'feedback_max_age', 'wall_timeout', 'future_tolerance']:
        if not np.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f'Invalid actuation {name}')
    if enabled and not robot['differential_drive']['enabled']:
        raise ValueError('Actuation requires the configured base drive')
    if config.get('publish_diagnostics', False):
        rate = float(config['diagnostic_rate_hz'])
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError('Diagnostic rate must be finite and positive')
    return ActuationComponents(names, reference, guard, lower, upper)
