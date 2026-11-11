"""Bounded, model-only numerical inspection through the authoritative D3 and RK4."""

import math
import time

import numpy as np

from robot_simulation.integration.rk4 import RK4Integrator

from .equation_model import EquationDynamics
from .motion_composition import compile_definition, definition_data


def preview_motion(definition, *, duration, dt, context=None, control=None):
    """Return unscored numerical samples; all external inputs are fixed assumptions.

    No environment is attached. The finite duration/step/cost/output bounds protect
    the local interactive service; the wall deadline aborts, never changes dt.
    """
    for name, value, upper in [('duration', duration, 60.0), ('dt', dt, 1.0)]:
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError(f'Preview {name} must be finite, positive and <= {upper}')
    step_count = duration / dt
    if not math.isfinite(step_count) or step_count > 2000:
        raise ValueError('Preview exceeds 2000 integration steps; reduce duration or increase dt')
    steps = math.ceil(step_count)
    compiled = compile_definition(definition_data(definition))
    model = EquationDynamics(model=compiled['model'])
    if (
        steps * max(1, model.expression_cost) > 1000000
        or (steps + 1) * model.state_dimension > 150000
    ):
        raise ValueError('Preview exceeds expression/output resource budget')
    context = definition_data(context or {})
    declared = compiled['model'].get('context', {})
    if set(context) - set(declared):
        raise ValueError('Preview has undeclared context inputs')
    if set(declared) - set(context):
        raise ValueError(
            'Fixed preview context required: ' + ', '.join(sorted(set(declared) - set(context)))
        )
    if model.control_dimension and control is None:
        raise ValueError('Explicit fixed preview control vector required')
    control = definition_data([] if control is None else control)
    x = model.initial_state
    samples, times = [x.tolist()], [0.0]
    integrator = RK4Integrator()
    deadline = time.monotonic() + 5.0
    for index in range(steps):
        start = times[-1]
        if start == duration:
            break
        end = min((index + 1) * dt, duration)
        if time.monotonic() > deadline:
            raise ValueError(f'Preview wall-time budget exceeded at step {index+1}, t={start:g} s')
        try:
            x = integrator.step(model, x, np.asarray(control), start, end - start, context)
        except (ValueError, TypeError, FloatingPointError, OverflowError) as error:
            raise ValueError(
                f'Preview failed at step {index+1}, t={start:g} s: {error}'
            ) from error
        samples.append(x.tolist())
        times.append(end)
    offset, series = 0, []
    for state in compiled['model']['state']:
        size = math.prod(state['shape']) if state['shape'] else 1
        series.append(
            {
                'name': state['name'],
                'unit': state['unit'],
                'indices': list(range(offset, offset + size)),
            }
        )
        offset += size
    deployment = compiled['model'].get('deployment')
    position = None
    if deployment is not None:
        position = next(s for s in series if s['name'] == deployment['position'])
    positions, velocities = [], []
    if model.output_mapping is not None:
        for instant, sample in zip(times, samples):
            if time.monotonic() > deadline:
                raise ValueError(f'Preview output wall-time budget exceeded at t={instant:g} s')
            p, v = model.physical_output(sample, control, instant, context)
            positions.append(p.tolist())
            velocities.append(v.tolist())
    return {
        'positions': positions, 'velocities': velocities,
        'output_mapping': model.output_mapping,
        'times': times,
        'states': samples,
        'state_series': series,
        'position': position,
        'fixed_context': context,
        'fixed_control': control,
        'environment_included': False,
        'capabilities': model.capabilities,
        'symbol_map': compiled['symbol_map'],
    }
