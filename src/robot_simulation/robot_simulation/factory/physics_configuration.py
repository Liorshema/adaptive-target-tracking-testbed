"""Pure configuration contract for the deployed physics backend selector."""

PHYSICS_ENGINES = frozenset({'dartsim', 'dartsim_recovery'})


def validate_physics_engine(engine):
    """Reject ignored selectors before any simulation processes start."""
    if not isinstance(engine, str) or engine not in PHYSICS_ENGINES:
        raise ValueError(f'world.physics.engine must be one of {sorted(PHYSICS_ENGINES)}')
