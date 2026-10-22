"""Supported Gazebo physics engines; paths resolve from the active installation."""
import importlib.util
from pathlib import Path

from robot_simulation.factory.physics_configuration import validate_physics_engine


def physics_engine_library(engine):
    """Find a physics plugin without importing a shared library as Python code."""
    validate_physics_engine(engine)
    if engine == 'dartsim':
        return 'gz-physics-dartsim-plugin'
    spec = importlib.util.find_spec('robot_simulation.gazebo._dartsim_recovery')
    if spec is None or not spec.origin or not Path(spec.origin).is_file():
        raise ValueError('Recovery physics engine is not built; rebuild robot_simulation '
                         'and source install/setup.bash')
    return str(Path(spec.origin).resolve())
