"""Verify installed interfaces, native plugin loading and real Catalog discovery.

This build/diagnostic check never launches an experiment or fabricates telemetry.
Run after sourcing ROS and the installed workspace.
"""

import hashlib
import json
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_share_directory
from rclpy.type_support import check_for_type_support
from robot_interfaces.msg import (
    ActuationTelemetry, ControlTelemetry, PredictedTrajectory, SafeVelocityCommand,
    StateEstimate,
)
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog
from robot_experiments.runner import _git
from robot_dashboard.app import create_app
from robot_simulation.gazebo.physics_engine import physics_engine_library
from robot_simulation.gazebo.robot_builder import actuator_library


def verify():
    """Fail on missing libraries/symbols, registration, interfaces or configuration."""
    import xml.etree.ElementTree as ET

    packages = {}
    source = Path(__file__).resolve().parents[2] / 'src'
    for manifest in sorted(source.glob('*/package.xml')):
        name = ET.parse(manifest).getroot().findtext('name')
        packages[name] = get_package_share_directory(name)
    if not packages:
        raise RuntimeError('No source package manifests found')
    for message in [ActuationTelemetry, ControlTelemetry, PredictedTrajectory,
                    SafeVelocityCommand, StateEstimate]:
        check_for_type_support(message)
    libraries = {}
    for name, library, registration in [
        ('actuator', actuator_library(), 'robot_simulation::ArmActuator'),
        ('physics', physics_engine_library('dartsim_recovery'),
         'gz::physics::robot_simulation_dartsim::Plugin'),
    ]:
        linked = subprocess.run(['ldd', library], check=True, text=True,
                                capture_output=True, timeout=30)
        if 'not found' in linked.stdout + linked.stderr:
            raise RuntimeError(f'{name}: unresolved dependency: {linked.stdout}')
        loaded = subprocess.run(['gz', 'plugin', '--info', '--plugin', library],
                                check=True, text=True, capture_output=True, timeout=30)
        if registration not in loaded.stdout:
            raise RuntimeError(f'{name}: registration missing: {loaded.stdout}')
        libraries[name] = {'path': library, 'registration': registration,
                           'sha256': hashlib.sha256(Path(library).read_bytes()).hexdigest(),
                           'ldd': linked.stdout, 'plugin_info': loaded.stdout}
    with TemporaryDirectory(prefix='testbed-install-check-') as temporary:
        root = Path(temporary)
        catalog = ExperimentCatalog(root / 'catalog.yaml')
        catalog.initialize()
        backend = ExperimentBackend(root, catalog=catalog)
        components = {role: [m['id'] for m in backend.list_component_options(role)]
                      for role in ['predictor', 'adaptation', 'fusion', 'controller',
                                   'robot_estimator', 'target_estimator', 'target_dynamics']}
        nominal = catalog.prepare('EXP-NOMINAL')
        dashboard_status = create_app(backend).server.test_client().get('/_dash-layout').status_code
        if dashboard_status != 200:
            raise RuntimeError(f'Dashboard layout failed: HTTP {dashboard_status}')
        return {'packages': packages, 'interfaces_checked': 5, 'libraries': libraries,
                'components': components, 'experiments': backend.list_experiments(),
                'dashboard_layout_status': dashboard_status,
                'nominal_variants': [v['name'] for v in nominal['variants']], 'git': _git()}


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2))
