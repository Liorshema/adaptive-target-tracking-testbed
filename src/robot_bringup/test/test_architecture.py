"""Small source/manifest checks for the current package boundaries."""

import ast
from importlib import import_module
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from setuptools import find_packages
from robot_experiments.configuration import resolve_system_resource

SRC = Path(__file__).resolve().parents[2]
ALLOWED = {
    'robot_configuration': set(), 'robot_description': set(), 'robot_interfaces': set(),
    'robot_models': set(), 'robot_estimation': {'robot_description', 'robot_interfaces', 'robot_configuration'},
    'robot_prediction': {'robot_interfaces', 'robot_configuration'},
    'robot_control': {'robot_description', 'robot_models', 'robot_interfaces', 'robot_configuration'},
    'robot_simulation': {'robot_description', 'robot_models', 'robot_interfaces', 'robot_configuration'},
    # Experiment materialization reuses the actual simulation sensor-owner validators.
    'robot_experiments': {'robot_bringup', 'robot_configuration', 'robot_description',
                          'robot_models', 'robot_interfaces', 'robot_simulation', 'robot_control',
                          'robot_estimation', 'robot_prediction'},
    'robot_bringup': {'robot_description', 'robot_models', 'robot_interfaces', 'robot_configuration',
                      'robot_simulation', 'robot_estimation', 'robot_prediction', 'robot_control'},
}
OBSOLETE = [
    'robot_control/robot_control/ros', 'robot_estimation/robot_estimation/estimation_node.py',
    'robot_prediction/robot_prediction/prediction_node.py',
    'robot_prediction/robot_prediction/system_factory.py', 'robot_prediction/robot_prediction/config_loader.py',
    'robot_simulation/robot_simulation/actuation/velocity_actuation.py',
    'robot_simulation/robot_simulation/factory/config_loader.py',
    'robot_bringup/robot_bringup/model_factory.py', 'robot_bringup/robot_bringup/tracking_feedback_tf.py',
    'robot_bringup/robot_bringup/tracking_visualization.py',
    'robot_models/robot_models/base/aerial', 'robot_models/robot_models/base/omnidirectional',
    'robot_simulation/robot_simulation/disturbances/gaussian_noise.py',
    'robot_simulation/robot_simulation/gazebo/ground_truth_adapter.py',
    'robot_simulation/robot_simulation/interfaces/integrator.py', 'robot_simulation/config/disturbances.yaml',
]
CORE = {
    'robot_configuration': None, 'robot_models': None,
    'robot_experiments': {'metrics'},
    'robot_bringup': {'visualization'},
    'robot_estimation': {'interfaces', 'filters', 'robot', 'target', 'factory'},
    'robot_prediction': {'baseline', 'evaluation', 'fusion', 'history', 'interfaces', 'online_adaptation', 'factory'},
    'robot_control': {'controllers', 'interfaces', 'pipeline', 'safety', 'tracking'},
    'robot_simulation': {'actuation', 'dynamics', 'disturbances', 'engine', 'integration', 'interfaces', 'measurements', 'state', 'factory'},
}
ROS = {'rclpy', 'tf2_ros', 'launch', 'launch_ros', 'geometry_msgs', 'nav_msgs', 'sensor_msgs',
       'std_msgs', 'robot_interfaces', 'ros_gz_interfaces', 'gz', 'gazebo'}


def imports(path):
    """Read direct import statements, including function-local imports."""
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module


def test_declared_and_imported_project_dependencies_are_acyclic_and_owned():
    """Runtime imports must be declared and follow domain ownership; test imports are excluded."""
    graph = {}
    for package, allowed in ALLOWED.items():
        root = ET.parse(SRC / package / 'package.xml').getroot()
        declared = {e.text for e in root if e.tag in {'depend', 'exec_depend', 'build_depend'}}
        graph[package] = declared & ALLOWED.keys()
        assert graph[package] <= allowed, (package, graph[package]-allowed)
        paths = list((SRC / package / package).rglob('*.py'))
        paths += list((SRC / package / 'launch').glob('*.py'))
        for path in paths:
            for module in imports(path):
                target = module.split('.')[0]
                if target in ALLOWED and target != package:
                    assert target in allowed and target in declared, (path, module)

    def visit(package, ancestors):
        assert package not in ancestors, (package, ancestors)
        for target in graph[package]:
            visit(target, ancestors | {package})
    for package in graph:
        visit(package, set())


def test_numerical_layers_do_not_import_ros_nodes_or_gazebo():
    """Numerical/domain code cannot acquire a hidden dependency on runtime adapters."""
    for package, folders in CORE.items():
        root = SRC / package / package
        for path in root.rglob('*.py'):
            relative = path.relative_to(root)
            if folders is not None and len(
                    relative.parts) > 1 and relative.parts[0] not in folders:
                continue
            for module in imports(path):
                assert module.split('.')[0] not in ROS, (path, module)
                assert '.nodes' not in module and '.gazebo' not in module, (path, module)


@pytest.mark.parametrize('path', OBSOLETE)
def test_obsolete_modules_and_placeholders_are_absent(path):
    assert not (SRC / path).exists(), path


def test_surviving_source_files_are_meaningful_and_packaged():
    """Initializers/resource markers are legitimate; empty non-initializer Python/YAML is not."""
    for package in ALLOWED:
        root = SRC / package
        packaged = set(find_packages(where=str(root), exclude=['test']))
        for path in (root / package).rglob('*.py'):
            if path.name != '__init__.py':
                assert path.read_text().strip(), path
            assert '.'.join(path.parent.relative_to(root).parts) in packaged, path
        for path in (root / 'config').rglob('*.yaml'):
            assert path.read_text().strip(), path


@pytest.mark.parametrize('module', [
    'robot_control.nodes.control_node', 'robot_estimation.nodes.estimation_node',
    'robot_prediction.nodes.prediction_node', 'robot_simulation.nodes.simulation_node',
    'robot_simulation.nodes.velocity_actuation', 'robot_bringup.nodes.tracking_feedback_tf',
    'robot_bringup.nodes.tracking_visualization',
])
def test_canonical_nodes_have_one_implementation(module):
    node = import_module(module)
    assert node.main.__globals__ is node.__dict__
    assert Path(node.__file__).parent.name == 'nodes'


def test_active_launch_profile_resource_references_resolve():
    """Every experiment-owned launch group and installed system preflights."""
    from robot_configuration.yaml_loader import load_yaml
    from robot_experiments.launching import launch_profile
    from robot_bringup.factory.system_configuration import load_system_configuration

    def share(name):
        return SRC / name
    definitions = load_yaml(
        share('robot_experiments') /
        'config/experiments.yaml')['direct_launch']
    assert definitions['cases']
    for name in definitions['cases']:
        plan = launch_profile(name, share)
        assert Path(plan['profile']['simulation_config']).is_file()
        assert plan['runtime']['enable_actuation'] is False
    selections = load_yaml(share('robot_experiments') /
                           'config/experiments.yaml')['compositions']['selections']
    for name in selections:
        reference = {'package': 'robot_experiments', 'path': 'config/experiments.yaml',
                     'group': ['compositions.common', 'compositions.selections.' + name]}
        source = resolve_system_resource(reference, Path.cwd(), share)
        assert load_system_configuration(source, share)['identities']


def test_semantic_layout_has_no_generic_common_or_replaced_namespaces():
    for package in ALLOWED:
        assert not list((SRC / package / package).rglob('common'))
    for relative in [
        'robot_models/robot_models/types.py',
        'robot_control/robot_control/baseline', 'robot_control/robot_control/optimization',
        'robot_prediction/robot_prediction/confidence', 'robot_prediction/robot_prediction/residuals',
        'robot_simulation/robot_simulation/geometry',
    ]:
        assert not (SRC / relative).exists(), relative


@pytest.mark.parametrize('module, symbol', [
    ('robot_models.spatial.transforms', 'make_transform'),
    ('robot_models.spatial.spatial_data', 'Pose'),
    ('robot_models.target.target_state', 'TargetState'),
    ('robot_models.whole_body.robot_state', 'RobotState'),
    ('robot_models.constraints.constraint_set', 'ConstraintSet'),
    ('robot_models.factory.whole_body_factory', 'build_configured_model'),
    ('robot_estimation.interfaces.estimator_interface', 'EstimatorInterface'),
    ('robot_estimation.interfaces.filter_interface', 'FilterInterface'),
    ('robot_estimation.interfaces.sensor_interface', 'SensorInterface'),
    ('robot_estimation.state', 'StateEstimate'),
    ('robot_estimation.measurement', 'Measurement'),
    ('robot_estimation.timestamp_buffer', 'TimestampBuffer'),
    ('robot_control.controllers.jacobian_controller', 'JacobianController'),
    ('robot_control.controllers.weighted_least_squares', 'WeightedLeastSquaresController'),
    ('robot_control.interfaces.velocity_task', 'VelocityTask'),
    ('robot_prediction.evaluation.performance_monitor', 'PerformanceMonitor'),
    ('robot_prediction.temporal_alignment', 'interpolate_predicted_state'),
    ('robot_simulation.gazebo.model_exporter', 'ModelExporter'),
    ('robot_simulation.adapters.target_state', 'state_to_vector'),
    ('robot_description.adapters.configuration', 'robot_component_configuration'),
])
def test_semantic_canonical_modules_resolve(module, symbol):
    implementation = import_module(module)
    assert getattr(implementation, symbol).__module__ == module
