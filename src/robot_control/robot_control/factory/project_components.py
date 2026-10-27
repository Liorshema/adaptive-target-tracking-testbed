"""
Register project components and adapt robot-description geometry.

Physical robot parameters are read from the existing robot description.
No ROS, Gazebo, or actuator dependencies are introduced.
"""

from collections.abc import Mapping
from typing import Any

from robot_control.adapters.planar_kinematics_adapter import PlanarKinematicsAdapter
from robot_control.controllers.jacobian_controller import JacobianController
from robot_control.controllers.weighted_least_squares import WeightedLeastSquaresController
from robot_control.factory.component_registry import ComponentRegistry
from robot_control.factory.control_factory import ControlFactory
from robot_control.tracking.radial_direction_policy import RadialDirectionPolicy
from robot_control.tracking.velocity_task import CartesianVelocityTask
from robot_control.tracking.velocity_task import DistancePointingTask
from robot_models.base.differential_drive.kinematics import DifferentialDriveKinematics
from robot_models.manipulator.serial_arm.model import SerialArmModel
from robot_models.reference.geometry.sphere_reference import SphereReference


def create_project_registries(configuration_path=None) -> dict[str, ComponentRegistry]:
    """Register implementations currently available in the project."""
    registries = {name: ComponentRegistry() for name in ControlFactory.REGISTRY_NAMES}

    registries['base'].register(
        'differential_drive',
        DifferentialDriveKinematics,
    )
    registries['manipulator'].register(
        'serial_arm',
        SerialArmModel,
    )
    registries['adapter'].register(
        'planar',
        PlanarKinematicsAdapter,
    )

    registries['task'] = ComponentRegistry()
    _register_algorithms(registries, configuration_path)
    return registries


def robot_configuration_from_description(
    description: Mapping[str, Any],
) -> dict[str, Any]:
    """Project authoritative limits into the existing control configuration contract."""
    from robot_description.adapters.configuration import robot_component_configuration

    return robot_component_configuration(description)


def _register_algorithms(registries, configuration_path):
    import numpy as np

    from robot_configuration.components import schema_for
    from robot_configuration.yaml_loader import load_yaml
    from robot_control.configuration import control_configuration_path

    data = load_yaml(configuration_path or control_configuration_path())

    def dls_builder(gains, damping, task):
        gains = np.asarray(gains)
        return JacobianController(
            task=task, gains=np.diag(gains) if gains.ndim == 1 else gains, damping=damping
        )

    for name, builder, dependencies in [
        ('jacobian', dls_builder, ('task',)),
        ('weighted_least_squares', WeightedLeastSquaresController, ('task', 'safety')),
    ]:
        defaults = data['controllers'][name]['parameters']
        constraints = {
            key: (
                {
                    'type': 'array',
                    'minItems': 1,
                    'items': {'type': 'number', 'exclusiveMinimum': 0.0},
                }
                if key in {'gains', 'task_weights', 'task_scales', 'motion_weights'}
                else {'type': 'number', 'minimum': 0.0}
            )
            for key in defaults
        }
        constraints['damping'] = {'type': 'number', 'exclusiveMinimum': 0.0}
        if name == 'jacobian':
            constraints['gains'] = {
                'type': 'array',
                'items': {
                    'oneOf': [{'type': 'number'}, {'type': 'array', 'items': {'type': 'number'}}]
                },
            }
        units = {key: '1' for key in defaults}
        units['gains'] = 's^-1'
        if 'task_scales' in units:
            units['task_scales'] = 'task units by row'
            units['posture_gain'] = 's^-1'
        if name == 'jacobian':
            units['damping'] = 'task-dependent regularization'
        schema = schema_for(defaults, units=units, constraints=constraints)
        if name == 'weighted_least_squares':
            schema['properties']['preferred_joint_positions'] = {
                'type': 'array',
                'items': {'type': 'number'},
                'unit': 'rad',
            }
        registries['controller'].register(
            name,
            builder,
            defaults=defaults,
            schema=schema,
            dependencies=dependencies,
            description=name.replace('_', ' '),
            capabilities={
                'deployable': True,
                'command_layout': 'body_base_joint_velocity',
                'command_dimension': 'base_velocity_dimension + joint_count',
                'safety': 'independent final projection; WLS shares feasible bounds',
            },
        )
    tracking = data['tracking']['parameters']
    radius = tracking['radius']
    policy = {**data['references']['radial']['parameters'], 'radius': radius}
    registries['policy'].register(
        'radial',
        RadialDirectionPolicy,
        defaults=policy,
        schema=schema_for(
            policy,
            units={'radius': 'm', 'epsilon': 'm'},
            constraints={k: {'type': 'number', 'exclusiveMinimum': 0.0} for k in policy},
        ),
        description='Radial spherical reference parameters',
        capabilities={
            'deployable': True,
            'spatial_dimension': 3,
            'input': 'current_position,target_position',
            'output': 'radius,azimuth,elevation',
        },
    )
    registries['geometry'].register(
        'sphere',
        SphereReference,
        defaults={},
        schema=schema_for({}),
        description='Spherical reference geometry',
        capabilities={
            'deployable': True,
            'spatial_dimension': 3,
            'parameters': 'radius,azimuth,elevation',
        },
    )
    distance = data['tasks']['distance_pointing']['parameters']
    schema = schema_for(
        distance,
        units={'minimum_distance': 'm', 'antipodal_tolerance': 'rad'},
        constraints={k: {'type': 'number', 'exclusiveMinimum': 0.0} for k in distance},
    )
    for key in ['radius', 'forward_axis_local', 'tangent_axis_local']:
        schema['properties'][key] = (
            {'type': 'number', 'exclusiveMinimum': 0.0}
            if key == 'radius'
            else {'type': 'array', 'minItems': 3, 'maxItems': 3, 'items': {'type': 'number'}}
        )
    registries['task'].register(
        'distance_pointing',
        DistancePointingTask,
        defaults=distance,
        schema=schema,
        description='Distance and free-roll pointing',
        capabilities={
            'deployable': True,
            'target_layout': 'position_velocity',
            'spatial_dimension': 3,
            'task_dimension': 3,
            'geometry_source': 'shared tracking configuration',
        },
    )
    registries['task'].register(
        'cartesian',
        CartesianVelocityTask,
        defaults={},
        schema={
            'type': 'object',
            'additionalProperties': False,
            'properties': {'selection_matrix': {'type': 'array'}},
        },
        description='Selected Cartesian twist task',
        capabilities={
            'deployable': True,
            'spatial_dimension': 3,
            'task_dimension': 'selection_matrix row count',
        },
    )
