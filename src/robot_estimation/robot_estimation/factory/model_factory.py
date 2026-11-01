"""Registered physical models and deployment contracts, separate from filter algorithms."""

import numpy as np

from robot_configuration.components import ComponentRegistry, schema_for
from robot_estimation.parameter_schema import OPTIONAL_PARAMETERS, PARAMETERS
from robot_estimation.robot.models.measurement_model import (
    ImuYawRateMeasurementModel,
    JointEncoderMeasurementModel,
    WheelOdometryMeasurementModel,
)
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.target.models.measurement_model import PositionMeasurementModel
from robot_estimation.target.models.process_model import ConstantVelocityProcessModel
from robot_estimation.target.target_state_estimator import TargetStateEstimator

UNITS = {
    'linear_acceleration_noise_std': 'm/s^(3/2)',
    'angular_acceleration_noise_std': 'rad/s^(3/2)',
    'joint_acceleration_noise_std': 'rad/s^(3/2)',
    'planar_slip_noise_std': 'm/s^(1/2)',
    'yaw_slip_noise_std': 'rad/s^(1/2)',
    'joint_bounds_numerical_tolerance': 'rad',
    'target_acceleration_noise_std': 'm/s^2',
    'target_position_variance': 'm^2',
    'wheel_linear_velocity_variance': '(m/s)^2',
    'wheel_angular_velocity_variance': '(rad/s)^2',
    'imu_yaw_rate_variance': '(rad/s)^2',
    'joint_position_variance': 'rad^2',
    'joint_velocity_variance': '(rad/s)^2',
}


def parameter_schema(defaults):
    """Use the existing ROS primitive contract; numerical defaults remain in owner YAML."""
    declared = {**PARAMETERS, **OPTIONAL_PARAMETERS}
    constraints = {}
    for key in defaults:
        kind = declared[key]
        if kind.endswith('_ARRAY'):
            item = {'type': 'number'}
            if 'covariance' in key or 'variance' in key:
                item['minimum'] = 0.0
            constraints[key] = {'type': 'array', 'items': item}
        elif kind == 'BOOL':
            constraints[key] = {'type': 'boolean'}
        elif kind == 'STRING':
            constraints[key] = {'type': 'string'}
        else:
            constraints[key] = {
                'type': 'integer' if kind == 'INTEGER' else 'number',
                'minimum': 0.0,
            }
    for key in ['joint_count', 'target_spatial_dimension']:
        if key in constraints:
            constraints[key]['minimum'] = 1
    return schema_for(defaults, units=UNITS, constraints=constraints)


def _robot_motion(**parameters):
    # Preserve the historical owner factory's coercion before squaring amplitudes.
    parameters['joint_count'] = int(parameters['joint_count'])
    for key in ('linear_acceleration_noise_std', 'angular_acceleration_noise_std',
                'joint_acceleration_noise_std'):
        parameters[key] = float(parameters[key])
    return RobotProcessModel(**parameters)


def _target_cv(target_spatial_dimension, target_acceleration_noise_std):
    return ConstantVelocityProcessModel(
        int(target_spatial_dimension), float(target_acceleration_noise_std))


def _robot_sensors(joint_count, **parameters):
    del parameters  # R is supplied by the existing sensor/measurement transport adapters.
    return {
        'wheel': WheelOdometryMeasurementModel(joint_count),
        'imu': ImuYawRateMeasurementModel(joint_count),
        'joints': JointEncoderMeasurementModel(joint_count),
    }


def _position(spatial_dimension, target_position_variance):
    variance = np.asarray(target_position_variance)
    if (
        variance.shape != (spatial_dimension,)
        or not np.isfinite(variance).all()
        or np.any(variance < 0)
    ):
        raise ValueError(
            'target_position_variance dimension must match position measurement model'
        )
    return PositionMeasurementModel(spatial_dimension)


def create_model_registries(data):
    """Construct factory-local existing registries; no YAML-controlled import paths."""
    registries = {
        key: ComponentRegistry()
        for key in ('estimation_process', 'estimation_measurement', 'estimation_adapter')
    }
    definitions = [
        (
            'estimation_process',
            'process_models',
            'robot_motion',
            _robot_motion,
            (),
            {
                'state_layout': 'planar_mobile_manipulator',
                'process_kind': 'nonlinear',
                'noise_discretization': 'continuous white acceleration density',
            },
        ),
        (
            'estimation_process',
            'process_models',
            'target_cv',
            _target_cv,
            (),
            {
                'state_layout': 'position_velocity',
                'process_kind': 'linear',
                'noise_discretization': 'discrete acceleration variance',
            },
        ),
        (
            'estimation_measurement',
            'measurement_models',
            'robot_sensors',
            _robot_sensors,
            ('joint_count',),
            {
                'state_layout': 'planar_mobile_manipulator',
                'measurements': 'wheel_imu_joint_pose',
                'sources': [
                    'wheel_odometry',
                    'imu_yaw_rate',
                    'joint_encoders',
                    'optional_planar_pose',
                ],
            },
        ),
        (
            'estimation_measurement',
            'measurement_models',
            'target_position',
            _position,
            ('spatial_dimension',),
            {
                'state_layout': 'position_velocity',
                'measurements': 'position',
                'sources': ['position'],
            },
        ),
        (
            'estimation_adapter',
            'deployment_adapters',
            'robot_state',
            RobotStateEstimator,
            (
                'initial_state',
                'process_model',
                'wheel_measurement_model',
                'imu_measurement_model',
                'joint_measurement_model',
                'state_constraint',
                'pose_acceptance',
                'filter_factory',
            ),
            {
                'role': 'robot',
                'state_layout': 'planar_mobile_manipulator',
                'process_kind': 'nonlinear',
                'measurements': 'wheel_imu_joint_pose',
                'output': 'RobotEstimatorProtocol',
            },
        ),
        (
            'estimation_adapter',
            'deployment_adapters',
            'target_state',
            TargetStateEstimator,
            ('filter_',),
            {
                'role': 'target',
                'state_layout': 'position_velocity',
                'process_kind': 'linear',
                'measurements': 'position',
                'output': 'EstimatorInterface',
            },
        ),
    ]
    for family, section, name, builder, dependencies, capabilities in definitions:
        group = data[section][name]
        # Adapter parameters configure constraints/acceptance in the role factory,
        # rather than becoming numerical-filter constructor arguments.
        defaults = (
            {}
            if family == 'estimation_adapter'
            else {**group['parameters'], **group.get('optional_parameters', {})}
        )
        registries[family].register(
            name,
            builder,
            defaults=defaults,
            schema=parameter_schema(defaults),
            dependencies=dependencies,
            capabilities={
                **capabilities,
                'deployable': False,
                'runtime_validation': 'requires validated algorithm and role binding',
            },
        )
    return registries


def resolve_binding(role, binding, parameters, models, algorithm):
    """Validate the whole algorithm/model/adapter combination before constructing it."""
    binding = dict(binding)
    for key, field in [
        ('process_model', 'process_model'),
        ('measurement_model', 'measurement_model'),
        ('adapter', 'deployment_adapter'),
    ]:
        binding[key] = parameters.get(role + '_' + field, binding[key])
    process = models['estimation_process'].describe(binding['process_model'])['capabilities']
    measurement = models['estimation_measurement'].describe(binding['measurement_model'])[
        'capabilities'
    ]
    adapter = models['estimation_adapter'].describe(binding['adapter'])['capabilities']
    if adapter['role'] != role:
        raise ValueError(f'{role}: incompatible deployment adapter {binding["adapter"]}')
    if (
        process['state_layout'] != adapter['state_layout']
        or measurement['state_layout'] != adapter['state_layout']
    ):
        raise ValueError(f'{role}: incompatible process/measurement state layout and ordering')
    if measurement['measurements'] != adapter['measurements']:
        raise ValueError(f'{role}: unsupported measurement sources')
    if (
        process['process_kind'] != adapter['process_kind']
        or algorithm['process_kind'] != adapter['process_kind']
    ):
        raise ValueError(f'{role}: incompatible algorithm/process model for deployment adapter')
    if role == 'robot' and not (
        algorithm['dynamic_measurements'] and algorithm['state_constraints']
    ):
        raise ValueError('robot: algorithm requires dynamic measurement and conditioning support')
    return binding


def model_parameters(registry, identifier, parameters):
    """Select one model's declared parameters from the historical merged transport map."""
    keys = registry.describe(identifier)['schema']['properties']
    return {key: value for key, value in parameters.items() if key in keys}


def validate_models(process, measurements, initial_mean):
    """Verify dimensions/Jacobians at the real deployment binding boundary."""
    dimension = process.state_dimension
    if initial_mean.shape != (dimension,):
        raise ValueError('Initial state dimension does not match process model')
    for model in measurements.values() if isinstance(measurements, dict) else [measurements]:
        if model.state_dimension != dimension:
            raise ValueError('Measurement/process state dimensions differ')
        for name in ('predict_measurement', 'jacobian'):
            if not callable(getattr(model, name, None)):
                raise ValueError(f'Measurement model requires {name}')
        h, H = model.predict_measurement(initial_mean), model.jacobian(initial_mean)
        if (
            np.shape(h) != (model.measurement_dimension,)
            or np.shape(H) != (model.measurement_dimension, dimension)
            or not np.isfinite(h).all()
            or not np.isfinite(H).all()
        ):
            raise ValueError('Invalid measurement mapping/Jacobian dimension or finite values')
    if not callable(getattr(process, 'jacobian', None)):
        raise ValueError('Process model requires a Jacobian')
    F = process.jacobian(initial_mean, 1.0)
    if np.shape(F) != (dimension, dimension) or not np.isfinite(F).all():
        raise ValueError('Invalid process Jacobian dimension or finite values')
