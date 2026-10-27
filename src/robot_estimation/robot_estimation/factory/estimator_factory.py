"""Pure construction of the currently supported robot and target estimators."""

import numpy as np

from robot_estimation.filters.kalman_filter import KalmanFilter
from robot_estimation.robot.joint_bounds import JointBoundsConditioner
from robot_estimation.robot.models.measurement_model import ImuYawRateMeasurementModel
from robot_estimation.robot.models.measurement_model import JointEncoderMeasurementModel
from robot_estimation.robot.models.measurement_model import WheelOdometryMeasurementModel
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.planar_pose import pose_acceptance_options
from robot_estimation.robot.planar_pose import PoseAcceptance
from robot_estimation.robot.planar_pose import validate_localization_parameters
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator
from robot_estimation.state import StateEstimate
from robot_estimation.target.models.measurement_model import PositionMeasurementModel
from robot_estimation.target.models.process_model import ConstantVelocityProcessModel
from robot_estimation.target.target_state_estimator import TargetStateEstimator
from robot_estimation.timestamp_buffer import TimestampBuffer


def validate_runtime_parameters(parameters):
    """Check the existing node's publication and causal-reordering budgets."""
    validate_localization_parameters(parameters)
    rate = parameters.get('full_state_publish_rate', 0.0)
    period = parameters.get('diagnostic_log_period', 5.0)
    if not np.isfinite(rate) or rate < 0:
        raise ValueError('full_state_publish_rate must be finite and nonnegative')
    if not np.isfinite(period) or period <= 0:
        raise ValueError('diagnostic_log_period must be finite and positive')
    buffer = TimestampBuffer(
        parameters['reorder_holdback'],
        parameters['reorder_max_age'],
        parameters['future_tolerance'],
        parameters['reorder_capacity'],
    )
    for name in [
        'reorder_rate_hz',
        'diagnostic_rate_hz',
        'max_joint_state_age',
        'max_wheel_odometry_age',
    ]:
        if not np.isfinite(parameters[name]) or parameters[name] <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if buffer.holdback + 1 / parameters['reorder_rate_hz'] >= min(
        parameters['max_joint_state_age'], parameters['max_wheel_odometry_age']
    ):
        raise ValueError('Reorder latency must fit within component freshness budgets')


def _build_robot_estimator(parameters, physical=None):
    """Construct the configured robot EKF."""
    joint_count = int(parameters['joint_count'])
    process_model = RobotProcessModel(
        joint_count=joint_count,
        linear_acceleration_noise_std=float(parameters['linear_acceleration_noise_std']),
        angular_acceleration_noise_std=float(parameters['angular_acceleration_noise_std']),
        joint_acceleration_noise_std=float(parameters['joint_acceleration_noise_std']),
        planar_slip_noise_std=parameters.get('planar_slip_noise_std', 0.0),
        yaw_slip_noise_std=parameters.get('yaw_slip_noise_std', 0.0),
    )
    state_dimension = process_model.state_dimension
    initial_mean = np.asarray(parameters['robot_initial_state'], dtype=float)
    covariance_diagonal = np.asarray(parameters['robot_initial_covariance_diagonal'], dtype=float)
    if initial_mean.shape != (state_dimension,):
        raise ValueError(
            f'robot_initial_state has invalid dimension. '
            f'Expected {state_dimension}, got {initial_mean.shape}.'
        )
    if covariance_diagonal.shape != (state_dimension,):
        raise ValueError(
            f'robot_initial_covariance_diagonal has invalid dimension. '
            f'Expected {state_dimension}, got {covariance_diagonal.shape}.'
        )
    if not np.all(np.isfinite(initial_mean)) or not np.all(np.isfinite(covariance_diagonal)):
        raise ValueError('Robot initial distribution must be finite')
    if np.any(covariance_diagonal < 0.0):
        raise ValueError('robot_initial_covariance_diagonal must contain non-negative values.')
    initial_state = StateEstimate(
        mean=initial_mean, covariance=np.diag(covariance_diagonal), timestamp=None
    )
    wheel_measurement_model = WheelOdometryMeasurementModel(joint_count=joint_count)
    imu_measurement_model = ImuYawRateMeasurementModel(joint_count=joint_count)
    joint_measurement_model = JointEncoderMeasurementModel(joint_count=joint_count)
    constraint = None
    if parameters['joint_bounds_enabled']:
        if len(physical) != joint_count:
            raise ValueError('Description joint order/count must match estimator layout')
        constraint = JointBoundsConditioner(
            np.arange(5, 5 + joint_count),
            [joint['lower'] for joint in physical.values()],
            [joint['upper'] for joint in physical.values()],
            parameters['joint_bounds_numerical_tolerance'],
            parameters['max_joint_state_age'],
        )
    options = (
        pose_acceptance_options(parameters)
        if parameters.get('localization_enabled', False)
        else None
    )
    estimator = RobotStateEstimator(
        initial_state=initial_state,
        process_model=process_model,
        wheel_measurement_model=wheel_measurement_model,
        imu_measurement_model=imu_measurement_model,
        joint_measurement_model=joint_measurement_model,
        state_constraint=constraint,
        pose_acceptance=PoseAcceptance(**options) if options else None,
    )
    return estimator, constraint


def _build_target_estimator(parameters):
    """Construct the configured target Kalman filter."""
    spatial_dimension = int(parameters['target_spatial_dimension'])
    process_model = ConstantVelocityProcessModel(
        spatial_dimension=spatial_dimension,
        acceleration_noise_std=float(parameters['target_acceleration_noise_std']),
    )
    measurement_model = PositionMeasurementModel(spatial_dimension=spatial_dimension)
    initial_mean = np.asarray(parameters['target_initial_state'], dtype=float)
    covariance_diagonal = np.asarray(parameters['target_initial_covariance_diagonal'], dtype=float)
    expected_state_dimension = process_model.state_dimension
    if initial_mean.shape != (expected_state_dimension,):
        raise ValueError(
            f'target_initial_state has invalid dimension. '
            f'Expected {expected_state_dimension}, got {initial_mean.shape}.'
        )
    if covariance_diagonal.shape != (expected_state_dimension,):
        raise ValueError(
            f'target_initial_covariance_diagonal has invalid dimension. '
            f'Expected {expected_state_dimension}, got {covariance_diagonal.shape}.'
        )
    if not np.all(np.isfinite(initial_mean)) or not np.all(np.isfinite(covariance_diagonal)):
        raise ValueError('Target initial distribution must be finite')
    if np.any(covariance_diagonal < 0.0):
        raise ValueError('target_initial_covariance_diagonal must contain non-negative values.')
    initial_state = StateEstimate(
        mean=initial_mean, covariance=np.diag(covariance_diagonal), timestamp=None
    )
    filter_ = KalmanFilter(
        initial_state=initial_state,
        state_transition=process_model.state_transition_matrix,
        process_covariance=process_model.process_covariance,
        measurement_matrix=measurement_model.measurement_matrix(),
    )
    return TargetStateEstimator(filter_=filter_)


def create_estimation_registries(configuration_path=None):
    """Register supported estimator contracts using only owner YAML defaults."""
    from robot_configuration.components import ComponentRegistry
    from robot_configuration.components import schema_for
    from robot_configuration.yaml_loader import load_yaml
    from robot_estimation.configuration import estimation_configuration_path
    from robot_estimation.interfaces.estimator_interface import EstimatorInterface
    from robot_estimation.interfaces.robot_estimator import RobotEstimatorProtocol

    data = load_yaml(configuration_path or estimation_configuration_path())
    registries = {'robot_estimator': ComponentRegistry(), 'target_estimator': ComponentRegistry()}
    robot = data['robot_estimators']['robot_ekf']
    robot_defaults = {
        **robot['parameters'],
        **robot.get('optional_parameters', {}),
        'max_joint_state_age': data['runtime']['robot']['max_joint_state_age'],
    }
    target_defaults = data['target_estimators']['kalman']['parameters']

    def robot_builder(physical=None, **parameters):
        if parameters['joint_bounds_enabled'] and not isinstance(physical, dict):
            raise ValueError(
                'robot_ekf: authoritative physical description required for joint bounds'
            )
        result = _build_robot_estimator(parameters, physical)
        if not isinstance(result[0], RobotEstimatorProtocol):
            raise TypeError('Robot estimator does not implement the sensor adapter contract')
        return result

    def target_builder(**parameters):
        dimension = parameters['target_spatial_dimension']
        variance = np.asarray(parameters['target_position_variance'])
        if variance.shape != (dimension,) or np.any(variance < 0):
            raise ValueError(
                'target_position_variance dimension must match target spatial dimension'
            )
        result = _build_target_estimator(parameters)
        if not isinstance(result, EstimatorInterface):
            raise TypeError('Target estimator must implement EstimatorInterface')
        return result

    for family, name, defaults, builder, units, layout, adapter in [
        (
            'robot_estimator',
            'robot_ekf',
            robot_defaults,
            robot_builder,
            {
                'linear_acceleration_noise_std': 'm/s^(3/2)',
                'angular_acceleration_noise_std': 'rad/s^(3/2)',
                'joint_acceleration_noise_std': 'rad/s^(3/2)',
                'planar_slip_noise_std': 'm/s^(1/2)',
                'yaw_slip_noise_std': 'rad/s^(1/2)',
                'joint_bounds_numerical_tolerance': 'rad',
            },
            'planar_mobile_manipulator',
            'wheel_imu_joint_pose',
        ),
        (
            'target_estimator',
            'kalman',
            target_defaults,
            target_builder,
            {'target_acceleration_noise_std': 'm/s^2', 'target_position_variance': 'm^2'},
            'position_velocity',
            'position',
        ),
    ]:
        from robot_estimation.parameter_schema import OPTIONAL_PARAMETERS
        from robot_estimation.parameter_schema import PARAMETERS

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
            else:
                constraints[key] = {
                    'type': 'integer' if kind == 'INTEGER' else 'number',
                    'minimum': 0.0,
                }
        for key in ['joint_count', 'target_spatial_dimension']:
            if key in defaults:
                constraints[key]['minimum'] = 1
        schema = schema_for(defaults, units=units, constraints=constraints)
        if family == 'robot_estimator':
            schema['properties']['localization_nis_limit'] = {
                'type': 'number',
                'exclusiveMinimum': 0.0,
                'unit': '1',
            }
        prefix = family.split('_')[0]
        registries[family].register(
            name,
            builder,
            description=(
                'Planar robot EKF'
                if prefix == 'robot'
                else 'Constant-velocity target Kalman filter'
            ),
            defaults=defaults,
            schema=schema,
            dependencies=('physical',) if prefix == 'robot' else (),
            capabilities={
                'state_layout': layout,
                'deployable': True,
                'state_dimension': '5+2*n' if prefix == 'robot' else '2*d',
                'measurements': adapter,
                'runtime_validation': 'conditional',
                'noise_discretization': (
                    'continuous white acceleration density'
                    if prefix == 'robot'
                    else 'discrete acceleration variance'
                ),
            },
            requirements={
                prefix + '_state_layout': layout,
                prefix + '_measurement_adapter': adapter,
            },
        )
    return registries


def _component_parameters(parameters, registry, name):
    """Extract selected numerical leaves from the merged ROS compatibility map."""
    from robot_estimation.parameter_schema import OPTIONAL_PARAMETERS
    from robot_estimation.parameter_schema import PARAMETERS

    info = registry.describe(name)
    keys = set(info['schema']['properties']) if info['schema'] else set(parameters)
    unexpected = set(parameters) - set(PARAMETERS) - set(OPTIONAL_PARAMETERS) - keys
    if unexpected:
        raise ValueError(f'{name}: unknown parameters {sorted(unexpected)}')
    return {key: value for key, value in parameters.items() if key in keys}


def create_robot_estimator(
    parameters, physical=None, estimator_type='robot_ekf', *, registry=None, context=None
):
    """Adapt merged ROS parameters to the selected registered robot estimator."""
    registry = registry or create_estimation_registries()['robot_estimator']
    values = _component_parameters(parameters, registry, estimator_type)
    dependencies = (
        {'physical': physical}
        if 'physical' in registry.describe(estimator_type)['dependencies']
        else {}
    )
    result = registry.create(estimator_type, values, context=context, **dependencies)
    from robot_estimation.interfaces.robot_estimator import RobotEstimatorProtocol

    if (
        not isinstance(result, tuple)
        or len(result) != 2
        or not isinstance(result[0], RobotEstimatorProtocol)
    ):
        raise TypeError(
            'Registered robot estimator must return (RobotEstimatorProtocol, constraint)'
        )
    return result


def create_target_estimator(parameters, estimator_type='kalman', *, registry=None, context=None):
    """Adapt merged ROS parameters without restricting context-free library dimensions."""
    registry = registry or create_estimation_registries()['target_estimator']
    values = _component_parameters(parameters, registry, estimator_type)
    result = registry.create(estimator_type, values, context=context)
    from robot_estimation.interfaces.estimator_interface import EstimatorInterface

    if not isinstance(result, EstimatorInterface):
        raise TypeError('Registered target estimator must implement EstimatorInterface')
    if context is not None and context.get('target_state_layout') == 'position_velocity':
        if result.state.dimension != 2 * context['spatial_dimension']:
            raise ValueError('Target estimator dimension does not match measurement adapter')
    return result
