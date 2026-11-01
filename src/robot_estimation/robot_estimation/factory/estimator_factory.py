"""Pure construction of the currently supported robot and target estimators."""

from copy import deepcopy

import numpy as np

from robot_estimation.robot.joint_bounds import JointBoundsConditioner
from robot_estimation.robot.planar_pose import pose_acceptance_options
from robot_estimation.robot.planar_pose import PoseAcceptance
from robot_estimation.robot.planar_pose import validate_localization_parameters
from robot_estimation.state import StateEstimate
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


def _build_robot_estimator(
    parameters, physical, *, process_model, measurement_models, filter_factory, adapter_factory
):
    """Construct the configured robot EKF."""
    joint_count = int(parameters['joint_count'])
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
    from robot_estimation.factory.model_factory import validate_models

    validate_models(process_model, measurement_models, initial_mean)
    wheel_measurement_model = measurement_models['wheel']
    imu_measurement_model = measurement_models['imu']
    joint_measurement_model = measurement_models['joints']
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
    estimator = adapter_factory(
        initial_state=initial_state,
        process_model=process_model,
        wheel_measurement_model=wheel_measurement_model,
        imu_measurement_model=imu_measurement_model,
        joint_measurement_model=joint_measurement_model,
        state_constraint=constraint,
        pose_acceptance=PoseAcceptance(**options) if options else None,
        filter_factory=filter_factory,
    )
    return estimator, constraint


def _build_target_estimator(
    parameters, *, process_model, measurement_model, filter_factory, adapter_factory
):
    """Construct the configured target Kalman filter."""
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
    from robot_estimation.factory.model_factory import validate_models

    validate_models(process_model, measurement_model, initial_mean)
    filter_ = filter_factory(
        initial_state=initial_state,
        state_transition=process_model.state_transition_matrix,
        process_covariance=process_model.process_covariance,
        measurement_matrix=measurement_model.measurement_matrix(),
    )
    return adapter_factory(filter_=filter_)


def create_estimation_registries(configuration_path=None, *, models=None, algorithms=None):
    """Compose registered algorithms/models into validated existing role contracts."""
    from robot_configuration.components import ComponentRegistry
    from robot_configuration.yaml_loader import load_yaml
    from robot_estimation.configuration import (
        binding_parameters,
        canonical_estimation,
        estimation_configuration_path,
        LEGACY_ESTIMATORS,
        role_binding,
    )
    from robot_estimation.factory.algorithm_factory import create_algorithm_registry
    from robot_estimation.factory.model_factory import create_model_registries, parameter_schema

    data = canonical_estimation(load_yaml(configuration_path or estimation_configuration_path()))
    algorithms = algorithms if algorithms is not None else create_algorithm_registry(data)
    models = models if models is not None else create_model_registries(data)
    registries = {
        'robot_estimator': ComponentRegistry(),
        'target_estimator': ComponentRegistry(),
        'estimator': algorithms,
        **models,
    }
    for role in ('robot', 'target'):
        binding_name, binding = role_binding(data, role)
        defaults = binding_parameters(data, binding_name, include_optional=True)
        if role == 'robot':
            defaults['max_joint_state_age'] = data['runtime']['robot']['max_joint_state_age']
        schema = parameter_schema(defaults)
        if role == 'robot':
            schema['properties']['localization_nis_limit'] = {
                'type': 'number',
                'exclusiveMinimum': 0.0,
                'unit': '1',
            }
        for suffix in ('process_model', 'measurement_model', 'deployment_adapter'):
            schema['properties'][role + '_' + suffix] = {'type': 'string'}
        adapter = models['estimation_adapter'].describe(binding['adapter'])['capabilities']
        process = models['estimation_process'].describe(binding['process_model'])['capabilities']
        for identifier in algorithms.names():
            capability = algorithms.describe(identifier)['capabilities']
            if capability['process_kind'] != adapter['process_kind']:
                continue  # numerical availability does not imply a compatible ROS adapter
            builder = _role_builder(role, binding, identifier, algorithms, models)
            role_defaults = deepcopy(defaults)
            role_defaults['algorithm_parameters'] = algorithms.describe(identifier)['defaults']
            role_schema = deepcopy(schema)
            role_schema['properties']['algorithm_parameters'] = algorithms.describe(identifier)[
                'schema']
            role_schema['properties'][role + '_algorithm_parameters'] = {'type': 'string'}
            names = [identifier] + [
                name
                for name, item in LEGACY_ESTIMATORS.items()
                if item == {'algorithm': identifier, 'binding': binding_name}
            ]
            for name in names:
                registries[role + '_estimator'].register(
                    name,
                    builder,
                    description=f'{identifier} with {binding_name} deployment binding',
                    defaults=role_defaults,
                    schema=role_schema,
                    dependencies=('physical',) if role == 'robot' else (),
                    capabilities={
                        'algorithm': identifier,
                        'alias_of': identifier if name != identifier else None,
                        'binding': binding_name,
                        'process_model': binding['process_model'],
                        'measurement_model': binding['measurement_model'],
                        'deployment_adapter': binding['adapter'],
                        'state_layout': adapter['state_layout'],
                        'deployable': True,
                        'intrinsic_parameter_transport': 'frozen per-role settings',
                        'state_dimension': '5+2*n' if role == 'robot' else '2*d',
                        'measurements': adapter['measurements'],
                        'runtime_validation': 'conditional',
                        'noise_discretization': process['noise_discretization'],
                    },
                    requirements={
                        role + '_state_layout': adapter['state_layout'],
                        role + '_measurement_adapter': adapter['measurements'],
                    },
                )
    return registries


def _role_builder(role, binding, algorithm, algorithms, models):
    """Bind dependencies explicitly; no algorithm switch is hidden in orchestration."""
    from robot_estimation.factory.model_factory import model_parameters, resolve_binding

    def build(physical=None, **parameters):
        from robot_estimation.factory.algorithm_parameters import decode_algorithm_parameters

        key = role + '_algorithm_parameters'
        intrinsic = (
            decode_algorithm_parameters(parameters[key], algorithm, algorithms)
            if key in parameters else parameters['algorithm_parameters']
        )
        selected = resolve_binding(
            role, binding, parameters, models, algorithms.describe(algorithm)['capabilities']
        )
        process_registry = models['estimation_process']
        process = process_registry.create(
            selected['process_model'],
            model_parameters(process_registry, selected['process_model'], parameters),
        )
        measurement_registry = models['estimation_measurement']
        dependencies = (
            {'joint_count': process.joint_count}
            if role == 'robot'
            else {'spatial_dimension': process.spatial_dimension}
        )
        measurements = measurement_registry.create(
            selected['measurement_model'],
            model_parameters(measurement_registry, selected['measurement_model'], parameters),
            **dependencies,
        )

        def filter_factory(**kwargs):
            return algorithms.create(algorithm, intrinsic, use_defaults=False, **kwargs)

        def adapter_factory(**kwargs):
            return models['estimation_adapter'].create(selected['adapter'], **kwargs)

        if role == 'robot':
            if parameters['joint_bounds_enabled'] and not isinstance(physical, dict):
                raise ValueError('authoritative physical description required for joint bounds')
            return _build_robot_estimator(
                parameters,
                physical,
                process_model=process,
                measurement_models=measurements,
                filter_factory=filter_factory,
                adapter_factory=adapter_factory,
            )
        return _build_target_estimator(
            parameters,
            process_model=process,
            measurement_model=measurements,
            filter_factory=filter_factory,
            adapter_factory=adapter_factory,
        )

    return build


def _component_parameters(parameters, registry, name, *, require_frozen=False):
    """Extract selected numerical leaves from the merged ROS compatibility map."""
    from robot_estimation.parameter_schema import OPTIONAL_PARAMETERS
    from robot_estimation.parameter_schema import PARAMETERS

    info = registry.describe(name)
    keys = set(info['schema']['properties']) if info['schema'] else set(parameters)
    unexpected = set(parameters) - set(PARAMETERS) - set(OPTIONAL_PARAMETERS) - keys
    if unexpected:
        raise ValueError(f'{name}: unknown parameters {sorted(unexpected)}')
    if require_frozen and info['defaults'].get('algorithm_parameters'):
        transports = {key for key in keys if key.endswith('_algorithm_parameters')}
        if not transports.intersection(parameters):
            raise ValueError(
                f'{name}: nonempty intrinsic settings require frozen algorithm parameters')
    if 'algorithm_parameters' in parameters and any(
        key.endswith('_algorithm_parameters') and key in parameters for key in keys
    ):
        raise ValueError(f'{name}: competing intrinsic overrides and frozen algorithm parameters')
    return {key: value for key, value in parameters.items() if key in keys}


def create_robot_estimator(
    parameters, physical=None, estimator_type='robot_ekf', *, registry=None, context=None,
    require_frozen=False,
):
    """Adapt merged ROS parameters to the selected registered robot estimator."""
    registry = registry or create_estimation_registries()['robot_estimator']
    values = _component_parameters(
        parameters, registry, estimator_type, require_frozen=require_frozen)
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


def create_target_estimator(
    parameters, estimator_type='kalman', *, registry=None, context=None, require_frozen=False,
):
    """Adapt merged ROS parameters without restricting context-free library dimensions."""
    registry = registry or create_estimation_registries()['target_estimator']
    values = _component_parameters(
        parameters, registry, estimator_type, require_frozen=require_frozen)
    result = registry.create(estimator_type, values, context=context)
    from robot_estimation.interfaces.estimator_interface import EstimatorInterface

    if not isinstance(result, EstimatorInterface):
        raise TypeError('Registered target estimator must implement EstimatorInterface')
    if context is not None and context.get('target_state_layout') == 'position_velocity':
        if result.state.dimension != 2 * context['spatial_dimension']:
            raise ValueError('Target estimator dimension does not match measurement adapter')
    return result
