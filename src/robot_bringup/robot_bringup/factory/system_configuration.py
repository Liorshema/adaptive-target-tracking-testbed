"""
Validate composition manifests and preflight actual package-owned components.

No ROS imports, algorithm implementations or process startup belong here.
Package-share lookup is supplied by deployment or tests.
"""

from copy import deepcopy
from pathlib import Path

import numpy as np

from robot_bringup.factory.explicit_composition import compile_profile
from robot_bringup.factory.explicit_composition import REFERENCES as EXPLICIT_REFERENCES
from robot_bringup.factory.sensor_inventory import build_sensor_inventory
from robot_configuration.parameter_validation import override_parameters
from robot_configuration.parameter_validation import validate_parameters
from robot_configuration.resources import resolve_resource
from robot_configuration.yaml_loader import load_yaml
from robot_configuration.yaml_loader import load_yaml_group
from robot_control.configuration import load_control_parameters
from robot_control.factory.control_factory import ControlFactory
from robot_control.factory.project_components import create_project_registries
from robot_control.factory.project_configuration import configured_control
from robot_control.parameter_schema import PARAMETERS as CONTROL_PARAMETERS
from robot_estimation.configuration import load_estimation_parameters
from robot_estimation.factory.estimator_factory import create_robot_estimator
from robot_estimation.factory.estimator_factory import create_target_estimator
from robot_estimation.factory.estimator_factory import validate_runtime_parameters
from robot_estimation.parameter_schema import OPTIONAL_PARAMETERS
from robot_estimation.parameter_schema import PARAMETERS as ESTIMATION_PARAMETERS
from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_prediction.interfaces.prediction_data import PredictionInput
from robot_simulation.factory.actuation_factory import create_actuation_components
from robot_simulation.factory.deployment_configuration import validate_deployment_configuration
from robot_simulation.factory.simulation_factory import SimulationFactory

REFERENCES = (
    'robot.description',
    'estimation.robot.config',
    'estimation.target.config',
    'prediction.config',
    'control.config',
    'control.strategy',
    'simulation.config',
)
RUNTIME = {
    'enable_actuation',
    'publish_commands',
    'gui',
    'visualize_tracking',
    'publish_ground_truth',
}


def _mapping(data, allowed, required, path):
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected mapping, got {data!r}')
    unknown, missing = set(data) - set(allowed), set(required) - set(data)
    if unknown or missing:
        raise ValueError(f'{path}: unknown keys {sorted(unknown)}; missing keys {sorted(missing)}')
    return data


def _equal(actual, expected, path):
    if actual != expected:
        raise ValueError(
            f'{path}={actual!r}: required {expected!r} by the deployed component contract'
        )


def _typename(component):
    return type(component).__module__ + '.' + type(component).__name__


def _ros_parameters(path, node, schema, required=(), group=None):
    loader = load_estimation_parameters if node == 'estimation_node' else load_control_parameters
    document = loader(path, group)
    _mapping(document, {node}, {node}, str(path))
    _mapping(document[node], {'ros__parameters'}, {'ros__parameters'}, str(path))
    parameters = document[node]['ros__parameters']
    return validate_parameters(parameters, schema, required=required, path=str(path))


def _resolve_references(document, origin, share):
    """Resolve each declared reference before inheritance merges its origin away."""
    for dotted in (*REFERENCES, *EXPLICIT_REFERENCES):
        keys = dotted.split('.')
        parent = document
        for key in keys[:-1]:
            if not isinstance(parent, dict) or key not in parent:
                break
            parent = parent[key]
        else:
            if not isinstance(parent, dict) or keys[-1] not in parent:
                continue
            value = parent[keys[-1]]
            if dotted in {'control.strategy', 'simulation.localization_sensor'} and value is None:
                continue
            field = keys[-1] + '_group'
            if dotted in {
                'estimation.robot.config',
                'estimation.target.config',
                'control.config',
                'control.strategy',
            } and not isinstance(value, Path) and field not in parent:
                # Clear an inherited group only when this declaration omits it.
                parent[field] = None
            if isinstance(value, dict) and 'group' in value:
                value = deepcopy(value)
                group = value.pop('group')
                field = keys[-1] + '_group'
                if parent.get(field) is not None and parent[field] != group:
                    raise ValueError(f'{origin}:{dotted}: competing parameter group')
                parent[field] = group
            if isinstance(value, Path):
                value = str(value)
            try:
                parent[keys[-1]] = resolve_resource(value, origin.parent, share)
            except ValueError as error:
                raise ValueError(f'{origin}:{dotted}: {error}') from error
    return document


def load_system_configuration(path, package_share, overrides=None):
    """Load a file through the same validated mapping boundary used by experiments."""
    source = Path(path).resolve()
    try:
        document = load_yaml(
            source, transform=lambda data, origin: _resolve_references(data, origin, package_share)
        )
        return assemble_system_configuration(document, source, package_share, overrides)
    except (ValueError, TypeError, KeyError, OSError) as error:
        raise ValueError(f'{source}: {error}') from error


def assemble_system_configuration(document, origin, package_share, overrides=None):
    """
    Validate a detached explicit composition with its resource declaration origin.

    Origin is provenance and the base for relative resources, not a file that must
    exist or be reread. Both entry points invoke exactly the same owner factories.
    """
    source = Path(origin).resolve()
    try:
        data = _resolve_references(deepcopy(document), source, package_share)
        return _assemble(data, source, package_share, {} if overrides is None else overrides)
    except (ValueError, TypeError, KeyError, OSError) as error:
        raise ValueError(f'{source}: {error}') from error


def _assemble(data, source, share, overrides):
    _mapping(
        overrides,
        RUNTIME | {'config', 'strategy_config', 'lookahead', 'feedforward'},
        set(),
        'launch overrides',
    )
    _mapping(
        data,
        {
            'schema_version',
            'robot',
            'estimation',
            'prediction',
            'control',
            'simulation',
            'runtime',
        },
        {'schema_version', 'robot', 'estimation', 'prediction', 'control', 'simulation'},
        'manifest',
    )
    version = data['schema_version']
    if type(version) is not int or version not in (1, 2):
        raise ValueError(f'schema_version={version!r}: supported: integer 1 or 2')
    robot = _mapping(
        data['robot'], {'type', 'description', 'model'}, {'type', 'description', 'model'}, 'robot'
    )
    _equal(robot['type'], 'mobile_manipulator', 'robot.type')
    model = _mapping(
        robot['model'], {'base', 'manipulator'}, {'base', 'manipulator'}, 'robot.model'
    )
    estimation = _mapping(
        data['estimation'],
        {'robot', 'target'} | ({'calibration'} if version == 2 else set()),
        {'robot', 'target'},
        'estimation',
    )
    for name in ['robot', 'target']:
        _mapping(
            estimation[name],
            {'type', 'config', 'config_group', 'parameters'},
            {'type', 'config'},
            f'estimation.{name}',
        )
    prediction = _mapping(
        data['prediction'],
        {'type', 'config', 'adaptation', 'fusion'},
        {'type', 'config', 'adaptation', 'fusion'},
        'prediction',
    )
    control = _mapping(
        data['control'],
        {
            'type',
            'config',
            'strategy',
            'config_group',
            'strategy_group',
            'strategy_overrides',
            'adapter',
            'reference',
        },
        {'type', 'config'},
        'control',
    )
    simulation_keys = (
        {'config'}
        if version == 1
        else {'config', 'robot', 'actuation', 'localization_sensor', 'tracking_tf'}
    )
    required_simulation = {'config'} if version == 1 else simulation_keys - {'localization_sensor'}
    simulation = _mapping(data['simulation'], simulation_keys, required_simulation, 'simulation')
    for label, value in [
        ('robot.type', robot['type']),
        *[('robot.model.' + key, model[key]) for key in model],
        *[('estimation.' + key + '.type', estimation[key]['type']) for key in ['robot', 'target']],
        *[('prediction.' + key, prediction[key]) for key in ['type', 'adaptation', 'fusion']],
        ('control.type', control['type']),
    ]:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{label}={value!r}: expected nonempty component identifier')
    runtime = deepcopy(data.get('runtime', {}))
    _mapping(runtime, RUNTIME, set(), 'runtime')
    for key, value in runtime.items():
        if type(value) is not bool:
            raise ValueError(f'runtime.{key}={value!r}: expected boolean')
    for key, value in overrides.items():
        if key in RUNTIME and value != '':
            if value not in ['true', 'false']:
                raise ValueError(f'runtime.{key}={value!r}: launch override must be true or false')
            runtime[key] = value == 'true'
    for key in ['enable_actuation', 'gui', 'visualize_tracking']:
        runtime.setdefault(key, False)
    runtime.setdefault('publish_commands', runtime['enable_actuation'])
    if runtime['publish_commands'] and not runtime['enable_actuation']:
        raise ValueError('runtime.publish_commands=true requires explicit enable_actuation=true')
    if runtime['visualize_tracking']:
        if runtime.get('publish_ground_truth') is False:
            raise ValueError('runtime.visualize_tracking requires publish_ground_truth=true')
        runtime['publish_ground_truth'] = True

    # Existing bringup profile supplies deployment resources, not algorithm implementations.
    composition = None
    if version == 2:
        profile_path = source  # This manifest owns the in-memory compatibility adapter.
        profile, composition = compile_profile(data, source, share, overrides)
    else:
        profile_path = simulation['config']
        if overrides.get('config'):
            profile_path = resolve_resource(overrides['config'], share('robot_bringup'), share)
        profile = load_yaml(profile_path)
    allowed = {
        'simulation_config',
        'actuation_config',
        'control_config',
        'robot_estimator_config',
        'target_estimator_config',
        'tracking_tf',
        'estimation_overrides',
    }
    _mapping(
        profile, allowed, allowed - {'estimation_overrides'}, f'simulation.config ({profile_path})'
    )
    profile = deepcopy(profile)
    profile['simulation_config'] = resolve_resource(
        profile['simulation_config'], share('robot_simulation'), share
    )
    profile['actuation_config'] = resolve_resource(
        profile['actuation_config'], share('robot_simulation'), share
    )
    profile['control_config'] = control['config']
    profile['robot_estimator_config'] = estimation['robot']['config']
    profile['target_estimator_config'] = estimation['target']['config']
    if 'estimation_overrides' in profile:
        profile['estimation_overrides'] = resolve_resource(
            profile['estimation_overrides'], share('robot_bringup'), share
        )
    sim = load_yaml(profile['simulation_config'])
    base = sim['paths']['base']
    if base not in {'package', 'config'}:
        raise ValueError(f'simulation.paths.base={base!r}: supported package, config')
    resource = (
        share('robot_simulation') if base == 'package' else profile['simulation_config'].parent
    )
    robot_path = resolve_resource(sim['robot']['config'], resource, share)
    deployed = load_yaml(robot_path)['robot']
    physical_path = resolve_resource(
        deployed['description']['config'], share(deployed['description']['package']), share
    )
    _equal(
        robot['description'],
        physical_path,
        'robot.description versus simulation robot description',
    )
    description = load_yaml(physical_path)
    physical = description['robot']['manipulator']['joints']
    names = [deployed['joint_name_template'].format(key=key) for key in physical]
    if len(set(names)) != len(names) or not names:
        raise ValueError('robot.description: invalid joint names/order')
    drive = deployed['differential_drive']
    if sim['robot']['enabled'] is not True or drive['enabled'] is not True:
        raise ValueError(
            'simulation.robot requires enabled robot and differential drive for this deployment'
        )
    for reference in [
        sim['world']['source'],
        sim['world']['terrain_config'],
        sim['robot']['joint_position_hold'],
    ]:
        resolve_resource(reference, resource, share)
    resolve_resource(
        deployed['description']['xacro'], share(deployed['description']['package']), share
    )
    holding = load_yaml(resolve_resource(sim['robot']['joint_position_hold'], resource, share))[
        'joint_position_hold'
    ]
    _equal(holding['enabled'], True, 'simulation.joint_position_hold.enabled')
    _equal(
        set(holding['positions']), set(physical), 'simulation.joint_position_hold.positions keys'
    )
    for name, angle in holding['positions'].items():
        if (
            not np.isfinite(angle)
            or not physical[name]['lower'] <= angle <= physical[name]['upper']
        ):
            raise ValueError(
                f'simulation.joint_position_hold.positions.{name}={angle!r}: '
                'outside authoritative limits'
            )
    params = {}
    parameter_sources = {}
    for name in ['robot', 'target']:
        parameters = _ros_parameters(
            estimation[name]['config'],
            'estimation_node',
            {**ESTIMATION_PARAMETERS, **OPTIONAL_PARAMETERS},
            group=estimation[name].get('config_group'),
        )
        if 'parameters' in estimation[name]:
            parameters.update(
                validate_parameters(
                    estimation[name]['parameters'],
                    {**ESTIMATION_PARAMETERS, **OPTIONAL_PARAMETERS},
                    path=f'estimation.{name}.parameters',
                )
            )
        overlap = params.keys() & parameters.keys()
        if overlap:
            raise ValueError(
                f'estimation.{name}.config: competing parameter owners {sorted(overlap)}'
            )
        params.update(parameters)
        parameter_sources.update({key: str(estimation[name]['config']) for key in parameters})
    if 'estimation_overrides' in profile:
        overrides_data = load_yaml(profile['estimation_overrides'])
        overrides_data = validate_parameters(
            overrides_data,
            {**ESTIMATION_PARAMETERS, **OPTIONAL_PARAMETERS},
            path=str(profile['estimation_overrides']),
        )
        params.update(overrides_data)
        parameter_sources.update(
            {key: str(profile['estimation_overrides']) for key in overrides_data}
        )
    # Grouped model defaults do not choose a physical robot variant. Bind their
    # placeholder to the manifest's authoritative description unless a caller
    # supplied an explicit competing description (which still must be rejected).
    robot_owner = estimation['robot']
    if (
        robot_owner.get('config_group') is not None
        and parameter_sources.get('robot_description_config') == str(robot_owner['config'])
        and 'robot_description_config' not in robot_owner.get('parameters', {})
    ):
        package_root = Path(share(params['robot_description_package'])).resolve()
        if physical_path.is_relative_to(package_root):
            params['robot_description_config'] = str(physical_path.relative_to(package_root))
            parameter_sources['robot_description_config'] = str(source) + ':robot.description'
        else:
            params['robot_description_file'] = str(physical_path)
            parameter_sources['robot_description_file'] = str(source) + ':robot.description'
    params = validate_parameters(
        params,
        {**ESTIMATION_PARAMETERS, **OPTIONAL_PARAMETERS},
        required=ESTIMATION_PARAMETERS,
        path='estimation.parameters',
    )
    validate_runtime_parameters(params)
    for name, parameter in params.items():
        if (
            name.endswith(('_variance', '_noise_std'))
            and isinstance(parameter, (int, float))
            and parameter < 0
        ):
            raise ValueError(f'estimation.{name}: uncertainty must be nonnegative')
    variance = np.asarray(params['target_position_variance'])
    if variance.shape != (3,) or np.any(variance < 0):
        raise ValueError(
            'estimation.target_position_variance: requires three nonnegative variances'
        )
    for key, value in {
        'joint_count': len(names),
        'joint_names': names,
        'wheel_odom_frame': drive['frame_id'],
        'base_frame': drive['child_frame_id'],
        'robot_state_frame_id': drive['frame_id'],
        'wheel_odometry_topic': drive['odom_topic'],
        'joint_states_topic': deployed['joint_states']['topic'],
    }.items():
        _equal(params[key], value, 'estimation.' + key)
    _equal(params['target_spatial_dimension'], 3, 'estimation.target_spatial_dimension')
    if params['robot_description_file']:
        _equal(
            Path(params['robot_description_file']).resolve(),
            physical_path,
            'estimation.robot_description_file',
        )
    else:
        _equal(
            resolve_resource(
                params['robot_description_config'],
                share(params['robot_description_package']),
                share,
            ),
            physical_path,
            'estimation.robot_description_config',
        )
    try:
        robot_estimator, _ = create_robot_estimator(
            params, physical, estimation['robot']['type'], context={
                'robot_state_layout': 'planar_mobile_manipulator',
                'robot_measurement_adapter': 'wheel_imu_joint_pose'})
        target_estimator = create_target_estimator(
            params, estimation['target']['type'], context={
                'target_state_layout': 'position_velocity',
                'target_measurement_adapter': 'position', 'spatial_dimension': 3})
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f'estimation: {error}') from error
    _equal(robot_estimator.state.dimension, 5 + 2 * len(names), 'estimation.robot state dimension')
    _equal(target_estimator.state.dimension, 6, 'estimation.target state dimension')

    control_parameters = _ros_parameters(
        control['config'], 'control_node', CONTROL_PARAMETERS, group=control.get('config_group')
    )
    if control.get('config_group') is not None or 'reference' in control:
        reference = _mapping(
            control.get('reference'),
            {'geometry', 'policy'},
            {'geometry', 'policy'},
            'control.reference',
        )
        if 'adapter' not in control:
            raise ValueError('control.adapter: explicit selection is required')
        control_parameters.update(
            controller_type=control['type'],
            base_model_type=model['base'],
            manipulator_model_type=model['manipulator'],
            kinematics_adapter_type=control['adapter'],
            reference_geometry_type=reference['geometry'],
            reference_policy_type=reference['policy'],
        )
    _equal(control_parameters['joint_count'], len(names), 'control.joint_count')
    if control_parameters.get('command_joint_names'):
        _equal(control_parameters['command_joint_names'], names, 'control.command_joint_names')
    if control_parameters.get('command_frame_id'):
        _equal(
            control_parameters['command_frame_id'],
            drive['child_frame_id'],
            'control.command_frame_id',
        )
    # ROS launch cannot infer the element type of the owner's empty placeholder.
    # This field is deployment metadata, always derived from authoritative joints.
    control_parameters['command_joint_names'] = names
    for category in ['base', 'manipulator']:
        _equal(
            control_parameters[category + '_model_type'],
            model[category],
            'robot.model.' + category,
        )
    _equal(
        control_parameters['robot_state_topic'],
        params['robot_state_full_topic'],
        'control.robot_state_topic',
    )
    if not control_parameters.get('compute_enabled') or not control_parameters.get(
        'safety_enabled'
    ):
        raise ValueError('control.config must enable computation and independent safety')
    if control_parameters.get('robot_description_file'):
        _equal(
            Path(control_parameters['robot_description_file']).resolve(),
            physical_path,
            'control.robot_description_file',
        )
    strategy_path = control.get('strategy')
    if overrides.get('strategy_config'):
        requested = overrides['strategy_config']
        strategy_path = (
            None
            if requested == 'none'
            else resolve_resource(requested, share('robot_control'), share)
        )
    strategy_group = None if overrides.get('strategy_config') else control.get('strategy_group')
    strategy = load_control_parameters(strategy_path, strategy_group) if strategy_path else None
    if overrides.get('strategy_config'):
        control_parameters['controller_type'] = (
            strategy['controller']['type'] if strategy else 'jacobian'
        )
    if strategy is not None and strategy_group is not None:
        strategy['controller']['type'] = control['type']
    if 'strategy_overrides' in control and not overrides.get('strategy_config'):
        if strategy is None:
            raise ValueError('control.strategy_overrides requires a selected strategy')
        strategy = override_parameters(
            strategy, control['strategy_overrides'], path='control.strategy_overrides'
        )
    try:
        control_config = configured_control(
            control_parameters,
            description,
            strategy,
            lookahead=overrides.get('lookahead', ''),
            feedforward=overrides.get('feedforward', ''),
        )
        if not overrides.get('strategy_config'):
            _equal(control['type'], control_config['controller']['type'], 'control.type')
        pipeline = ControlFactory(create_project_registries()).build(control_config)
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"control ({control['config']}, {strategy_path}): {error}") from error

    selection = {
        'robot_estimator_type': estimation['robot']['type'],
        'target_estimator_type': estimation['target']['type'],
        'predictor_type': prediction['type'],
        'adaptation_type': prediction['adaptation'],
        'fusion_type': prediction['fusion'],
    }
    try:
        prediction_config = PredictionConfig(prediction['config'])
        system = PredictionSystemFactory(prediction_config).create(
            **{k: selection[k] for k in ['predictor_type', 'adaptation_type', 'fusion_type']}
        )
        settings = prediction_config.get('prediction')
        if settings['dt'] * settings['horizon'] < control_config['tracking']['lookahead_time']:
            raise ValueError('prediction horizon must cover the selected control lookahead')
        sample = system.predict(
            PredictionInput(
                state=target_estimator.state.mean,
                timestamp=0.0,
                dt=settings['dt'],
                horizon=settings['horizon'],
            )
        )
        if (
            sample.states.shape != (settings['horizon'], 6)
            or not np.all(np.isfinite(sample.states))
            or not np.array_equal(
                sample.timestamps, settings['dt'] * np.arange(1, settings['horizon'] + 1)
            )
        ):
            raise ValueError(
                'deployed prediction must preserve finite [p,v] and absolute future timestamps'
            )
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"prediction ({prediction['config']}): {error}") from error

    # Invoke the existing simulation registry; no scenario-specific branch is needed.
    target = load_yaml(resolve_resource(sim['target']['config'], resource, share))
    environment = load_yaml(
        resolve_resource(sim['simulation']['environment_config'], resource, share)
    )
    integration = load_yaml(resolve_resource(sim['simulation']['config'], resource, share))
    try:
        engine, _ = SimulationFactory.with_defaults().create(target, environment, integration)
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f'simulation dynamics/integration: {error}') from error
    for key in ['initial_position', 'initial_velocity']:
        vector = np.asarray(sim['target'][key], dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f'simulation.target.{key}: deployed target requires finite 3-vector')
    measurements = load_yaml(
        resolve_resource(sim['simulation']['measurement_config'], resource, share)
    )
    try:
        validate_deployment_configuration(sim, target, environment, integration, measurements)
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"simulation ({profile['simulation_config']}): {error}") from error
    if sim.get('physical_context'):
        truth = deployed['ground_truth_odometry']
        ctx = sim['physical_context']
        if (
            runtime.get('publish_ground_truth', truth['enabled']) is not True
            or truth['topic'] != ctx['base_topic']
            or truth['frame_id'] != ctx['reference_frame']
        ):
            raise ValueError('physical guidance requires enabled matching physical ground truth')
    sensor_config = sim.get('localization_sensor_config')
    sensor_path = None
    localization_sensor = None
    if sensor_config:
        from robot_simulation.measurements.planar_localization import validate_configuration

        sensor_path = resolve_resource(sensor_config, resource, share)
        localization_sensor = load_yaml(sensor_path)['localization_sensor']
        validate_configuration(localization_sensor)
        if localization_sensor['enabled']:
            truth = deployed['ground_truth_odometry']
            if (
                not runtime.get('publish_ground_truth', truth['enabled'])
                or localization_sensor['source_topic'] != truth['topic']
                or localization_sensor['source_frame'] != truth['frame_id']
                or localization_sensor['child_frame'] != truth['child_frame_id']
            ):
                raise ValueError(
                    'Localization simulation requires matching enabled physical source'
                )
        if params.get('localization_enabled', False) and (
            params['localization_topic'] != localization_sensor['topic']
            or params['localization_frame'] != localization_sensor['source_frame']
        ):
            raise ValueError(
                'Localization measurement topic/frame do not match the selected sensor'
            )
    if (
        params.get('localization_enabled', False)
        and params['localization_topic'] == deployed['ground_truth_odometry']['topic']
    ):
        raise ValueError(
            'Ground truth is not an estimator measurement; select the imperfect sensor output'
        )
    sensor = measurements['measurements']['target_position']
    _equal(sensor['enabled'], True, 'simulation.measurements.target_position.enabled')
    _equal(
        sensor['observation']['components'], ['position'], 'simulation.measurements.observation'
    )
    _equal(sensor['topic'], params['target_measurement_topic'], 'simulation.measurements.topic')
    actuator = load_yaml(profile['actuation_config'])['actuation']
    _equal(
        actuator['command_topic'], control_parameters['command_topic'], 'actuation.command_topic'
    )
    try:
        create_actuation_components(
            actuator,
            deployed,
            physical,
            holding,
            control_parameters['base_velocity_lower'],
            control_parameters['base_velocity_upper'],
            enabled=runtime['enable_actuation'],
        )
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"actuation ({profile['actuation_config']}): {error}") from error
    identities = {
        'robot_estimator': _typename(robot_estimator),
        'target_estimator': _typename(target_estimator),
        **system.component_types,
        **pipeline.component_types,
        'dynamics': _typename(engine.dynamics),
        'integrator': _typename(engine.integrator),
    }
    sensor_inventory = build_sensor_inventory(
        deployment=sim,
        robot=deployed,
        parameters=params,
        measurements=measurements,
        integration=integration,
        localization=localization_sensor,
        runtime=runtime,
        tracking_tf=profile['tracking_tf'],
        actuation=actuator,
        context_required=bool(engine.dynamics.environment_field.context_requirements),
        resources={
            'system': source,
            'profile': profile_path,
            'simulation': profile['simulation_config'],
            'robot': robot_path,
            'description': physical_path,
            'localization': sensor_path,
            'robot_estimator': estimation['robot']['config'],
            'target_estimator': estimation['target']['config'],
            'estimation_overrides': profile.get('estimation_overrides'),
            'actuation': profile['actuation_config'],
            'integration': resolve_resource(sim['simulation']['config'], resource, share),
            'target_measurements': resolve_resource(
                sim['simulation']['measurement_config'], resource, share
            ),
        },
        parameter_sources=parameter_sources,
    )
    result = {
        'source': source,
        'manifest': deepcopy(data),
        'profile': profile,
        'profile_path': profile_path,
        'description_path': physical_path,
        'prediction_path': prediction['config'],
        'strategy_path': strategy_path,
        'strategy': deepcopy(strategy),
        'strategy_group': strategy_group,
        'selection': selection,
        'runtime': runtime,
        'estimation_parameters': params,
        'control_parameters': control_parameters,
        'identities': identities,
        'sensor_inventory': sensor_inventory,
        'prediction_topics': {
            'estimated_state_topic': params['target_state_full_topic'],
            'trajectory_full_topic': control_parameters['prediction_topic'],
        },
    }
    if composition is not None:
        result['composition'] = composition
    return result
