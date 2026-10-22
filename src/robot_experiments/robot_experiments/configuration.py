"""Validate experiment composition and materialize seed-owned replay inputs."""
from copy import deepcopy
import hashlib
from pathlib import Path
import re

import yaml

from robot_configuration.yaml_loader import load_yaml
from robot_configuration.resources import resolve_resource
from robot_bringup.factory.system_configuration import load_system_configuration

METRICS = {'tracking_distance', 'pointing_error', 'prediction_rmse', 'estimation_error',
           'joint_speed_violation', 'joint_position_violation', 'realtime_factor', 'control_compute'}
DEFAULT_EXECUTION = dict(readiness_timeout=45., stall_timeout=12., wall_timeout=150.,
                         shutdown_timeout=5., max_skew=.08, failure_policy='continue', record_actuator_trace=False)


def mapping(value, allowed, required, label):
    if not isinstance(value, dict):
        raise ValueError(f'{label}: expected mapping')
    if set(value)-set(allowed) or set(required)-set(value):
        raise ValueError(f'{label}: unknown keys {sorted(set(value)-set(allowed))}; missing {sorted(set(required)-set(value))}')
    return value


def positive(value, label, zero=False):
    import math
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (not zero and value == 0):
        raise ValueError(f'{label}: expected finite {"nonnegative" if zero else "positive"} number')
    return float(value)


def load_experiment(path, share, output_override=None):
    """Resolve variant resources at their declaring document before inheritance."""
    source = Path(path).resolve()
    def normalize(data, origin):
        experiment = data.get('experiment')
        if isinstance(experiment, dict) and isinstance(experiment.get('variants'), list):
            for variant in experiment['variants']:
                if isinstance(variant, dict) and 'system_config' in variant:
                    variant['system_config'] = resolve_resource(variant['system_config'], origin.parent, share)
        return data
    try:
        data = load_yaml(source, transform=normalize)
        mapping(data, {'schema_version', 'experiment'}, {'schema_version', 'experiment'}, str(source))
        if type(data['schema_version']) is not int or data['schema_version'] != 1:
            raise ValueError('schema_version: supported integer 1')
        ex = mapping(data['experiment'], {'name','description','duration','settling_time','seeds','variants','metrics','output','execution','runtime'},
                     {'name','duration','settling_time','seeds','variants','metrics','output'}, 'experiment')
        if not isinstance(ex['name'], str) or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}', ex['name']):
            raise ValueError('experiment.name: use 1-80 alphanumeric, underscore or hyphen characters')
        if 'description' in ex and not isinstance(ex['description'], str):
            raise ValueError('experiment.description: expected string')
        ex['duration'] = positive(ex['duration'], 'duration')
        ex['settling_time'] = positive(ex['settling_time'], 'settling_time', zero=True)
        seeds = ex['seeds']
        if not isinstance(seeds,list) or not seeds or any(type(v) is not int or not 0 <= v < 2**32 for v in seeds) or len(seeds)!=len(set(seeds)):
            raise ValueError('seeds: expected nonempty unique uint32 list')
        if not isinstance(ex['metrics'],list) or not ex['metrics'] or any(not isinstance(v,str) or v not in METRICS for v in ex['metrics']) or len(ex['metrics'])!=len(set(ex['metrics'])):
            raise ValueError(f'metrics: supported {sorted(METRICS)} without duplicates')
        mapping(ex['output'], {'directory'}, {'directory'}, 'output')
        if not isinstance(ex['output']['directory'],str) or not ex['output']['directory'].strip():
            raise ValueError('output.directory: expected nonempty path')
        ex['output']['directory'] = Path(output_override or ex['output']['directory']).expanduser().resolve()
        runtime = mapping(ex.get('runtime',{}), {'enable_actuation'}, set(), 'runtime')
        if 'enable_actuation' in runtime and type(runtime['enable_actuation']) is not bool:
            raise ValueError('runtime.enable_actuation: expected boolean')
        ex['runtime'] = {'enable_actuation':runtime.get('enable_actuation',False)}
        execution = mapping(ex.get('execution',{}), DEFAULT_EXECUTION, set(), 'execution')
        ex['execution'] = {**DEFAULT_EXECUTION,**execution}
        for key in DEFAULT_EXECUTION.keys()-{'failure_policy', 'record_actuator_trace'}:
            ex['execution'][key] = positive(ex['execution'][key], 'execution.'+key)
        if type(ex['execution']['record_actuator_trace']) is not bool:
            raise ValueError('execution.record_actuator_trace: expected boolean')
        if ex['execution']['failure_policy'] not in {'continue','abort'}:
            raise ValueError('execution.failure_policy: supported continue, abort')
        variants = ex['variants']
        if not isinstance(variants,list) or not variants:
            raise ValueError('variants: expected nonempty list')
        names = set()
        for v in variants:
            mapping(v, {'name','system_config'}, {'name','system_config'}, 'variant')
            if not isinstance(v['name'],str) or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}',v['name']) or v['name'] in names:
                raise ValueError('variant.name: expected unique safe identifier')
            names.add(v['name'])
            v['plan'] = load_system_configuration(v['system_config'],share,overrides={
                'enable_actuation':str(ex['runtime']['enable_actuation']).lower(),
                'publish_ground_truth':'true','gui':'false','visualize_tracking':'false'})
        ex['source'] = source
        return ex
    except (ValueError, TypeError, KeyError, OSError) as error:
        raise ValueError(f'{source}: {error}') from error


def _write(path, data):
    path.write_text(yaml.safe_dump(data,sort_keys=False))
    return str(path)


def materialize_run(variant, seed, directory, share, runtime, record_actuator_trace=False):
    """Snapshot selected owner values and wire the supported sensor RNG to this seed.

    References outside the run are immutable installed owner resources, whose
    contents/hashes are captured. No owner source file is modified.
    """
    directory = Path(directory).resolve()
    inputs = directory/'inputs';inputs.mkdir()
    plan = variant['plan']; profile = deepcopy(plan['profile'])
    snapshots = {}
    def read(path):
        path = Path(path).resolve()
        value = load_yaml(path)
        snapshots[str(path)] = {'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'resolved':deepcopy(value)}
        declared=yaml.safe_load(path.read_text())
        parents=declared.get('extends',[]) if isinstance(declared,dict) else []
        if isinstance(parents,str):parents=[parents]
        for parent in parents:
            parent_path=(path.parent/parent).resolve()
            if str(parent_path) not in snapshots:read(parent_path)
        return value
    original_manifest=read(plan['source'])
    read(plan['profile_path'])
    for owner,entry in [('estimation',original_manifest['estimation']['robot']),
                        ('estimation',original_manifest['estimation']['target']),
                        ('control',original_manifest['control'])]:
        reference=entry['config']
        # Package references are source independent; merged values themselves are
        # captured below even when declaring-file-relative inheritance is used.
        if isinstance(reference,dict):read(resolve_resource(reference,Path(plan['source']).parent,share))
    sim = read(profile['simulation_config'])
    base = share('robot_simulation') if sim['paths']['base']=='package' else Path(profile['simulation_config']).parent
    sim['world']['source'] = str(resolve_resource(sim['world']['source'],base,share))
    world_path=Path(sim['world']['source'])
    snapshots[str(world_path)]={'sha256':hashlib.sha256(world_path.read_bytes()).hexdigest(),'text':world_path.read_text()}
    for section,key in [('world','terrain_config'),('robot','config'),('robot','joint_position_hold'),
                        ('target','config'),('simulation','config'),('simulation','environment_config'),('simulation','measurement_config')]:
        resource=resolve_resource(sim[section][key],base,share)
        value=read(resource)
        sim[section][key]=str(resource)
    localization_sensor = None
    if sim.get('localization_sensor_config'):
        localization_sensor = read(resolve_resource(sim['localization_sensor_config'],base,share))
        localization_sensor['localization_sensor']['random_seed'] = int.from_bytes(
            hashlib.sha256(f'{seed}:localization'.encode()).digest()[:4], 'little')
        sim['localization_sensor_config'] = _write(inputs/'localization_sensor.yaml',localization_sensor)
    original_holding = read(sim['robot']['joint_position_hold'])
    if record_actuator_trace:
        holding = deepcopy(original_holding)
        holding['joint_position_hold']['trace_directory'] = str(directory/'physics')
        sim['robot']['joint_position_hold'] = _write(inputs/'holding.yaml', holding)
    robot=read(sim['robot']['config'])['robot']
    description=read(plan['description_path'])
    description_file=_write(inputs/'robot_description.yaml',description)
    physical_deployment=deepcopy(robot)
    physical_deployment.pop('description')
    robot['description']['config']=description_file
    sim['robot']['config']=_write(inputs/'robot.yaml',{'robot':robot})
    measurements=read(sim['simulation']['measurement_config'])
    # The only deployed RNG consumer: SyntheticMeasurementGenerator in simulation.
    measurements['measurements']['target_position']['random_seed']=seed
    sim['simulation']['measurement_config']=_write(inputs/'measurements.yaml',measurements)
    sim['paths']['base']='config'
    simulation_file=inputs/'simulation.yaml';_write(simulation_file,sim)
    profile['simulation_config']=str(simulation_file)
    for key,package in [('actuation_config','robot_simulation'),('estimation_overrides','robot_bringup')]:
        if key in profile:
            path=resolve_resource(str(profile[key]),share(package),share)
            read(path);profile[key]=str(path)
    # Calibration overrides were already applied by the M07 owner preflight.
    profile.pop('estimation_overrides',None)
    estimation_parameters=deepcopy(plan['estimation_parameters'])
    estimation_parameters['robot_description_file']=description_file
    control_parameters=deepcopy(plan['control_parameters'])
    control_parameters['robot_description_file']=description_file
    est=_write(inputs/'estimation.yaml',{'estimation_node':{'ros__parameters':estimation_parameters}})
    target_est=_write(inputs/'target_estimation.yaml',{'estimation_node':{'ros__parameters':{}}})
    control=_write(inputs/'control.yaml',{'control_node':{'ros__parameters':control_parameters}})
    profile.update(control_config=control,robot_estimator_config=est,target_estimator_config=target_est)
    profile_path=_write(inputs/'profile.yaml',profile)
    source=read(plan['source'])
    manifest={'schema_version':1,'robot':{'type':source['robot']['type'],'description':description_file,'model':source['robot']['model']},
        'estimation':{'robot':{'type':plan['selection']['robot_estimator_type'],'config':est},
                      'target':{'type':plan['selection']['target_estimator_type'],'config':target_est}},
        'prediction':{'type':plan['selection']['predictor_type'],'config':str(plan['prediction_path']),
                      'adaptation':plan['selection']['adaptation_type'],'fusion':plan['selection']['fusion_type']},
        'control':{'type':source['control']['type'],'config':control,'strategy':str(plan['strategy_path']) if plan['strategy_path'] else None},
        'simulation':{'config':profile_path},'runtime':{**plan['runtime'],'enable_actuation':runtime['enable_actuation'],
         'publish_ground_truth':True,'gui':False,'visualize_tracking':False}}
    prediction_configuration=read(plan['prediction_path'])
    if plan['strategy_path']:read(plan['strategy_path'])
    system_path=inputs/'system.yaml';_write(system_path,manifest)
    resolved=load_system_configuration(system_path,share)
    sensor=measurements['measurements']['target_position'];estparams=plan['estimation_parameters'];ctrl=plan['control_parameters']
    integration=read(sim['simulation']['config'])
    topics={'clock':'/clock','target_truth':integration['simulation']['ground_truth']['topic'],
      'target_measurement':sensor['topic'],'joints':robot['joint_states']['topic'],'base_truth':robot['ground_truth_odometry']['topic'],
      'odometry':robot['differential_drive']['odom_topic'],'robot_estimate':estparams['robot_state_full_topic'],
      'target_estimate':estparams['target_state_full_topic'],'prediction':ctrl['prediction_topic'],
      'control':ctrl['telemetry_topic'],'command':ctrl['command_topic'],'actuation':read(profile['actuation_config'])['actuation']['diagnostic_topic'],
      'diagnostics':estparams['diagnostic_topic']}
    if localization_sensor:
        topics['localization_measurement']=localization_sensor['localization_sensor']['topic']
    context={'description':description,'robot':robot,'control':ctrl,'joint_names':ctrl['command_joint_names'],
       'world_frame':sim['tracking']['reference_frame'],'estimator_frame':estparams['robot_state_frame_id'],
       'target_frame':sim['tracking']['reference_frame'],'topics':topics,
       'physical_numerical_tolerance':.001}
    comparison_contract={'description':description,'robot':physical_deployment,
      'target_initial_position':sim['target']['initial_position'],'target_initial_velocity':sim['target']['initial_velocity'],
      'target_dynamics':read(sim['target']['config']),'environment':read(sim['simulation']['environment_config']),
      'integration':integration,'measurements':measurements,'tracking':sim['tracking'],
      'physical_context':deepcopy(sim.get('physical_context')),
      'prediction_sampling':{key:prediction_configuration['prediction'][key] for key in ['dt','horizon']},
      'holding':original_holding,'terrain':read(sim['world']['terrain_config']),
      'world_sha256':snapshots[str(world_path)]['sha256'],'actuation':read(profile['actuation_config']),
      'tracking_reference':{key:ctrl[key] for key in ['tracking_radius','tracking_forward_axis','base_velocity_lower','base_velocity_upper']}}
    if localization_sensor:
        comparison_contract['localization_sensor']=localization_sensor
    snapshot={'schema_version':1,'resources':snapshots,'seed_consumers':{'measurements.target_position.random_seed':seed},
      'initial_conditions':{'robot_pose':robot['pose'],'target_position':sim['target']['initial_position'],'target_velocity':sim['target']['initial_velocity']},
      'reproducibility':'Owner config replay and sensor RNG control; asynchronous ROS/Gazebo, contact/skid dynamics and reactive target trajectories are not bitwise deterministic.'}
    if localization_sensor:
        snapshot['seed_consumers']['localization_sensor.random_seed']=localization_sensor['localization_sensor']['random_seed']
    return {'plan':resolved,'context':context,'snapshot':snapshot,'system_path':system_path,'simulation_path':simulation_file,
            'comparison_contract':comparison_contract}
