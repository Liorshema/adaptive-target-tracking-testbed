"""Describe resolved sensing contracts; never discover publishers or alter configuration.

This is an assembly-time view of the existing built-in topology, not a sensor
registry. Producer selection and estimator subscription are not live health.
"""

from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


def build_sensor_inventory(
    *, deployment: Mapping[str, Any], robot: Mapping[str, Any],
    parameters: Mapping[str, Any], measurements: Mapping[str, Any],
    integration: Mapping[str, Any], localization: Mapping[str, Any] | None,
    runtime: Mapping[str, Any], tracking_tf: Mapping[str, Any],
    actuation: Mapping[str, Any], context_required: bool,
    resources: Mapping[str, Path | None], parameter_sources: Mapping[str, str],
) -> dict[str, Any]:
    """Build detached JSON metadata from validated, final owner values.

    Resource provenance names selected documents, including applied calibration
    overrides. It does not claim to identify a key's declaring ancestor within
    an inherited document. Unknown rates and covariance stay explicit.
    """
    owners = {name: str(path) if path is not None else 'not configured'
              for name, path in resources.items()}

    def configuration(owner: str | None, keys: tuple[str, ...] = ()) -> dict[str, Any]:
        return {'producer_resource': owners.get(owner, 'not configured'),
                'parameter_sources': {key: parameter_sources[key] for key in keys
                                      if key in parameter_sources}}

    p = parameters
    drive = robot['differential_drive']
    truth = robot.get('ground_truth_odometry', {})
    tracking = deployment['tracking']
    target = measurements['measurements']['target_position']
    robot_selected = deployment['robot']['enabled']
    truth_selected = robot_selected and runtime.get(
        'publish_ground_truth', truth.get('enabled', False))
    tf_selected = tracking['source'] == 'tf'
    physical_tf = tf_selected and tracking.get('pose_source') == 'physical'
    pose_selected = localization is not None and localization['enabled']
    pose_fusion = p.get('localization_enabled', False)
    robot_timing_parameters = (
        'reorder_holdback', 'reorder_max_age', 'reorder_rate_hz',
        'reorder_capacity', 'future_tolerance')
    robot_time = {key: p[key] for key in robot_timing_parameters}
    robot_time['stamp'] = 'acquisition time; shared bounded robot reorder buffer'
    feedback = {'required_for_full_robot_state': True,
                'required_for_active_actuation': True,
                'actuation_enabled': runtime['enable_actuation'],
                'feedback_max_age': actuation['feedback_max_age'],
                'wall_timeout': actuation['wall_timeout'],
                'configuration_resource': owners['actuation']}

    # Feedback observations use configured R, not Odometry/Imu message covariance.
    sources = {
        'wheel_odometry': {
            'name': 'Wheel odometry', 'classification': 'robot observation',
            'producer': {'configured': robot_selected and drive['enabled'],
                         'implementation': 'Gazebo DiffDrive via ros_gz_bridge',
                         'rate_hz': drive['rate_hz']},
            'fusion': {'enabled': True, 'subscription_configured': True,
                       'consumer': 'RobotStateEstimator.update_wheel_odometry'},
            'external_provider': 'not selected; built-in feedback required by this deployment',
            'transport': {'topic': p['wheel_odometry_topic'],
                          'message_type': 'nav_msgs/msg/Odometry',
                          'frames': {'source': p['wheel_odom_frame'], 'child': p['base_frame']}},
            'measurement': {'fields': ['twist.linear.x', 'twist.angular.z'],
                            'assumed_variance': [p['wheel_linear_velocity_variance'],
                                                 p['wheel_angular_velocity_variance']],
                            'covariance_source': ('configured diagonal measurement R; '
                                                  'message covariance ignored')},
            'timing': {**robot_time, 'max_wheel_odometry_age': p['max_wheel_odometry_age']},
            'safety_feedback': feedback,
            'consumers': (['robot estimation', 'actuator feedback']
                          + (['tracking FK'] if tf_selected and not physical_tf else [])),
            'configuration': configuration('robot', (
                'wheel_linear_velocity_variance', 'wheel_angular_velocity_variance',
                'wheel_odometry_topic', 'wheel_odom_frame', 'base_frame',
                'max_wheel_odometry_age', *robot_timing_parameters)),
        },
        'joint_feedback': {
            'name': 'Joint feedback', 'classification': 'robot observation',
            'producer': {'configured': robot_selected,
                         'implementation': 'Gazebo JointStatePublisher via ros_gz_bridge',
                         'rate_hz': 'not configured'},
            'fusion': {'enabled': True, 'subscription_configured': True,
                       'consumer': 'RobotStateEstimator.update_joint_encoders'},
            'external_provider': 'not selected; built-in feedback required by this deployment',
            'transport': {'topic': p['joint_states_topic'],
                          'message_type': 'sensor_msgs/msg/JointState',
                          'frames': 'joint-name coordinates; no Cartesian frame transform'},
            'measurement': {'fields': ['position', 'velocity'], 'joint_names': p['joint_names'],
                            'position_variance': p['joint_position_variance'],
                            'velocity_variance': p['joint_velocity_variance'],
                            'covariance_source': ('configured diagonal measurement R '
                                                  'in description joint order')},
            'timing': {**robot_time, 'max_joint_state_age': p['max_joint_state_age']},
            'safety_feedback': feedback,
            'consumers': (['robot estimation and raw joint-bound gate', 'actuator feedback']
                          + (['tracking FK'] if tf_selected else [])),
            'configuration': configuration('robot', (
                'joint_names', 'joint_count', 'joint_states_topic', 'joint_position_variance',
                'joint_velocity_variance', 'max_joint_state_age', *robot_timing_parameters)),
        },
        'imu': {
            'name': 'IMU yaw rate', 'classification': 'optional robot observation',
            'producer': {'configured': False, 'implementation': 'not built in',
                         'rate_hz': 'not configured'},
            'fusion': {'enabled': True, 'subscription_configured': True,
                       'consumer': 'RobotStateEstimator.update_imu'},
            'external_provider': 'required for observations',
            'transport': {'topic': p['imu_topic'], 'message_type': 'sensor_msgs/msg/Imu',
                          'frames': {'source': p['imu_frame']}},
            'measurement': {'fields': ['angular_velocity.z'],
                            'assumed_variance': [p['imu_yaw_rate_variance']],
                            'covariance_source': ('configured scalar measurement R; '
                                                  'message covariance ignored')},
            'timing': robot_time,
            'consumers': ['robot estimation when compatible messages arrive'],
            'configuration': configuration(None, (
                'imu_topic', 'imu_frame', 'imu_yaw_rate_variance', *robot_timing_parameters)),
        },
    }

    # Imperfect pose generation and pose acceptance have different owners.
    sources['external_planar_localization'] = {
        'name': 'External planar localization', 'classification': 'optional robot observation',
        'producer': {'configured': pose_selected,
                     'implementation': 'robot_simulation.nodes.localization_sensor',
                     'rate_hz': localization['rate_hz'] if localization else 'not configured'},
        'fusion': {'enabled': pose_fusion, 'subscription_configured': pose_fusion,
                   'consumer': ('RobotStateEstimator.update_planar_pose; '
                                'PoseAcceptance trust policy'),
                   'acceptance': {key: value for key, value in p.items()
                                  if key.startswith('localization_') and key not in {
                                      'localization_enabled', 'localization_topic',
                                      'localization_frame'}}},
        'external_provider': ('required for observations' if pose_fusion and not pose_selected
                              else 'possible compatible replacement'),
        'transport': {'topic': (localization['topic'] if localization
                                else p.get('localization_topic', 'not configured')),
                      'estimator_topic': p.get('localization_topic', 'not configured'),
                      'message_type': 'nav_msgs/msg/Odometry',
                      'frames': {'source': (localization['source_frame'] if localization
                                            else p.get('localization_frame', 'not configured')),
                                 'child': (localization['child_frame'] if localization
                                           else p['base_frame']),
                                 'fusion': p['robot_state_frame_id']}},
        'measurement': {'fields': ['pose.x', 'pose.y', 'pose.yaw'],
                        'covariance_source': 'message pose covariance: x, y, yaw',
                        'transform': ('acquisition-time TF; full selected covariance '
                                      'transformed into fusion frame')},
        'simulated_effects': ({key: localization[key] for key in (
            'white_noise_std', 'calibration_bias_std', 'dropout_probability', 'outages',
            'outlier_probability', 'outlier_offset', 'latency', 'latency_jitter', 'random_seed')}
            if localization else 'not configured'),
        'timing': {**robot_time,
                   'localization_max_age': p.get('localization_max_age', 'not configured'),
                   'generator_delivery': ({key: localization[key] for key in (
                       'max_delivery_age', 'future_tolerance', 'timer_period')}
                       if localization else 'not configured'),
                   'stamp': 'acquisition time retained through delayed delivery'},
        'consumers': (['robot estimation and full-state localization trust gate']
                      if pose_fusion else []),
        'configuration': configuration('localization', (
            *robot_timing_parameters, *(key for key in p if key.startswith('localization_')))),
    }
    sources['target_position'] = {
        'name': 'Synthetic target position', 'classification': 'target observation',
        'producer': {'configured': target['enabled'],
                     'implementation': 'SimulationNode / SyntheticMeasurementGenerator',
                     'rate_hz': target['rate_hz']},
        'fusion': {'enabled': True, 'subscription_configured': True,
                   'consumer': 'TargetStateEstimator.update'},
        'external_provider': 'not selected; built-in measurement required by this deployment',
        'transport': {'topic': p['target_measurement_topic'],
                      'message_type': 'geometry_msgs/msg/PointStamped',
                      'frames': {'source': tracking['reference_frame']}},
        'measurement': {'fields': target['observation']['components'],
                        'generated_covariance': target['noise']['covariance'],
                        'assumed_variance': p['target_position_variance'],
                        'covariance_source': ('configured estimator diagonal R; '
                                              'PointStamped carries no covariance')},
        'simulated_effects': {'noise': target['noise'], 'random_seed': target['random_seed']},
        'timing': {'stamp': ('committed target integration time, published after '
                             'Gazebo pose acknowledgement'),
                   'publication_rate': ('configured maximum schedule; '
                                        'acknowledgement/context dependent'),
                   'admission_max_age': 'not configured',
                   'future_tolerance': p['future_tolerance'],
                   'ordering': 'separate target filter chronology; no robot reorder buffer'},
        'consumers': ['target estimation; estimated target then prediction and control'],
        'configuration': configuration('target_measurements', (
            'target_measurement_topic', 'target_position_variance', 'target_spatial_dimension',
            'future_tolerance')),
    }

    # Truth/context and TF are not additional independent EKF observations.
    sources['physical_base_context'] = {
        'name': 'Physical base truth/context',
        'classification': 'physical truth and simulation context',
        'producer': {'configured': truth_selected,
                     'implementation': 'Gazebo OdometryPublisher via ros_gz_bridge',
                     'rate_hz': truth.get('rate_hz', 'not configured')},
        'fusion': {'enabled': False, 'subscription_configured': False},
        'transport': {'topic': truth.get('topic', 'not configured'),
                      'message_type': 'nav_msgs/msg/Odometry',
                      'frames': {'source': truth.get('frame_id', 'not configured'),
                                 'child': truth.get('child_frame_id', 'not configured')}},
        'measurement': {'covariance_source': 'not an estimator measurement',
                        'provenance': 'physical Gazebo world pose, not estimated robot state'},
        'timing': {'stamp': ('physical source time; contextual integration selects '
                             'only causal fresh samples'),
                   'physical_context': deployment.get('physical_context', 'not configured')},
        'consumers': ((['target contextual integration'] if context_required else [])
                      + (['imperfect localization generator'] if pose_selected else [])
                      + (['physical tracking FK'] if physical_tf else [])
                      + (['physical visualization'] if runtime['visualize_tracking'] else [])),
        'configuration': {**configuration('robot'),
                          'selection': ('runtime.publish_ground_truth overrides '
                                        'robot.ground_truth_odometry.enabled'),
                          'runtime_override': runtime.get(
                              'publish_ground_truth', 'not configured')},
    }
    sources['tracking_frame_tf'] = {
        'name': 'Tracking-frame TF', 'classification': 'derived tracking context',
        'producer': {'configured': tf_selected,
                     'implementation': 'robot_bringup.nodes.tracking_feedback_tf',
                     'rate_hz': tracking_tf['rate_hz'] if tf_selected else 'not applicable'},
        'fusion': {'enabled': False, 'subscription_configured': False},
        'transport': {'topic': 'not configured (TF2 default transport)',
                      'message_type': 'tf2_msgs/msg/TFMessage',
                      'frames': {'source': (truth.get('frame_id') if physical_tf
                                            else drive['frame_id']),
                                 'child': tracking['tracking_frame'],
                                 'reference': tracking['reference_frame']}},
        'measurement': {'pose_source': (tracking.get('pose_source', 'wheel_odometry')
                                        if tf_selected else 'static'),
                        'covariance_source': ('not an independent observation; '
                                              'FK from base pose and measured joints'),
                        'mount_resource': owners['description']},
        'timing': {'stamp': 'maximum paired feedback stamp with configured age/skew gates',
                   'broadcaster': dict(tracking_tf), 'consumer': dict(tracking)},
        'consumers': (['target tracking context'] if tf_selected
                      else ['static tracking position replaces TF']),
        'configuration': configuration('profile'),
    }
    sources['simulation_clock'] = {
        'name': 'Simulation clock', 'classification': 'timing infrastructure',
        'producer': {'configured': True, 'implementation': 'Gazebo clock via ros_gz_bridge',
                     'rate_hz': 'not configured'},
        'fusion': {'enabled': False, 'subscription_configured': False},
        'transport': {'topic': deployment['gazebo']['clock_topic'],
                      'message_type': 'rosgraph_msgs/msg/Clock',
                      'frames': 'not applicable'},
        'timing': {'stamp': ('simulation epoch; backwards clock jump clears '
                             'existing source/filter/context state')},
        'consumers': ['simulated ROS time, target stepping, freshness checks and epoch recovery'],
        'configuration': configuration('simulation'),
    }
    sources['target_ground_truth'] = {
        'name': 'Target committed truth', 'classification': 'evaluation truth',
        'producer': {'configured': True, 'implementation': 'SimulationNode._on_pose_result',
                     'rate_hz': 'not configured'},
        'fusion': {'enabled': False, 'subscription_configured': False},
        'transport': {'topic': integration['simulation']['ground_truth']['topic'],
                      'message_type': 'geometry_msgs/msg/PointStamped',
                      'frames': {'source': tracking['reference_frame']}},
        'measurement': {'provenance': ('committed numerical target state '
                                       'after Gazebo acknowledgement')},
        'timing': {'stamp': ('committed target integration time; '
                             'not an independent physical sensor')},
        'consumers': ['evaluation/visualization when selected; '
                      'synthetic measurement generator input'],
        'configuration': configuration('integration'),
    }
    for source in sources.values():
        source['live_status'] = 'not observed'
        source.setdefault('external_provider', 'not applicable')
        source.setdefault('measurement', {'covariance_source': 'not applicable'})
        source.setdefault('simulated_effects', 'not modeled by built-in producer')
        source.setdefault('safety_feedback', {'required_for_active_actuation': False})
    return deepcopy({
        'schema_version': 1, 'basis': 'resolved configuration, not live availability or health',
        'provenance_scope': ('selected resources and final override; '
                             'inherited per-key declaring ancestors not tracked'),
        'configuration_resources': owners, 'sources': sources,
    })


def format_sensor_inventory(inventory: Mapping[str, Any]) -> str:
    """Keep launch output concise; the resolved plan retains the complete inventory."""
    sources = inventory['sources']
    selections = []
    for key in ('wheel_odometry', 'joint_feedback', 'imu',
                'external_planar_localization', 'target_position'):
        source = sources[key]
        producer = 'selected' if source['producer']['configured'] else 'external/absent'
        fusion = 'enabled' if source['fusion']['enabled'] else 'disabled'
        selections.append(f'{key}: producer={producer}, fusion={fusion}')
    wheel = sources['wheel_odometry']['measurement']['assumed_variance']
    pose = sources['external_planar_localization']
    detail = f'Wheel measurement variance [v, omega]={wheel}'
    if isinstance(pose['simulated_effects'], dict):
        effects = pose['simulated_effects']
        detail += (f'; pose rate={pose["producer"]["rate_hz"]} Hz, '
                   f'dropout={effects["dropout_probability"]}, '
                   f'latency={effects["latency"]}+jitter[0,{effects["latency_jitter"]}] s')
    base = sources['physical_base_context']['producer']['configured']
    tracking = sources['tracking_frame_tf']['measurement']['pose_source']
    context = (f'Context (not fused): physical base truth selected={base}; '
               f'tracking pose source={tracking}')
    return '\n'.join(['Effective sensors (configured, not live): ' + '; '.join(selections),
                      detail, context])
