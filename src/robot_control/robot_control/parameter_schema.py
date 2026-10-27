"""Owner-controlled primitive parameter types shared by ROS declaration and preflight."""

REQUIRED_PARAMETERS = {'joint_count': 'INTEGER',
 'robot_state_topic': 'STRING',
 'prediction_topic': 'STRING',
 'max_robot_state_age': 'DOUBLE',
 'max_prediction_age': 'DOUBLE',
 'future_tolerance': 'DOUBLE',
 'validation_rate': 'DOUBLE'}

CONTROL_PARAMETERS = {'base_model_type': 'STRING',
 'manipulator_model_type': 'STRING',
 'kinematics_adapter_type': 'STRING',
 'controller_type': 'STRING',
 'controller_damping': 'DOUBLE',
 'controller_task_axes': 'INTEGER_ARRAY',
 'controller_task_gains': 'DOUBLE_ARRAY',
 'reference_geometry_type': 'STRING',
 'reference_policy_type': 'STRING',
 'tracking_radius': 'DOUBLE',
 'tracking_lookahead_time': 'DOUBLE',
 'tracking_forward_axis': 'DOUBLE_ARRAY',
 'tracking_up_axis': 'DOUBLE_ARRAY',
 'world_up_axis': 'DOUBLE_ARRAY',
 'base_velocity_lower': 'DOUBLE_ARRAY',
 'base_velocity_upper': 'DOUBLE_ARRAY'}

PARAMETERS = {
    **REQUIRED_PARAMETERS,
    **CONTROL_PARAMETERS,
    'compute_enabled': 'BOOL',
    'publish_commands': 'BOOL',
    'safety_enabled': 'BOOL',
    'command_topic': 'STRING',
    'command_frame_id': 'STRING',
    'command_joint_names': 'STRING_ARRAY',
    'publish_telemetry': 'BOOL',
    'telemetry_topic': 'STRING',
    'strategy_config_file': 'STRING',
    'strategy_configuration': 'STRING',
    'robot_description_file': 'STRING',
    'feedforward_override': 'STRING',
    'lookahead_override': 'STRING',
}
