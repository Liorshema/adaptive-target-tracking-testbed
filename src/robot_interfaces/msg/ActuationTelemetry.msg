# Diagnostic observation only; never an actuator command.
std_msgs/Header header
bool enabled
bool active
string reason
float64[] commanded_velocity
float64[] position_reference
float64[] measured_positions
uint64 accepted_commands
uint64 rejected_commands
uint64 timeout_events
uint64 reference_limit_events
uint64 inactive_commands
uint64 feedback_rejections

# Adapter snapshot at header.stamp, not a physics-step observation or reference receipt.
# Arm arrays below and position_reference/measured_positions follow joint_names.
string[] joint_names
# feedback_valid includes ROS and wall freshness; stale values retain their source stamp.
bool feedback_valid
builtin_interfaces/Time feedback_stamp
float64[] measured_velocities
# q_reference - q_measured; empty unless both position arrays are available.
float64[] position_errors
# Last accepted active command in this clock epoch; validity is historical presence.
# active above states whether that command currently authorizes normal integration.
# Inactive heartbeats, rejected commands and holds do not replace this identity.
bool accepted_command_valid
string accepted_session_id
uint64 accepted_sequence
builtin_interfaces/Time accepted_stamp
