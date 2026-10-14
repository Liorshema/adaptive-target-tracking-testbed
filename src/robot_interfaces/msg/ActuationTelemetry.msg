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
