# All poses and target positions are in header.frame_id (estimate frame).
std_msgs/Header header
bool valid
string reason
# Input source times in seconds; -1 means absent, also populated during holds.
float64 robot_state_time
float64 prediction_reference_time
geometry_msgs/Point target_position
geometry_msgs/Pose desired_pose
geometry_msgs/Pose actual_pose
float64[] raw_velocity
float64[] safe_velocity
float64 computation_time
float64[] task_error
float64[] jacobian_singular_values
float64 weighted_motion_cost
uint32 active_bounds
float64 task_residual
