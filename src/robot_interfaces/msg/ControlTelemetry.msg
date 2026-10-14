# All poses and target positions are in header.frame_id (estimate frame).
std_msgs/Header header
bool valid
string reason
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
