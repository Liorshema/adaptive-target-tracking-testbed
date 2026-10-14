# All poses and target positions are in header.frame_id (estimate frame).
std_msgs/Header header
bool valid
string reason
geometry_msgs/Point target_position
geometry_msgs/Pose desired_pose
geometry_msgs/Pose actual_pose
float64[] raw_velocity
float64[] safe_velocity
