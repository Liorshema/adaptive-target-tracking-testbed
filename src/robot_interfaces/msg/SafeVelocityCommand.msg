# Body-frame base [v, omega], then named joint velocities (rad/s).
# Stamp is computation time; frame_id is the base body frame.
# A new session is required after clock reset; sequence increases per session.
std_msgs/Header header
string session_id
uint64 sequence
string[] joint_names
float64[] velocity
# false means stop the base and hold the last safe arm reference.
bool active
