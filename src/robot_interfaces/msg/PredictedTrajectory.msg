
# Prediction reference timestamp and coordinate frame.
# header.stamp is the time of the state used to generate
# this prediction, not the time of the first predicted state.
std_msgs/Header header

# Trajectory dimensions.
uint32 horizon
uint32 state_dimension

# Absolute predicted state timestamps, in seconds.
# Shape: (horizon,)
float64[] timestamps

# Predicted states flattened in row-major order.
# Shape: (horizon, state_dimension)
# Expected length: horizon * state_dimension.
float64[] states

# Actual producer/model state, including explicitly labeled warm-up/fallback.
# Scientific model selection remains frozen in the experiment configuration.
string model_name
