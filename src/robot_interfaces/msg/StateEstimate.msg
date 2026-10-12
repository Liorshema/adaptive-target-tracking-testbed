
# Estimated state timestamp and coordinate frame.
std_msgs/Header header

# Estimated state vector, shape (n,).
float64[] mean

# Full covariance matrix P, shape (n, n),
# flattened in row-major order.
# Expected length: n * n.
float64[] covariance
