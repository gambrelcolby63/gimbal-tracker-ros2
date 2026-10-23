std_msgs/Header header
string model
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
# Row-major 6x6 covariance of [position, velocity].
float64[36] pose_velocity_covariance
float64 nis
