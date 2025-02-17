#PoseTrajectory.msg
std_msgs/Header header
string[] joint_names
geometry_msgs/Twist[] poses
geometry_msgs/Twist[] velocities
geometry_msgs/Twist[] accelerations
builtin_interfaces/Duration time_from_start