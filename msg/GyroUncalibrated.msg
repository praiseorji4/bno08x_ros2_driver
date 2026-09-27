# Uncalibrated gyroscope (SH-2 0x07): angular velocity without bias correction,
# plus the bias the SH-2 firmware is currently estimating.
std_msgs/Header header
ReportInfo info
geometry_msgs/Vector3 angular_velocity  # [rad/s]
geometry_msgs/Vector3 bias              # [rad/s]
