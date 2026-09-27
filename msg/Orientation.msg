# Orientation from one SH-2 fusion report: rotation vector (0x05), game rotation
# vector (0x08) or gyro-integrated rotation vector (0x2A). info.sensor_id says which.
std_msgs/Header header
ReportInfo info
geometry_msgs/Quaternion orientation

# Heading accuracy estimate [rad]. Only the rotation vector provides it; -1 otherwise.
float64 heading_accuracy

# Gyro-integrated rotation vector only: angular velocity [rad/s]. Zero otherwise.
geometry_msgs/Vector3 angular_velocity
