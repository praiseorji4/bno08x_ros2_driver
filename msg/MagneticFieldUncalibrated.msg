# Uncalibrated magnetic field (SH-2 0x0F): field without hard-iron correction,
# plus the hard-iron offset the SH-2 firmware is currently estimating.
std_msgs/Header header
ReportInfo info
geometry_msgs/Vector3 magnetic_field  # [T]
geometry_msgs/Vector3 hard_iron_bias  # [T]
