# Raw accelerometer, gyroscope or magnetometer report (SH-2 0x14, 0x15, 0x16).
std_msgs/Header header
ReportInfo info

# Raw ADC counts, before any calibration.
int16 x
int16 y
int16 z

# Raw gyroscope only: die temperature in ADC counts. Zero for other sensors.
int16 temperature

# Timestamp from the sensor's own clock [us]. Useful for measuring clock drift
# between the sensor and the host.
uint32 sensor_timestamp_us
