# Metadata carried by every SH-2 sensor report.

# SH-2 report ID (e.g. 0x02 calibrated gyroscope, 0x15 raw gyroscope).
uint8 sensor_id

# Per-report 8-bit sequence number. A gap between consecutive messages of the
# same sensor_id means reports were dropped.
uint8 sequence

# SH-2 accuracy status.
uint8 ACCURACY_UNRELIABLE = 0
uint8 ACCURACY_LOW = 1
uint8 ACCURACY_MEDIUM = 2
uint8 ACCURACY_HIGH = 3
uint8 accuracy

# Delay between the sample and the start of the report batch, reported by the sensor [us].
uint32 delay_us

# Sample time estimated by the sh2 library, in the host clock domain used by the driver [us].
uint64 sample_time_us

# Host clock when the driver processed the report [us]. receive_time_us - sample_time_us is
# the sample's latency inside the sensor, the bus and the driver's poll loop.
uint64 receive_time_us
