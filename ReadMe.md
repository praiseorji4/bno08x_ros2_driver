# BNO08x ROS2 Driver

ROS2 driver for the BNO08x sensor family based on the SH-2 protocol.

The BNO08x family (BNO085/BNO086) is a compact System in Package (SiP) with integrated accelerometer, gyroscope, magnetometer, and a 32-bit ARM® Cortex™-M0+ running CEVA's SH-2 firmware. It delivers real-time 3D orientation, heading, calibrated acceleration, and angular velocity, with on-board sensor fusion algorithms and calibration. It supports I2C, SPI, and UART interfaces for sensor data output.
For more information, refer to the [datasheet](./docs/BNO080_085-Datasheet.pdf).

[![CI](https://github.com/bnbhat/bno08x_ros2_driver/actions/workflows/ci.yaml/badge.svg)](https://github.com/bnbhat/bno08x_ros2_driver/actions/workflows/ci.yaml)

### Supported Features:
#### Communication Interfaces:
- I2C
- UART (Not implemented)
- SPI (Not implemented)

#### Data Rates:
- IMU data up to `400Hz`
- Magnetic field data up to `100Hz`

## Parameters
| Parameter |	Type	| Default	| Description |
|-----------|---------|-----------|-------------|
| frame_id      |string	|"bno085"	|Frame ID for sensor data messages.|
| publish.magnetic_field.enabled |bool	|true	|Enable publishing of magnetic field data.|
| publish.magnetic_field.rate	| int	|100	|Rate at which to publish magnetic field data (Hz).|
| publish.imu.enabled	|bool	|true	|Enable publishing of IMU data.|
| publish.imu.rate	|int	|100	|Rate at which to publish IMU data (Hz).|
| i2c.enabled	|bool	|true	|Enable I2C communication.|
| i2c.device	|string	|"/dev/i2c-7"	|I2C device path.|
| i2c.address	|string	|"0x4A"	|I2C address of the BNO08x sensor.|

## Characterization mode
For sensor characterization and data recording, the driver can publish every useful SH-2 report
with sample-time stamps, sequence numbers and accuracy. All of this is off by default, so the
standard `/imu` and `/magnetic_field` outputs are unchanged unless you enable it.

```bash
ros2 launch bno08x_driver bno085_i2c_characterization.launch.py
ros2 bag record -s mcap -a
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| timestamp.use_sample_time | bool | false | Stamp messages with the sample time estimated by the sh2 library (host read time minus the sensor-reported delay) instead of the time the driver processed them. |
| poll.rate_multiplier | int | 1 | Poll the sensor this many times faster than the fastest enabled report. |
| publish.imu.orientation_source | string | "rotation_vector" | `/imu` orientation from the 9-axis rotation vector or the 6-axis `game_rotation_vector` (no magnetometer). |
| publish.magnetic_field.tesla | bool | false | Publish `/magnetic_field` in tesla, as `sensor_msgs/MagneticField` specifies. The default keeps the previous microtesla values. |
| covariance.{orientation,angular_velocity,linear_acceleration,magnetic_field} | double[9] | zeros | Row-major covariances copied into `/imu` and `/magnetic_field`. |
| publish.report_info.enabled | bool | false | Metadata for every report on `/bno08x/report_info`. |
| publish.raw.enabled / rate | bool / int | false / 100 | Raw ADC counts on `/bno08x/raw/{accelerometer,gyroscope,magnetometer}` (magnetometer capped at 100 Hz). |
| publish.gyroscope_uncalibrated.enabled / rate | bool / int | false / 100 | `/bno08x/gyroscope_uncalibrated`, including the SH-2 bias estimate. |
| publish.magnetic_field_uncalibrated.enabled / rate | bool / int | false / 100 | `/bno08x/magnetic_field_uncalibrated`, including the hard-iron estimate (tesla). |
| publish.rotation_vector.enabled / rate | bool / int | false / 100 | `/bno08x/rotation_vector`, including the heading accuracy estimate. |
| publish.game_rotation_vector.enabled / rate | bool / int | false / 100 | `/bno08x/game_rotation_vector`. |
| publish.gyro_integrated_rotation_vector.enabled / rate | bool / int | false / 100 | `/bno08x/gyro_integrated_rotation_vector`, orientation plus angular velocity. |
| publish.stability.enabled / rate | bool / int | false / 10 | `/bno08x/stability` classifier (on table, stationary, stable, motion). |

Every characterization message carries a `ReportInfo` with the SH-2 report ID, the per-report
sequence number (gaps mean dropped reports), the accuracy status (0 to 3), the sensor-reported
delay, and the sample and receive times in the host clock. The message definitions are in `msg/`.

A report used by several outputs is enabled once at the highest requested rate, so an output can
receive samples faster than its own `rate`.

## Installation
Clone the repository:
```bash
cd ~/ros_ws/src
git clone https://github.com/bnbhat/bno08x-ros2-driver.git
```
Build the package:
```bash
cd ~/ros_ws
colcon build --packages-select bno08x_driver
```
Install any missing dependencies using rosdep:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Usage
Sensor is configured to I2C by default. To use other communication interfaces , 
you need to do a hardware change. Please refer to the [datasheet](./docs/BNO080_085-Datasheet.pdf) 
for more information.
Once the hardware change is done, you can configure the driver to use UART/SPI communication 
interface.

> [!CAUTION]
> If you want to use UART, make sure to select UART-SHTP mode in the sensor configuration.  
> UART-RVC is a simplified interface mode with minimal functionalities and is not supported 
> by this driver.


### I2C
To run the driver with I2C communication interface, you need to provide the I2C bus number and 
the I2C address of the BNO08x sensor.

example config file:
```yaml
bno08x_driver:
  ros__parameters:

    frame_id: "bno085"  # The frame_id to use for the sensor data

    # Communication Interface
    # Select the communication interface to use 
    # (only one interface can be enabled at a time)
    # This requires a hardware change to the board
    # (see documentation for more information)
    i2c:
      enabled: true
      device: "/dev/i2c-18"
      address: "0x4A"

    publish:
      all: false
      magnetic_field: 
        enabled: true
        rate: 100   # max 100 Hz
      imu:
        enabled: true
        rate: 100   # max 400 Hz
```

Launch the driver:
```bash
ros2 launch bno08x_driver bno085_i2c.launch.py
```

## Code Structure
 
At the core of this driver is a C++ class, `BNO08x`, which represents the BNO080/BNO085/BNO086 sensor and 
provides required methods to interact with the sensor using the [SH-2 protocol](./docs/SH-2-Reference-Manual.pdf).
 
The communication interface is abstracted using the `CommInterface` class, which can be implemented for any communication method (I2C, UART, SPI).

Currently, `I2C` communication interface is implemted and tested on Linux systems.

The Core `BNO08x` implementation can be used in any C++ project on any platform by implementing the `CommInterface` for the respective platform.

`BNO08xROS` is a wrapper class that uses the `BNO08x` class to provide ROS2 specific functionality.

The directory structure is as follows:
```plaintext
.
├── CMakeLists.txt
├── config
│   └── bno085_i2c.yaml
├── docs
│   ├── BNO080_085-Datasheet.pdf
│   └── SH-2-Reference-Manual.pdf
├── include
│   ├── bno08x_driver
│   │   ├── bno08x.hpp
│   │   ├── bno08x_ros.hpp
│   │   ├── comm_interface.hpp
│   │   ├── i2c_interface.hpp
│   │   ├── logger.h
│   │   ├── spi_interface.hpp
│   │   └── uart_interface.hpp
│   └── sh2
│       ├── CMakeLists.txt
│       ├── NOTICE.txt
│       ├── README.md
│       ├── sh2.c
│       ├── sh2_err.h
│       ├── sh2.h
│       ├── sh2_hal.h
│       ├── sh2_SensorValue.c
│       ├── sh2_SensorValue.h
│       ├── sh2_util.c
│       ├── sh2_util.h
│       ├── shtp.c
│       └── shtp.h
├── launch
│   └── bno085_i2c.launch.py
├── LICENSE
├── package.xml
├── ReadMe.md
└──  src
    ├── bno08x.cpp
    ├── bno08x_ros.cpp
    └── ros_node.cpp
```

## Acknowledgements
This driver uses the SH-2 protocol library provided by Hillcrest Labs.
It can be found in the `include/sh2` directory.
Visit the official repository here: [SH-2 Protocol Library](https://github.com/ceva-dsp/sh2.git)

## License
This project is licensed under the Apache License 2.0. You can find the full license text in the [LICENSE](./LICENSE) file of the repository.