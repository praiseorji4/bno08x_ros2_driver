#include "bno08x_driver/bno08x_ros.hpp"
#include "bno08x_driver/i2c_interface.hpp"
#include "bno08x_driver/uart_interface.hpp"
#include "bno08x_driver/spi_interface.hpp"

constexpr uint8_t ROTATION_VECTOR_RECEIVED = 0x01;
constexpr uint8_t ACCELEROMETER_RECEIVED   = 0x02;
constexpr uint8_t GYROSCOPE_RECEIVED       = 0x04;

BNO08xROS::BNO08xROS()
    : Node("bno08x_ros"), watchdog_(nullptr)
{  
    this->init_parameters();
    // Publishers exist before the sensor starts, since reports can arrive during initialization
    this->init_publishers();
    this->init_comms();
    this->init_sensor();

    // Poll the sensor faster than the fastest enabled report. Each poll reads at most one
    // SHTP transfer, so polling at exactly the report rate lets samples wait in the sensor
    // for up to a full period and can fall behind.
    int fastest_rate = 1;
    for (const auto &entry : report_rates_) {
        fastest_rate = std::max(fastest_rate, entry.second);
    }
    const int poll_rate = fastest_rate * poll_rate_multiplier_;
    RCLCPP_INFO(this->get_logger(), "Polling sensor at %d Hz", poll_rate);
    this->imu_received_flag_ = 0;
    this->poll_timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000000 / poll_rate), // Hz to us
        std::bind(&BNO08xROS::poll_timer_callback, this)
    );

    // Initialize the watchdog timer
    auto timeout = std::chrono::milliseconds(2000);
    watchdog_ = new Watchdog();
    watchdog_->set_timeout(timeout);
    watchdog_->set_check_interval(timeout / 2); 
    watchdog_->set_callback([this]() {
        RCLCPP_ERROR(this->get_logger(), "Watchdog timeout! No data received from sensor. Resetting...");
        this->reset();
    });
    watchdog_->start();

    RCLCPP_INFO(this->get_logger(), "BNO08X ROS Node started.");
}

BNO08xROS::~BNO08xROS() {
    delete watchdog_;
    delete bno08x_;
    delete comm_interface_;
}

/**
 * @brief Create the publishers for every enabled output
 */
void BNO08xROS::init_publishers() {
    if (publish_imu_) {
        this->imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        RCLCPP_INFO(this->get_logger(), "IMU Publisher created");
        RCLCPP_INFO(this->get_logger(), "IMU Rate: %d", imu_rate_);
    }

    if (publish_magnetic_field_) {
        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
                                                                        "/magnetic_field", 10);
        RCLCPP_INFO(this->get_logger(), "Magnetic Field Publisher created");
        RCLCPP_INFO(this->get_logger(), "Magnetic Field Rate: %d", magnetic_field_rate_);
    }

    // Characterization outputs use a deeper queue so bursts are not dropped while recording
    const auto qos = rclcpp::QoS(100);
    using namespace bno08x_driver::msg;
    if (publish_report_info_) {
        report_info_publisher_ = this->create_publisher<Report>("/bno08x/report_info", qos);
    }
    if (publish_raw_) {
        raw_accel_publisher_ = this->create_publisher<RawSensor>("/bno08x/raw/accelerometer", qos);
        raw_gyro_publisher_ = this->create_publisher<RawSensor>("/bno08x/raw/gyroscope", qos);
        raw_mag_publisher_ = this->create_publisher<RawSensor>("/bno08x/raw/magnetometer", qos);
    }
    if (publish_gyro_uncal_) {
        gyro_uncal_publisher_ = this->create_publisher<GyroUncalibrated>(
                                                        "/bno08x/gyroscope_uncalibrated", qos);
    }
    if (publish_mag_uncal_) {
        mag_uncal_publisher_ = this->create_publisher<MagneticFieldUncalibrated>(
                                                        "/bno08x/magnetic_field_uncalibrated", qos);
    }
    if (publish_rotation_vector_) {
        rotation_vector_publisher_ = this->create_publisher<Orientation>(
                                                        "/bno08x/rotation_vector", qos);
    }
    if (publish_game_rotation_vector_) {
        game_rotation_vector_publisher_ = this->create_publisher<Orientation>(
                                                        "/bno08x/game_rotation_vector", qos);
    }
    if (publish_gyro_integrated_rv_) {
        gyro_integrated_rv_publisher_ = this->create_publisher<Orientation>(
                                                        "/bno08x/gyro_integrated_rotation_vector", qos);
    }
    if (publish_stability_) {
        stability_publisher_ = this->create_publisher<StabilityClassification>(
                                                        "/bno08x/stability", qos);
    }
}

/**
 * @brief Initialize the communication interface
 * 
 * communication interface based on the parameters
 */
void BNO08xROS::init_comms() {
    bool i2c_enabled, uart_enabled, spi_enabled;
    this->get_parameter("i2c.enabled", i2c_enabled);
    this->get_parameter("uart.enabled", uart_enabled);
    this->get_parameter("spi.enabled", spi_enabled);

    if (i2c_enabled) {
        std::string device;
        std::string address;
        this->get_parameter("i2c.bus", device);
        this->get_parameter("i2c.address", address);
        RCLCPP_INFO(this->get_logger(), "Communication Interface: I2C");
        try {
            comm_interface_ = new I2CInterface(device, std::stoi(address, nullptr, 16));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "Failed to create I2CInterface: %s", e.what());
            throw std::runtime_error("I2CInterface creation failed");
        }
    } else if (uart_enabled) {
        RCLCPP_INFO(this->get_logger(), "Communication Interface: UART");
        std::string device;
        this->get_parameter("uart.device", device);
        try{
            comm_interface_ = new UARTInterface(device);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "UART Interface not implemented: %s", e.what());
            throw std::runtime_error("UARTInterface creation failed");
        }
    } else if (spi_enabled){
        RCLCPP_INFO(this->get_logger(), "Communication Interface: SPI");
        std::string device;
        this->get_parameter("spi.device", device);
        try {
            comm_interface_ = new SPIInterface(device);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "SPI Interface not implemented: %s", e.what());
            throw std::runtime_error("SPIInterface creation failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "No communication interface enabled!");
        throw std::runtime_error("Communication interface setup failed");
    }
}

/**
 * @brief Initialize the parameters
 * 
 * This function initializes the parameters for the node
 * 
 */
void BNO08xROS::init_parameters() {
    this->declare_parameter<std::string>("frame_id", "bno085");

    this->declare_parameter<bool>("publish.magnetic_field.enabled", true);
    this->declare_parameter<int>("publish.magnetic_field.rate", 100);
    this->declare_parameter<bool>("publish.magnetic_field.tesla", false);
    this->declare_parameter<bool>("publish.imu.enabled", true);
    this->declare_parameter<int>("publish.imu.rate", 100);
    this->declare_parameter<std::string>("publish.imu.orientation_source", "rotation_vector");

    this->declare_parameter<bool>("i2c.enabled", true);
    this->declare_parameter<std::string>("i2c.bus", "/dev/i2c-7");
    this->declare_parameter<std::string>("i2c.address", "0x4A");
    this->declare_parameter<bool>("uart.enabled", false);
    this->declare_parameter<std::string>("uart.device", "/dev/ttyACM0");
    this->declare_parameter<bool>("spi.enabled", false);
    this->declare_parameter<std::string>("spi.device", "/dev/spidev0.0");

    this->declare_parameter<bool>("timestamp.use_sample_time", false);
    this->declare_parameter<int>("poll.rate_multiplier", 1);

    const std::vector<double> zeros(9, 0.0);
    this->declare_parameter<std::vector<double>>("covariance.orientation", zeros);
    this->declare_parameter<std::vector<double>>("covariance.angular_velocity", zeros);
    this->declare_parameter<std::vector<double>>("covariance.linear_acceleration", zeros);
    this->declare_parameter<std::vector<double>>("covariance.magnetic_field", zeros);

    this->declare_parameter<bool>("publish.report_info.enabled", false);
    this->declare_parameter<bool>("publish.raw.enabled", false);
    this->declare_parameter<int>("publish.raw.rate", 100);
    this->declare_parameter<bool>("publish.gyroscope_uncalibrated.enabled", false);
    this->declare_parameter<int>("publish.gyroscope_uncalibrated.rate", 100);
    this->declare_parameter<bool>("publish.magnetic_field_uncalibrated.enabled", false);
    this->declare_parameter<int>("publish.magnetic_field_uncalibrated.rate", 100);
    this->declare_parameter<bool>("publish.rotation_vector.enabled", false);
    this->declare_parameter<int>("publish.rotation_vector.rate", 100);
    this->declare_parameter<bool>("publish.game_rotation_vector.enabled", false);
    this->declare_parameter<int>("publish.game_rotation_vector.rate", 100);
    this->declare_parameter<bool>("publish.gyro_integrated_rotation_vector.enabled", false);
    this->declare_parameter<int>("publish.gyro_integrated_rotation_vector.rate", 100);
    this->declare_parameter<bool>("publish.stability.enabled", false);
    this->declare_parameter<int>("publish.stability.rate", 10);

    this->get_parameter("frame_id", frame_id_);

    this->get_parameter("publish.magnetic_field.enabled", publish_magnetic_field_);
    this->get_parameter("publish.magnetic_field.rate", magnetic_field_rate_);
    this->get_parameter("publish.magnetic_field.tesla", magnetic_field_tesla_);
    this->get_parameter("publish.imu.enabled", publish_imu_);
    this->get_parameter("publish.imu.rate", imu_rate_);

    std::string orientation_source;
    this->get_parameter("publish.imu.orientation_source", orientation_source);
    if (orientation_source == "rotation_vector") {
        imu_orientation_sensor_ = SH2_ROTATION_VECTOR;
    } else if (orientation_source == "game_rotation_vector") {
        imu_orientation_sensor_ = SH2_GAME_ROTATION_VECTOR;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown publish.imu.orientation_source '%s', "
                     "expected 'rotation_vector' or 'game_rotation_vector'", orientation_source.c_str());
        throw std::runtime_error("Invalid orientation source");
    }

    this->get_parameter("timestamp.use_sample_time", use_sample_time_);
    this->get_parameter("poll.rate_multiplier", poll_rate_multiplier_);
    if (poll_rate_multiplier_ < 1) {
        RCLCPP_WARN(this->get_logger(), "poll.rate_multiplier must be >= 1, using 1");
        poll_rate_multiplier_ = 1;
    }

    const std::vector<std::pair<std::string, std::array<double, 9>*>> covariances = {
        {"covariance.orientation", &orientation_covariance_},
        {"covariance.angular_velocity", &angular_velocity_covariance_},
        {"covariance.linear_acceleration", &linear_acceleration_covariance_},
        {"covariance.magnetic_field", &magnetic_field_covariance_},
    };
    for (const auto &cov : covariances) {
        std::vector<double> values;
        this->get_parameter(cov.first, values);
        if (values.size() != 9) {
            RCLCPP_ERROR(this->get_logger(), "%s must have 9 elements (row-major 3x3), got %zu",
                         cov.first.c_str(), values.size());
            throw std::runtime_error("Invalid covariance parameter");
        }
        std::copy(values.begin(), values.end(), cov.second->begin());
    }

    this->get_parameter("publish.report_info.enabled", publish_report_info_);
    this->get_parameter("publish.raw.enabled", publish_raw_);
    this->get_parameter("publish.raw.rate", raw_rate_);
    this->get_parameter("publish.gyroscope_uncalibrated.enabled", publish_gyro_uncal_);
    this->get_parameter("publish.gyroscope_uncalibrated.rate", gyro_uncal_rate_);
    this->get_parameter("publish.magnetic_field_uncalibrated.enabled", publish_mag_uncal_);
    this->get_parameter("publish.magnetic_field_uncalibrated.rate", mag_uncal_rate_);
    this->get_parameter("publish.rotation_vector.enabled", publish_rotation_vector_);
    this->get_parameter("publish.rotation_vector.rate", rotation_vector_rate_);
    this->get_parameter("publish.game_rotation_vector.enabled", publish_game_rotation_vector_);
    this->get_parameter("publish.game_rotation_vector.rate", game_rotation_vector_rate_);
    this->get_parameter("publish.gyro_integrated_rotation_vector.enabled", publish_gyro_integrated_rv_);
    this->get_parameter("publish.gyro_integrated_rotation_vector.rate", gyro_integrated_rv_rate_);
    this->get_parameter("publish.stability.enabled", publish_stability_);
    this->get_parameter("publish.stability.rate", stability_rate_);

    // Magnetometer reports are limited to 100 Hz by the sensor
    raw_rate_ = std::max(raw_rate_, 1);
    const std::vector<int*> rates = {
        &magnetic_field_rate_, &imu_rate_, &gyro_uncal_rate_, &mag_uncal_rate_,
        &rotation_vector_rate_, &game_rotation_vector_rate_, &gyro_integrated_rv_rate_,
        &stability_rate_};
    for (int *rate : rates) {
        if (*rate < 1) {
            RCLCPP_WARN(this->get_logger(), "Report rates must be >= 1 Hz, using 1 Hz");
            *rate = 1;
        }
    }
}

/**
 * @brief Initialize the sensor
 * 
 * This function initializes the sensor and enables the required sensor reports
 * 
 */
void BNO08xROS::init_sensor() {

    try {
        bno08x_ = new BNO08x(comm_interface_, std::bind(&BNO08xROS::sensor_callback, this, 
                                        std::placeholders::_1, std::placeholders::_2), this);
    } catch (const std::bad_alloc& e) {
        RCLCPP_ERROR(this->get_logger(), 
                        "Failed to allocate memory for BNO08x object: %s", e.what());
        throw std::runtime_error("BNO08x object allocation failed");
    }

    if (!bno08x_->begin()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO08X sensor");
        throw std::runtime_error("BNO08x initialization failed");
    }

    report_rates_.clear();
    if (publish_magnetic_field_) {
        request_report(SH2_MAGNETIC_FIELD_CALIBRATED, magnetic_field_rate_);
    }
    if (publish_imu_) {
        request_report(imu_orientation_sensor_, imu_rate_);
        request_report(SH2_ACCELEROMETER, imu_rate_);
        request_report(SH2_GYROSCOPE_CALIBRATED, imu_rate_);
    }
    if (publish_raw_) {
        request_report(SH2_RAW_ACCELEROMETER, raw_rate_);
        request_report(SH2_RAW_GYROSCOPE, raw_rate_);
        request_report(SH2_RAW_MAGNETOMETER, std::min(raw_rate_, 100));
    }
    if (publish_gyro_uncal_) {
        request_report(SH2_GYROSCOPE_UNCALIBRATED, gyro_uncal_rate_);
    }
    if (publish_mag_uncal_) {
        request_report(SH2_MAGNETIC_FIELD_UNCALIBRATED, std::min(mag_uncal_rate_, 100));
    }
    if (publish_rotation_vector_) {
        request_report(SH2_ROTATION_VECTOR, rotation_vector_rate_);
    }
    if (publish_game_rotation_vector_) {
        request_report(SH2_GAME_ROTATION_VECTOR, game_rotation_vector_rate_);
    }
    if (publish_gyro_integrated_rv_) {
        request_report(SH2_GYRO_INTEGRATED_RV, gyro_integrated_rv_rate_);
    }
    if (publish_stability_) {
        request_report(SH2_STABILITY_CLASSIFIER, stability_rate_);
    }

    if (report_rates_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No sensor reports enabled! Exiting...");
        throw std::runtime_error("No sensor reports enabled");
    }

    for (const auto &entry : report_rates_) {
        if (!this->bno08x_->enable_report(entry.first, 1000000 / entry.second)) {  // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable sensor report 0x%02x", entry.first);
        } else {
            RCLCPP_INFO(this->get_logger(), "Enabled sensor report 0x%02x at %d Hz",
                        entry.first, entry.second);
        }
    }
}

/**
 * @brief Record that a report is needed at the given rate
 *
 * A report requested by several outputs is enabled once, at the highest requested rate.
 */
void BNO08xROS::request_report(sh2_SensorId_t sensor_id, int rate) {
    int &current = report_rates_[sensor_id];
    current = std::max(current, rate);
}

/**
 * @brief Compute the header stamp for a sensor sample and fill its report metadata
 *
 * With timestamp.use_sample_time, the stamp is the sample time estimated by the sh2 library
 * (host read time minus the delay reported by the sensor), converted to ROS time. Otherwise
 * it is the time the driver processed the report, as before.
 */
rclcpp::Time BNO08xROS::sample_stamp(const sh2_SensorValue_t *sensor_value,
                                     bno08x_driver::msg::ReportInfo &info) {
    const rclcpp::Time now = this->get_clock()->now();

    const uint32_t age_us = BNO08x::sample_age_us(BNO08x::host_time_us(), sensor_value->timestamp);

    info.sensor_id = sensor_value->sensorId;
    info.sequence = sensor_value->sequence;
    info.accuracy = sensor_value->status & 0x03;
    info.delay_us = sensor_value->delay;
    info.sample_time_us = sensor_value->timestamp;
    info.receive_time_us = sensor_value->timestamp + age_us;

    if (!use_sample_time_) {
        return now;
    }
    // A sample older than a second means the clocks disagree (e.g. the wall clock jumped),
    // so fall back to the receive time rather than publish a wrong stamp.
    if (age_us > 1000000) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Implausible sample age of %u us, using receive time", age_us);
        return now;
    }
    return now - rclcpp::Duration(std::chrono::microseconds(age_us));
}

/**
 * @brief Callback function for sensor events
 * 
 * @param cookie Pointer to the object that called the function, not used here
 * @param sensor_value The sensor value from parsing the sensor event buffer
 * 
 */
void BNO08xROS::sensor_callback(void *cookie, sh2_SensorValue_t *sensor_value) {
	DEBUG_LOG("Sensor Callback");
    if (watchdog_) {
        watchdog_->reset();
    }

    using namespace bno08x_driver::msg;
    ReportInfo info;
    const rclcpp::Time stamp = sample_stamp(sensor_value, info);

    if (report_info_publisher_) {
        Report report;
        report.header.frame_id = frame_id_;
        report.header.stamp = stamp;
        report.info = info;
        report_info_publisher_->publish(report);
    }

    const double mag_scale = magnetic_field_tesla_ ? 1e-6 : 1.0;  // sensor reports uT

	switch(sensor_value->sensorId){
		case SH2_MAGNETIC_FIELD_CALIBRATED:
			this->mag_msg_.magnetic_field.x = sensor_value->un.magneticField.x * mag_scale;
			this->mag_msg_.magnetic_field.y = sensor_value->un.magneticField.y * mag_scale;
			this->mag_msg_.magnetic_field.z = sensor_value->un.magneticField.z * mag_scale;
			this->mag_msg_.magnetic_field_covariance = magnetic_field_covariance_;
			this->mag_msg_.header.frame_id = this->frame_id_;
			this->mag_msg_.header.stamp = stamp;
			// IMU will still return infrequent magnetic field reports even if the report
			// was not enabled, so check it was enabled before publishing.
			if (mag_publisher_) {
				this->mag_publisher_->publish(this->mag_msg_);
			}
			break;
		case SH2_ROTATION_VECTOR:
		case SH2_GAME_ROTATION_VECTOR: {
			const bool is_rv = sensor_value->sensorId == SH2_ROTATION_VECTOR;
			const float i = is_rv ? sensor_value->un.rotationVector.i : sensor_value->un.gameRotationVector.i;
			const float j = is_rv ? sensor_value->un.rotationVector.j : sensor_value->un.gameRotationVector.j;
			const float k = is_rv ? sensor_value->un.rotationVector.k : sensor_value->un.gameRotationVector.k;
			const float real = is_rv ? sensor_value->un.rotationVector.real
			                         : sensor_value->un.gameRotationVector.real;
			if (publish_imu_ && sensor_value->sensorId == imu_orientation_sensor_) {
				this->imu_msg_.orientation.x = i;
				this->imu_msg_.orientation.y = j;
				this->imu_msg_.orientation.z = k;
				this->imu_msg_.orientation.w = real;
				imu_received_flag_ |= ROTATION_VECTOR_RECEIVED;
			}
			auto publisher = is_rv ? rotation_vector_publisher_ : game_rotation_vector_publisher_;
			if (publisher) {
				Orientation msg;
				msg.header.frame_id = frame_id_;
				msg.header.stamp = stamp;
				msg.info = info;
				msg.orientation.x = i;
				msg.orientation.y = j;
				msg.orientation.z = k;
				msg.orientation.w = real;
				msg.heading_accuracy = is_rv ? sensor_value->un.rotationVector.accuracy : -1.0;
				publisher->publish(msg);
			}
			break;
		}
		case SH2_ACCELEROMETER:
			this->imu_msg_.linear_acceleration.x = sensor_value->un.accelerometer.x;
			this->imu_msg_.linear_acceleration.y = sensor_value->un.accelerometer.y;
			this->imu_msg_.linear_acceleration.z = sensor_value->un.accelerometer.z;
			imu_received_flag_ |= ACCELEROMETER_RECEIVED;
			break;
		case SH2_GYROSCOPE_CALIBRATED:
			this->imu_msg_.angular_velocity.x = sensor_value->un.gyroscope.x;
			this->imu_msg_.angular_velocity.y = sensor_value->un.gyroscope.y;
			this->imu_msg_.angular_velocity.z = sensor_value->un.gyroscope.z;
			imu_gyro_stamp_ = stamp;
			imu_received_flag_ |= GYROSCOPE_RECEIVED;
			break;
		case SH2_RAW_ACCELEROMETER:
		case SH2_RAW_GYROSCOPE:
		case SH2_RAW_MAGNETOMETER: {
			RawSensor msg;
			msg.header.frame_id = frame_id_;
			msg.header.stamp = stamp;
			msg.info = info;
			rclcpp::Publisher<RawSensor>::SharedPtr publisher;
			if (sensor_value->sensorId == SH2_RAW_ACCELEROMETER) {
				const auto &raw = sensor_value->un.rawAccelerometer;
				msg.x = raw.x; msg.y = raw.y; msg.z = raw.z;
				msg.sensor_timestamp_us = raw.timestamp;
				publisher = raw_accel_publisher_;
			} else if (sensor_value->sensorId == SH2_RAW_GYROSCOPE) {
				const auto &raw = sensor_value->un.rawGyroscope;
				msg.x = raw.x; msg.y = raw.y; msg.z = raw.z;
				msg.temperature = raw.temperature;
				msg.sensor_timestamp_us = raw.timestamp;
				publisher = raw_gyro_publisher_;
			} else {
				const auto &raw = sensor_value->un.rawMagnetometer;
				msg.x = raw.x; msg.y = raw.y; msg.z = raw.z;
				msg.sensor_timestamp_us = raw.timestamp;
				publisher = raw_mag_publisher_;
			}
			if (publisher) {
				publisher->publish(msg);
			}
			break;
		}
		case SH2_GYROSCOPE_UNCALIBRATED:
			if (gyro_uncal_publisher_) {
				const auto &gyro = sensor_value->un.gyroscopeUncal;
				GyroUncalibrated msg;
				msg.header.frame_id = frame_id_;
				msg.header.stamp = stamp;
				msg.info = info;
				msg.angular_velocity.x = gyro.x;
				msg.angular_velocity.y = gyro.y;
				msg.angular_velocity.z = gyro.z;
				msg.bias.x = gyro.biasX;
				msg.bias.y = gyro.biasY;
				msg.bias.z = gyro.biasZ;
				gyro_uncal_publisher_->publish(msg);
			}
			break;
		case SH2_MAGNETIC_FIELD_UNCALIBRATED:
			if (mag_uncal_publisher_) {
				const auto &mag = sensor_value->un.magneticFieldUncal;
				MagneticFieldUncalibrated msg;
				msg.header.frame_id = frame_id_;
				msg.header.stamp = stamp;
				msg.info = info;
				msg.magnetic_field.x = mag.x * 1e-6;  // uT to T
				msg.magnetic_field.y = mag.y * 1e-6;
				msg.magnetic_field.z = mag.z * 1e-6;
				msg.hard_iron_bias.x = mag.biasX * 1e-6;
				msg.hard_iron_bias.y = mag.biasY * 1e-6;
				msg.hard_iron_bias.z = mag.biasZ * 1e-6;
				mag_uncal_publisher_->publish(msg);
			}
			break;
		case SH2_GYRO_INTEGRATED_RV:
			if (gyro_integrated_rv_publisher_) {
				const auto &girv = sensor_value->un.gyroIntegratedRV;
				Orientation msg;
				msg.header.frame_id = frame_id_;
				msg.header.stamp = stamp;
				msg.info = info;
				msg.orientation.x = girv.i;
				msg.orientation.y = girv.j;
				msg.orientation.z = girv.k;
				msg.orientation.w = girv.real;
				msg.heading_accuracy = -1.0;
				msg.angular_velocity.x = girv.angVelX;
				msg.angular_velocity.y = girv.angVelY;
				msg.angular_velocity.z = girv.angVelZ;
				gyro_integrated_rv_publisher_->publish(msg);
			}
			break;
		case SH2_STABILITY_CLASSIFIER:
			if (stability_publisher_) {
				StabilityClassification msg;
				msg.header.frame_id = frame_id_;
				msg.header.stamp = stamp;
				msg.info = info;
				msg.classification = sensor_value->un.stabilityClassifier.classification;
				stability_publisher_->publish(msg);
			}
			break;
		default:
			break;
	}

	if(imu_publisher_ &&
	   imu_received_flag_ == (ROTATION_VECTOR_RECEIVED | ACCELEROMETER_RECEIVED | GYROSCOPE_RECEIVED)){
		this->imu_msg_.header.frame_id = this->frame_id_;
		// The gyro sample time is used because angular velocity is what a filter integrates
		this->imu_msg_.header.stamp = use_sample_time_ ? imu_gyro_stamp_ : this->get_clock()->now();
		this->imu_msg_.orientation_covariance = orientation_covariance_;
		this->imu_msg_.angular_velocity_covariance = angular_velocity_covariance_;
		this->imu_msg_.linear_acceleration_covariance = linear_acceleration_covariance_;
		this->imu_publisher_->publish(this->imu_msg_);
		imu_received_flag_ = 0;
	}

}

/**
 * @brief Poll the sensor for new events
 * 
 * This function is called periodically at the rate of the fastest sensor report
 * to get the buffered sensor events
 * called by the poll_timer_ timer
 */
void BNO08xROS::poll_timer_callback() {
    {
        std::lock_guard<std::mutex> lock(bno08x_mutex_);
        this->bno08x_->poll();
    }
}

void BNO08xROS::reset() {
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    delete bno08x_;
    this->init_sensor();
}
