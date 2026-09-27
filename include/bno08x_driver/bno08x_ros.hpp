#pragma once

#include <map>
#include <mutex>
#include <array>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "bno08x_driver/msg/report.hpp"
#include "bno08x_driver/msg/raw_sensor.hpp"
#include "bno08x_driver/msg/gyro_uncalibrated.hpp"
#include "bno08x_driver/msg/magnetic_field_uncalibrated.hpp"
#include "bno08x_driver/msg/orientation.hpp"
#include "bno08x_driver/msg/stability_classification.hpp"
#include "bno08x_driver/bno08x.hpp"
#include "bno08x_driver/logger.h"
#include "bno08x_driver/watchdog.hpp"
#include "sh2/sh2.h"

class BNO08xROS : public rclcpp::Node
{
public:
    BNO08xROS();
    ~BNO08xROS();
    void sensor_callback(void *cookie, sh2_SensorValue_t *sensor_value);

private:
    void init_comms();
    void init_parameters();
    void init_publishers();
    void init_sensor();
    void request_report(sh2_SensorId_t sensor_id, int rate);
    void poll_timer_callback();
    void reset();
    rclcpp::Time sample_stamp(const sh2_SensorValue_t *sensor_value,
                              bno08x_driver::msg::ReportInfo &info);

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::MagneticField mag_msg_;
    uint8_t imu_received_flag_;
    rclcpp::Time imu_gyro_stamp_;

    // Characterization publishers (all optional)
    rclcpp::Publisher<bno08x_driver::msg::Report>::SharedPtr report_info_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::RawSensor>::SharedPtr raw_accel_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::RawSensor>::SharedPtr raw_gyro_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::RawSensor>::SharedPtr raw_mag_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::GyroUncalibrated>::SharedPtr gyro_uncal_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::MagneticFieldUncalibrated>::SharedPtr mag_uncal_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::Orientation>::SharedPtr rotation_vector_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::Orientation>::SharedPtr game_rotation_vector_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::Orientation>::SharedPtr gyro_integrated_rv_publisher_;
    rclcpp::Publisher<bno08x_driver::msg::StabilityClassification>::SharedPtr stability_publisher_;

    // ROS Timer
    rclcpp::TimerBase::SharedPtr poll_timer_;

    // BNO08X Sensor Interface
    BNO08x* bno08x_;
    std::mutex bno08x_mutex_;
    CommInterface* comm_interface_;

    // Watchdog
    Watchdog* watchdog_;

    // Parameters
    std::string frame_id_;
    bool publish_magnetic_field_;
    int magnetic_field_rate_;
    bool magnetic_field_tesla_;
    bool publish_imu_;
    int imu_rate_;
    sh2_SensorId_t imu_orientation_sensor_;

    bool use_sample_time_;
    int poll_rate_multiplier_;

    std::array<double, 9> orientation_covariance_;
    std::array<double, 9> angular_velocity_covariance_;
    std::array<double, 9> linear_acceleration_covariance_;
    std::array<double, 9> magnetic_field_covariance_;

    bool publish_report_info_;
    bool publish_raw_;
    int raw_rate_;
    bool publish_gyro_uncal_;
    int gyro_uncal_rate_;
    bool publish_mag_uncal_;
    int mag_uncal_rate_;
    bool publish_rotation_vector_;
    int rotation_vector_rate_;
    bool publish_game_rotation_vector_;
    int game_rotation_vector_rate_;
    bool publish_gyro_integrated_rv_;
    int gyro_integrated_rv_rate_;
    bool publish_stability_;
    int stability_rate_;

    // Highest rate [Hz] requested for each SH-2 report, so a report shared by several
    // outputs is enabled once at the fastest rate any of them needs.
    std::map<sh2_SensorId_t, int> report_rates_;
};
