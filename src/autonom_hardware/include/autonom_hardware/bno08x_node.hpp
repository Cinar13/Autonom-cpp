#pragma once

#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "autonom_hardware/bno08x.hpp"
#include "autonom_hardware/i2c_device.hpp"

namespace autonom_hardware
{

/// BNO08x IMU on I2C, published as sensor_msgs/Imu on imu/data (frame imu_link) with configured covariances.
/// Reports: calibrated gyroscope (the EKF's yaw rate), accelerometer with gravity (the EKF removes it using the
/// orientation) and the game rotation vector (no magnetometer, so the motors cannot disturb it). One message per
/// gyroscope report, carrying the latest accelerometer and orientation. The sensor is (re)initialised at start-up,
/// after it resets itself, and whenever gyroscope reports stop for `report_timeout`.
class Bno08xNode : public rclcpp::Node
{
public:
  explicit Bno08xNode(const rclcpp::NodeOptions & options);

  /// For tests: drives `transport` instead of opening the i2c_bus parameter.
  Bno08xNode(const rclcpp::NodeOptions & options, std::unique_ptr<I2cTransport> transport);

private:
  void declareParameters();
  void start();
  bool initialize();
  void poll();
  void handleEvent(const bno08x::SensorEvent & event);

  std::unique_ptr<I2cTransport> transport_;
  std::unique_ptr<Bno08x> sensor_;

  std::string frame_id_;
  std::uint32_t report_interval_us_{10000};
  rclcpp::Duration report_timeout_{0, 0};
  rclcpp::Duration retry_period_{0, 0};
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};

  bool initialized_{false};
  bool have_acceleration_{false};
  bool have_orientation_{false};
  sensor_msgs::msg::Imu latest_;
  rclcpp::Time last_gyro_time_;
  rclcpp::Time last_init_attempt_;
  rclcpp::Time reset_grace_until_{0, 0, RCL_STEADY_TIME};
  std::uint64_t read_errors_{0};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace autonom_hardware
