#pragma once

#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace autonom_localization
{

/// Republishes IMU messages with the covariances the real BNO055 driver reports. Gazebo's IMU sensor reports its
/// noise stddev squared (accelerometer 3e-4 against the driver's 0.017), so the EKF trusted simulated acceleration
/// spikes ~60x more than it will on the robot: a hard wall hit at 1.25 m/s was integrated into a runaway pose.
/// Reads the driver's own parameter names (variance_orientation / variance_angular_vel / variance_acc), so the
/// simulation loads autonom_bringup/config/imu.yaml unchanged.
class ImuCovarianceRelay : public rclcpp::Node
{
public:
  explicit ImuCovarianceRelay(const rclcpp::NodeOptions & options);

private:
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

}  // namespace autonom_localization
