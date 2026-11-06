#pragma once

#include <array>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autonom_hardware
{

/// Republishes odometry with configured covariances. rf2o_laser_odometry publishes all-zero covariance,
/// which robot_localization would otherwise treat as near-perfect certainty.
class OdometryCovarianceRelay : public rclcpp::Node
{
public:
  explicit OdometryCovarianceRelay(const rclcpp::NodeOptions & options);

private:
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace autonom_hardware
