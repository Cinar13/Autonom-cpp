#include "autonom_localization/odometry_covariance_relay.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

namespace autonom_localization
{
namespace
{

std::array<double, 36> declareCovarianceDiagonal(
  rclcpp::Node & node, const std::string & name, const std::array<double, 6> & default_diagonal)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Diagonal [x, y, z, roll, pitch, yaw] of the 6x6 covariance";
  descriptor.read_only = true;
  const auto diagonal = node.declare_parameter<std::vector<double>>(
    name, std::vector<double>(default_diagonal.begin(), default_diagonal.end()), descriptor);
  if (diagonal.size() != 6) {
    throw std::invalid_argument(name + " must contain exactly 6 values");
  }
  std::array<double, 36> covariance{};
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(diagonal[i]) || diagonal[i] < 0.0) {
      throw std::invalid_argument(name + " values must be finite and non-negative");
    }
    covariance[i * 7] = diagonal[i];
  }
  return covariance;
}

}  // namespace

OdometryCovarianceRelay::OdometryCovarianceRelay(const rclcpp::NodeOptions & options)
: rclcpp::Node("odometry_covariance_relay", options)
{
  pose_covariance_ = declareCovarianceDiagonal(*this, "pose_covariance_diagonal", {1e-2, 1e-2, 1e6, 1e6, 1e6, 2e-2});
  twist_covariance_ = declareCovarianceDiagonal(*this, "twist_covariance_diagonal", {5e-3, 5e-3, 1e6, 1e6, 1e6, 1e-2});

  publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom_out", rclcpp::SystemDefaultsQoS());
  subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom_in", rclcpp::SystemDefaultsQoS(), [this](nav_msgs::msg::Odometry::UniquePtr msg) {
      msg->pose.covariance = pose_covariance_;
      msg->twist.covariance = twist_covariance_;
      publisher_->publish(std::move(msg));
    });
}

}  // namespace autonom_localization

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_localization::OdometryCovarianceRelay)
