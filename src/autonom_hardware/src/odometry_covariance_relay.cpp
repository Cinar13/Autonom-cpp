#include "autonom_hardware/odometry_covariance_relay.hpp"

#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

#include "autonom_hardware/parameter_utils.hpp"

namespace autonom_hardware
{

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

}  // namespace autonom_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_hardware::OdometryCovarianceRelay)
