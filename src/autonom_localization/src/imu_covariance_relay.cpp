#include "autonom_localization/imu_covariance_relay.hpp"

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

std::array<double, 9> declareVariances(
  rclcpp::Node & node, const std::string & name, const std::vector<double> & default_variances)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Variances [x, y, z], the diagonal of the 3x3 covariance (bno055 driver convention)";
  descriptor.read_only = true;
  const auto variances = node.declare_parameter<std::vector<double>>(name, default_variances, descriptor);
  if (variances.size() != 3) {
    throw std::invalid_argument(name + " must contain exactly 3 values");
  }
  std::array<double, 9> covariance{};
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(variances[i]) || variances[i] < 0.0) {
      throw std::invalid_argument(name + " values must be finite and non-negative");
    }
    covariance[i * 4] = variances[i];
  }
  return covariance;
}

}  // namespace

ImuCovarianceRelay::ImuCovarianceRelay(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_covariance_relay", options)
{
  // Defaults match autonom_bringup/config/imu.yaml.
  orientation_covariance_ = declareVariances(*this, "variance_orientation", {0.0159, 0.0159, 0.0159});
  angular_velocity_covariance_ = declareVariances(*this, "variance_angular_vel", {0.0004, 0.0004, 0.0004});
  linear_acceleration_covariance_ = declareVariances(*this, "variance_acc", {0.017, 0.017, 0.017});

  publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", rclcpp::SystemDefaultsQoS());
  subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu_in", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::UniquePtr msg) {
      msg->orientation_covariance = orientation_covariance_;
      msg->angular_velocity_covariance = angular_velocity_covariance_;
      msg->linear_acceleration_covariance = linear_acceleration_covariance_;
      publisher_->publish(std::move(msg));
    });
}

}  // namespace autonom_localization

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_localization::ImuCovarianceRelay)
