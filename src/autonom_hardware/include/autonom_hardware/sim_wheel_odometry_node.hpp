#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "autonom_hardware/diff_drive_kinematics.hpp"

namespace autonom_hardware
{

/// Simulation stand-in for WheelOdometryNode: turns the simulated wheel joint angles (Gazebo joint_states) into
/// encoder counts with the real robot's resolution (ticks_per_revolution, 1372 = 98 CPR x 14:1) and computes
/// /wheel/odom exactly like the real node (count deltas -> wheel angles -> DiffDriveKinematics -> Pose2D).
/// Wheel slip still shows up, because the joint angle is what the wheel really turned.
class SimWheelOdometryNode : public rclcpp::Node
{
public:
  explicit SimWheelOdometryNode(const rclcpp::NodeOptions & options);

private:
  void onJointStates(const sensor_msgs::msg::JointState & msg);

  std::optional<DiffDriveKinematics> kinematics_;
  std::int64_t ticks_per_revolution_{0};
  double radians_per_tick_{0.0};
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string left_joint_name_;
  std::string right_joint_name_;
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};

  bool initialised_{false};
  rclcpp::Time last_stamp_;
  std::int64_t last_left_ticks_{0};
  std::int64_t last_right_ticks_{0};
  Pose2D pose_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

}  // namespace autonom_hardware
