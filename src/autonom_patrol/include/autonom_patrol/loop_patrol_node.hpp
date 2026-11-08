#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonom_patrol/track_planner.hpp"

namespace autonom_patrol
{

/// Race patrol: explores the track ahead until the ring closes, then drives continuous laps in the start direction.
///
/// Event-driven state machine on a 20 Hz wall timer (wait for Nav2 -> wait for RACING -> plan -> drive), so planning
/// runs only when a new goal is needed. Never turns back: a rear-gate keepout mask follows the robot's forward travel,
/// and laps and exploration goals are planned by driving distance ahead (track_planner.hpp).
class LoopPatrolNode : public rclcpp::Node
{
public:
  explicit LoopPatrolNode(const rclcpp::NodeOptions & options);
  ~LoopPatrolNode() override;

private:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  enum class Phase { kWaitNav2, kIdle, kActive };
  enum class Kind { kLoop, kExplore };

  void onTick();
  void waitForNav2();
  void planAndSend();
  void watchActive();
  [[nodiscard]] bool racing() const;
  [[nodiscard]] std::optional<Waypoint> robotPose() const;  // (x, y, yaw) in the global frame
  [[nodiscard]] std::optional<Grid> currentGrid() const;
  std::vector<Waypoint> planRaceLoop(const Grid & grid, Point robot, double heading);
  std::vector<Waypoint> aheadOnly(const std::vector<Waypoint> & waypoints, Point robot, double heading) const;
  void sendLoop(const std::vector<Waypoint> & waypoints);
  void sendExploration(const Waypoint & goal);
  void cancelGoal();
  void setSpeed(double speed, bool log = true);
  void setAcceleration(const std::vector<double> & max_accel);
  void startFastModeTimer();
  void updateDrivingProfile();
  void updateSectionSpeed();
  void watchStartLine(const Waypoint & pose);
  void updateRearGate(const Waypoint & pose);
  void publishRearGate();
  void requestLocalizationMode();
  [[nodiscard]] geometry_msgs::msg::PoseStamped toPose(const Waypoint & w) const;

  // Parameters
  std::string global_frame_;
  std::string robot_frame_;
  double min_known_area_{4.0};
  double lap_pause_{0.0};
  double retry_period_{3.0};
  int preempt_poses_remaining_{1};
  double exploration_speed_{0.0};
  double race_speed_{0.0};
  std::vector<double> exploration_max_accel_;  // velocity_smoother max_accel [x, y, theta]; empty = untouched
  std::vector<double> race_max_accel_;
  double race_speed_delay_{0.0};
  SectionSpeedSettings sections_;  // race_speed filled in at use
  bool rear_gate_enabled_{true};
  double rear_gate_step_{0.3};
  double behind_drop_radius_{2.0};
  bool wait_for_race_start_{true};
  std::string direction_{"auto"};
  double start_line_half_width_{1.0};
  int localize_after_laps_{0};
  LoopSettings settings_;

  // State
  Phase phase_{Phase::kWaitNav2};
  std::vector<std::string> nav2_nodes_{"slam_toolbox", "bt_navigator"};
  std::size_t nav2_index_{0};
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_client_;
  std::optional<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture> state_future_;
  std::optional<std::int64_t> state_request_id_;
  // Retry / timeout bookkeeping on the steady clock: simulated time is zero until /clock arrives.
  mutable rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time state_request_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time next_plan_time_{0, 0, RCL_STEADY_TIME};

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::string race_state_{"UNKNOWN"};
  std::optional<StartLine> start_line_;
  std::optional<StartLineMonitor> start_monitor_;
  Mask start_zone_;
  std::optional<Grid> zone_grid_;
  int laps_completed_{0};
  int reverse_crossings_{0};
  std::optional<rclcpp::Time> last_crossing_;
  std::vector<Waypoint> last_loop_;
  std::vector<Waypoint> section_loop_;  // lap the section flags were computed for
  std::vector<double> section_speed_;
  int lap_{0};
  std::optional<Point> gate_anchor_;
  std::optional<double> gate_direction_;
  std::optional<double> speed_;
  std::vector<double> accel_;  // last max_accel the smoother accepted
  std::optional<rclcpp::Time> fast_at_;  // node clock (sim time in Gazebo): race speed from here on
  bool fast_{false};
  int crossings_with_lap_{0};  // forward start-line crossings since the first lap was found
  bool localization_requested_{false};

  Kind kind_{Kind::kExplore};
  std::string label_;
  std::uint64_t goal_seq_{0};
  bool goal_done_{false};
  rclcpp_action::ResultCode goal_result_{rclcpp_action::ResultCode::UNKNOWN};
  int poses_remaining_{-1};
  rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr loop_handle_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr explore_handle_;

  // ROS interfaces
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr race_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gate_pub_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr loop_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr explore_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_params_;
  std::shared_ptr<rclcpp::AsyncParametersClient> smoother_params_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr localization_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonom_patrol
