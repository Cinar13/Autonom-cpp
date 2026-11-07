#include "autonom_patrol/loop_patrol_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>

namespace autonom_patrol
{
namespace
{

constexpr int kThrottleMs = 10000;

rclcpp::QoS latchedQos() { return rclcpp::QoS(1).reliable().transient_local(); }

const char * resultName(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "FAILED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

LoopPatrolNode::LoopPatrolNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("loop_patrol", options)
{
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  robot_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
  min_known_area_ = declare_parameter<double>("min_known_area", 4.0);
  lap_pause_ = declare_parameter<double>("lap_pause", 2.0);
  retry_period_ = declare_parameter<double>("retry_period", 3.0);
  preempt_poses_remaining_ = static_cast<int>(declare_parameter<std::int64_t>("preempt_poses_remaining", 1));
  // Map slowly, race fast: at full speed the robot overshot hairpins into corner pockets while the map was still
  // incomplete. 0 leaves the controller's own desired_linear_vel untouched.
  exploration_speed_ = declare_parameter<double>("exploration_speed", 0.0);
  race_speed_ = declare_parameter<double>("race_speed", 0.0);
  const auto controller = declare_parameter<std::string>("controller_server", "controller_server");
  // Rear gate: keepout wall across the track behind the robot (global costmap KeepoutFilter), so Nav2 can never
  // plan a path that turns back (docs/06 section 4).
  rear_gate_enabled_ = declare_parameter<bool>("rear_gate_enabled", true);
  rear_gate_step_ = declare_parameter<double>("rear_gate_update_distance", 0.3);
  behind_drop_radius_ = declare_parameter<double>("behind_drop_radius", 2.0);
  const auto gate_topic = declare_parameter<std::string>("rear_gate_mask_topic", "/rear_gate_mask");
  wait_for_race_start_ = declare_parameter<bool>("wait_for_race_start", true);
  // 'auto': laps run the way the robot faced at the start; it must never cross the start backwards.
  direction_ = declare_parameter<std::string>("direction", "auto");
  if (direction_ != "auto" && direction_ != "clockwise" && direction_ != "counterclockwise") {
    throw std::invalid_argument("direction must be 'auto', 'clockwise' or 'counterclockwise', got '" + direction_ + "'");
  }
  start_line_half_width_ = declare_parameter<double>("start_line_half_width", 1.0);
  settings_.clearance = declare_parameter<double>("clearance", 0.35);
  settings_.min_spacing = declare_parameter<double>("min_spacing", 0.5);
  settings_.free_threshold = static_cast<int>(declare_parameter<std::int64_t>("free_threshold", 25));
  settings_.waypoint_spacing = declare_parameter<double>("waypoint_spacing", 1.0);
  settings_.min_loop_length = declare_parameter<double>("min_loop_length", 2.0);
  settings_.cut_offset = declare_parameter<double>("rear_gate_offset", 0.4);
  settings_.cut_half_width = declare_parameter<double>("cut_half_width", 1.5);
  settings_.max_exploration_distance = declare_parameter<double>("max_exploration_distance", 2.5);
  const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
  const auto race_state_topic = declare_parameter<std::string>("race_state_topic", "/race/state");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // Dedicated spin thread: TF stays current however long planning takes on the main executor.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, latchedQos(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map_ = std::move(msg); });
  race_state_sub_ = create_subscription<std_msgs::msg::String>(
    race_state_topic, latchedQos(), [this](const std_msgs::msg::String & msg) {
      if (msg.data != race_state_) {
        RCLCPP_INFO(get_logger(), "Race state: %s", msg.data.c_str());
      }
      race_state_ = msg.data;
    });
  gate_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(gate_topic, latchedQos());
  loop_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
  explore_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  controller_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, controller);
  // Wall timer: runs before /clock arrives and is independent of the simulation rate.
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { onTick(); });
}

LoopPatrolNode::~LoopPatrolNode()
{
  cancelGoal();
}

bool LoopPatrolNode::racing() const
{
  // Nav2 goals are allowed only after the start gate's 5 s delay (AC-3.2).
  return !wait_for_race_start_ || race_state_ == "RACING";
}

void LoopPatrolNode::onTick()
{
  if (phase_ == Phase::kWaitNav2) {
    waitForNav2();
    return;
  }
  if (start_monitor_) {
    if (const auto pose = robotPose()) {
      watchStartLine(*pose);
    }
  }
  if (phase_ == Phase::kActive) {
    watchActive();
    return;
  }
  if (!racing()) {
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "Waiting for the race start (/race/state)...");
    return;
  }
  if (steady_clock_.now() >= next_plan_time_) {
    planAndSend();
  }
}

void LoopPatrolNode::waitForNav2()
{
  // Own get_state polling with a timeout: a request that reaches a node while it is still starting can lose its
  // response, and an unbounded wait (BasicNavigator.waitUntilNav2Active) then hangs forever.
  if (nav2_index_ >= nav2_nodes_.size()) {
    phase_ = Phase::kIdle;
    return;
  }
  const auto & name = nav2_nodes_[nav2_index_];
  const auto now = steady_clock_.now();
  if (!state_client_) {
    state_client_ = create_client<lifecycle_msgs::srv::GetState>(name + "/get_state");
  }
  if (!state_future_) {
    if (!state_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "Waiting for %s...", name.c_str());
      return;
    }
    if (now < next_plan_time_) {
      return;
    }
    auto request = state_client_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    state_request_id_ = request.request_id;
    state_future_ = request.future.share();
    state_request_time_ = now;
    return;
  }
  if (state_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const auto response = state_future_->get();
    state_future_.reset();
    if (response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(get_logger(), "%s is active", name.c_str());
      ++nav2_index_;
      state_client_.reset();
    } else {
      next_plan_time_ = now + rclcpp::Duration::from_seconds(1.0);
    }
  } else if ((now - state_request_time_).seconds() > 2.0) {
    state_client_->remove_pending_request(*state_request_id_);
    state_future_.reset();
    next_plan_time_ = now + rclcpp::Duration::from_seconds(1.0);
  }
}

std::optional<Waypoint> LoopPatrolNode::robotPose() const
{
  try {
    const auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero).transform;
    const auto & q = t.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return Waypoint{t.translation.x, t.translation.y, yaw};
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<Grid> LoopPatrolNode::currentGrid() const
{
  if (!map_) {
    return std::nullopt;
  }
  Grid grid;
  grid.width = static_cast<int>(map_->info.width);
  grid.height = static_cast<int>(map_->info.height);
  grid.resolution = map_->info.resolution;
  grid.origin_x = map_->info.origin.position.x;
  grid.origin_y = map_->info.origin.position.y;
  grid.data.assign(map_->data.begin(), map_->data.end());
  if (!grid.valid()) {
    return std::nullopt;
  }
  return grid;
}

void LoopPatrolNode::planAndSend()
{
  const auto pose = robotPose();
  const auto grid = currentGrid();
  const auto now = steady_clock_.now();
  if (!pose || !grid) {
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "Waiting for /map and robot pose...");
    next_plan_time_ = now + rclcpp::Duration::from_seconds(retry_period_);
    return;
  }
  const Point robot{pose->x, pose->y};
  if (!start_line_) {
    start_line_ = StartLine{pose->x, pose->y, pose->yaw};
    start_monitor_.emplace(*start_line_, start_line_half_width_);
    gate_anchor_ = robot;
    gate_direction_ = pose->yaw;
    publishRearGate();
    RCLCPP_INFO(get_logger(), "Start line at (%.2f, %.2f), race heading %.2f rad", pose->x, pose->y, pose->yaw);
  }
  // Wall-aware start zone from the latest map: a passage behind a wall is never "at the start line".
  start_zone_ = startZoneMask(*grid, *start_line_, start_line_half_width_, start_monitor_->depth(),
    settings_.free_threshold);
  if (!start_zone_.empty()) {
    zone_grid_ = grid;
    zone_grid_->data.clear();  // geometry only
    start_monitor_->zone = [this](Point p) {
      const Cell c = zone_grid_->cellOf(p);
      return zone_grid_->inside(c.row, c.col) && start_zone_[zone_grid_->index(c.row, c.col)] != 0;
    };
  }

  const auto known = std::count_if(grid->data.begin(), grid->data.end(), [](std::int8_t v) { return v >= 0; });
  if (static_cast<double>(known) * grid->resolution * grid->resolution >= min_known_area_) {
    const auto waypoints = planRaceLoop(*grid, robot, pose->yaw);
    if (!waypoints.empty()) {
      sendLoop(waypoints);
      return;
    }
  }
  const double travel = gate_direction_ ? *gate_direction_ : pose->yaw;
  if (const auto goal = planExplorationGoal(*grid, robot, settings_, travel)) {
    sendExploration(*goal);
    return;
  }
  RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "No reachable loop or frontier in the explored space yet");
  next_plan_time_ = now + rclcpp::Duration::from_seconds(retry_period_);
}

std::vector<Waypoint> LoopPatrolNode::planRaceLoop(const Grid & grid, Point robot, double heading)
{
  const double direction = gate_direction_ ? *gate_direction_ : heading;
  const auto waypoints = planLoop(grid, robot, direction, settings_);
  if (waypoints.empty()) {
    if (!last_loop_.empty()) {
      // An obstacle on the track (another robot) breaks the closed ring in the map for a while; keep lapping on
      // the last valid loop and let Nav2 route around the obstacle.
      RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "Track ring not closed right now; reusing the last loop");
      return aheadOnly(last_loop_, robot, heading);
    }
    return {};
  }
  if (direction_ != "auto" && loopIsClockwise(waypoints) != (direction_ == "clockwise")) {
    // Laps follow the travel direction set at the start; a forced direction that disagrees would need a U-turn.
    RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, kThrottleMs,
      "direction '%s' runs against the robot's start heading: place the robot facing the race direction. "
      "Not driving this loop.", direction_.c_str());
    return {};
  }
  last_loop_ = waypoints;
  return aheadOnly(waypoints, robot, heading);
}

std::vector<Waypoint> LoopPatrolNode::aheadOnly(
  const std::vector<Waypoint> & waypoints, Point robot, double heading) const
{
  const double direction = gate_direction_ ? *gate_direction_ : heading;
  // The lap is a cycle starting at the waypoint nearest the robot (needed when a stored lap is reused from another
  // position); then skip anything just behind the robot.
  const auto nearest = std::min_element(waypoints.begin(), waypoints.end(), [&robot](const auto & a, const auto & b) {
    return std::hypot(a.x - robot.x, a.y - robot.y) < std::hypot(b.x - robot.x, b.y - robot.y);
  });
  std::vector<Waypoint> cycle(nearest, waypoints.end());
  cycle.insert(cycle.end(), waypoints.begin(), nearest);
  return startAhead(dropBehind(cycle, robot, direction, behind_drop_radius_), robot, direction);
}

geometry_msgs::msg::PoseStamped LoopPatrolNode::toPose(const Waypoint & w) const
{
  // Zero stamp ("latest TF"): valid under both wall and simulated time.
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.pose.position.x = w.x;
  pose.pose.position.y = w.y;
  pose.pose.orientation.z = std::sin(0.5 * w.yaw);
  pose.pose.orientation.w = std::cos(0.5 * w.yaw);
  return pose;
}

void LoopPatrolNode::sendLoop(const std::vector<Waypoint> & waypoints)
{
  if (!loop_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "navigate_through_poses not available yet");
    next_plan_time_ = steady_clock_.now() + rclcpp::Duration::from_seconds(1.0);
    return;
  }
  ++lap_;
  label_ = "Lap " + std::to_string(lap_);
  RCLCPP_INFO(get_logger(), "%s: %zu waypoint(s)", label_.c_str(), waypoints.size());
  setSpeed(race_speed_);
  NavigateThroughPoses::Goal goal;
  for (const auto & w : waypoints) {
    goal.poses.push_back(toPose(w));
  }
  const auto seq = ++goal_seq_;
  rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
  options.goal_response_callback = [this, seq](const auto & handle) {
    if (seq != goal_seq_) {
      return;
    }
    loop_handle_ = handle;
    if (!handle) {
      goal_done_ = true;
      goal_result_ = rclcpp_action::ResultCode::ABORTED;
    }
  };
  options.feedback_callback = [this, seq](auto, const auto & feedback) {
    if (seq == goal_seq_) {
      poses_remaining_ = feedback->number_of_poses_remaining;
    }
  };
  options.result_callback = [this, seq](const auto & result) {
    if (seq == goal_seq_) {
      goal_done_ = true;
      goal_result_ = result.code;
    }
  };
  kind_ = Kind::kLoop;
  goal_done_ = false;
  poses_remaining_ = -1;
  phase_ = Phase::kActive;
  // One continuous path through the whole lap; sending the next lap preempts this goal.
  loop_client_->async_send_goal(goal, options);
}

void LoopPatrolNode::sendExploration(const Waypoint & target)
{
  if (!explore_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "navigate_to_pose not available yet");
    next_plan_time_ = steady_clock_.now() + rclcpp::Duration::from_seconds(1.0);
    return;
  }
  char label[96];
  std::snprintf(label, sizeof(label), "Exploration step to (%.2f, %.2f)", target.x, target.y);
  label_ = label;
  RCLCPP_INFO(get_logger(), "%s: 1 waypoint(s)", label_.c_str());
  setSpeed(exploration_speed_);
  NavigateToPose::Goal goal;
  goal.pose = toPose(target);
  const auto seq = ++goal_seq_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = [this, seq](const auto & handle) {
    if (seq != goal_seq_) {
      return;
    }
    explore_handle_ = handle;
    if (!handle) {
      goal_done_ = true;
      goal_result_ = rclcpp_action::ResultCode::ABORTED;
    }
  };
  options.result_callback = [this, seq](const auto & result) {
    if (seq == goal_seq_) {
      goal_done_ = true;
      goal_result_ = result.code;
    }
  };
  kind_ = Kind::kExplore;
  goal_done_ = false;
  poses_remaining_ = -1;
  phase_ = Phase::kActive;
  explore_client_->async_send_goal(goal, options);
}

void LoopPatrolNode::watchActive()
{
  const auto now = steady_clock_.now();
  if (!racing()) {
    RCLCPP_WARN(get_logger(), "Race stopped during %s; cancelling the goal", label_.c_str());
    cancelGoal();
    phase_ = Phase::kIdle;
    return;
  }
  if (kind_ == Kind::kLoop && poses_remaining_ > 0 && poses_remaining_ <= preempt_poses_remaining_ && !goal_done_) {
    // Plan and send the next lap while still driving, so the robot never brakes for a lap end.
    phase_ = Phase::kIdle;
    next_plan_time_ = now;
    return;
  }
  if (!goal_done_) {
    return;
  }
  if (goal_result_ != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(get_logger(), "%s ended with %s", label_.c_str(), resultName(goal_result_));
  }
  phase_ = Phase::kIdle;
  next_plan_time_ = now + rclcpp::Duration::from_seconds(kind_ == Kind::kLoop ? lap_pause_ : 0.0);
}

void LoopPatrolNode::cancelGoal()
{
  if (loop_handle_) {
    loop_client_->async_cancel_goal(loop_handle_);
    loop_handle_.reset();
  }
  if (explore_handle_) {
    explore_client_->async_cancel_goal(explore_handle_);
    explore_handle_.reset();
  }
  ++goal_seq_;  // ignore results of the cancelled goal
}

void LoopPatrolNode::setSpeed(double speed)
{
  // Path follower cruise speed (RPP FollowPath.desired_linear_vel), set only when it changes.
  if (speed <= 0.0 || (speed_ && *speed_ == speed)) {
    return;
  }
  if (!controller_params_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kThrottleMs, "Could not set cruise speed %.2f m/s", speed);
    return;
  }
  speed_ = speed;
  controller_params_->set_parameters(
    {rclcpp::Parameter("FollowPath.desired_linear_vel", speed)},
    [this, speed](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      if (std::all_of(results.begin(), results.end(), [](const auto & r) { return r.successful; })) {
        RCLCPP_INFO(get_logger(), "Cruise speed %.2f m/s", speed);
      } else {
        speed_.reset();
        RCLCPP_WARN(get_logger(), "Could not set cruise speed %.2f m/s", speed);
      }
    });
}

void LoopPatrolNode::watchStartLine(const Waypoint & pose)
{
  // Count forward crossings as laps; report reverse crossings (team rule, docs/06 section 4).
  updateRearGate(pose);
  const int crossing = start_monitor_->update({pose.x, pose.y});
  if (crossing > 0) {
    ++laps_completed_;
    const auto now = get_clock()->now();
    if (last_crossing_) {
      RCLCPP_INFO(get_logger(), "Start line crossed forward: %d lap(s) completed in %.1f s", laps_completed_,
        (now - *last_crossing_).seconds());
    } else {
      RCLCPP_INFO(get_logger(), "Start line crossed forward: %d lap(s) completed", laps_completed_);
    }
    last_crossing_ = now;
  } else if (crossing < 0) {
    ++reverse_crossings_;
    RCLCPP_ERROR(get_logger(), "REVERSE crossing of the start line (%d): against the rules", reverse_crossings_);
  }
}

void LoopPatrolNode::updateRearGate(const Waypoint & pose)
{
  // Advance the gate with the robot's forward travel (displacement agreeing with its heading); never move it while
  // the robot backs up. Comparing with the gate's previous direction instead failed in the arena's hairpins.
  if (!rear_gate_enabled_ || !gate_anchor_) {
    return;
  }
  const double dx = pose.x - gate_anchor_->x;
  const double dy = pose.y - gate_anchor_->y;
  if (std::hypot(dx, dy) < rear_gate_step_) {
    return;
  }
  const double direction = std::atan2(dy, dx);
  const double agreement = std::cos(direction - pose.yaw);
  if (agreement < 0.5) {
    if (agreement < 0.0) {
      gate_anchor_ = Point{pose.x, pose.y};  // backing up or pushed: re-anchor, keep the direction
    }
    return;
  }
  gate_anchor_ = Point{pose.x, pose.y};
  gate_direction_ = direction;
  publishRearGate();
}

void LoopPatrolNode::publishRearGate()
{
  if (!rear_gate_enabled_ || !gate_anchor_ || !gate_direction_) {
    return;
  }
  const auto grid = currentGrid();
  if (!grid) {
    return;
  }
  nav_msgs::msg::OccupancyGrid mask;
  mask.header.frame_id = map_->header.frame_id.empty() ? global_frame_ : map_->header.frame_id;
  mask.header.stamp = get_clock()->now();
  mask.info = map_->info;
  mask.data.assign(grid->data.size(), 0);
  for (const auto & cell : rearGate(*grid, *gate_anchor_, *gate_direction_, settings_.cut_offset,
      settings_.cut_half_width).cells)
  {
    mask.data[grid->index(cell.row, cell.col)] = 100;
  }
  gate_pub_->publish(mask);
}

}  // namespace autonom_patrol

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_patrol::LoopPatrolNode)
