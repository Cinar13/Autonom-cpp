#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace autonom_competitor
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Command
{
  double linear{0.0};   // m/s
  double angular{0.0};  // rad/s
};

struct LaneFollowerSettings
{
  double speed{0.2};              // cruise speed, m/s
  double lane_offset{0.0};        // lateral offset from the centre line, m, positive = left of the travel direction
  double merge_distance{1.0};     // driven distance over which the start offset blends into lane_offset, m
  double lookahead{0.3};          // pure-pursuit lookahead along the lane, m
  double max_angular{2.0};        // rad/s
  double turn_in_place_angle{1.2};  // rad: target further off the heading than this -> turn on the spot
  double search_window{1.0};      // m of centre line searched ahead of the last position (no jumping across walls)
};

/// True when the centre of one of `others` lies ahead of `self` within `distance` and within `half_width` of its
/// heading line (robot frame: 0 < forward <= distance, |lateral| <= half_width). Competitors stop for it instead of
/// ramming (docs/06 section 2): a blind path follower pushed the main robot over in the simulation.
[[nodiscard]] bool robotAhead(const Pose2D & self, const std::vector<Point> & others, double distance,
  double half_width);

/// Pure-pursuit follower of a closed lane around the race track (ROS-free, unit-tested).
///
/// The lane is the closed centre line shifted sideways by `lane_offset`. The robot's own lateral offset at the
/// start (e.g. placed beside the main robot) blends linearly into `lane_offset` over `merge_distance` of driving.
/// Progress along the loop only moves forward, inside `search_window`, so a nearby corridor behind a wall is never
/// mistaken for the current position.
class LaneFollower
{
public:
  /// `centre_line`: closed loop in driving order (the last point connects back to the first), >= 3 points.
  LaneFollower(std::vector<Point> centre_line, const LaneFollowerSettings & settings);

  [[nodiscard]] Command update(const Pose2D & pose);

  /// Arc length of the loop, m.
  [[nodiscard]] double length() const { return length_; }
  /// Position along the loop of the last update, m in [0, length).
  [[nodiscard]] double progress() const { return progress_; }
  /// Total distance driven along the loop since the first update, m.
  [[nodiscard]] double travelled() const { return travelled_; }
  /// Lane point at arc length `s` for the lateral offset `offset`.
  [[nodiscard]] Point lanePoint(double s, double offset) const;
  /// Signed lateral offset of `p` from the centre line at arc length `s` (positive = left).
  [[nodiscard]] double lateralOffset(Point p, double s) const;

private:
  [[nodiscard]] double wrap(double s) const;
  [[nodiscard]] std::size_t segmentAt(double s) const;
  [[nodiscard]] double project(Point p, double from, double window) const;
  [[nodiscard]] double offsetAt(double travelled) const;

  std::vector<Point> points_;
  std::vector<double> station_;  // arc length at each point
  double length_{0.0};
  LaneFollowerSettings settings_;
  std::optional<double> start_offset_;
  double progress_{0.0};
  double travelled_{0.0};
};

}  // namespace autonom_competitor
