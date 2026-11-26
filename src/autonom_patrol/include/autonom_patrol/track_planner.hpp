#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace autonom_patrol
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Cell
{
  int row{0};
  int col{0};
};

/// Row-major occupancy grid in the ROS convention: -1 unknown, 0..100 occupancy, origin at cell (0, 0)'s corner.
struct Grid
{
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::int8_t> data;

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool inside(int row, int col) const noexcept { return row >= 0 && row < height && col >= 0 && col < width; }
  [[nodiscard]] std::size_t index(int row, int col) const noexcept
  {
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(width) + static_cast<std::size_t>(col);
  }
  [[nodiscard]] Cell cellOf(Point p) const noexcept;
  [[nodiscard]] Point centre(int row, int col) const noexcept;
  [[nodiscard]] bool isFree(std::size_t i, int free_threshold) const noexcept
  {
    return data[i] >= 0 && data[i] < free_threshold;
  }
};

/// One byte per cell (0 / 1), same layout as Grid::data.
using Mask = std::vector<std::uint8_t>;

struct LoopSettings
{
  /// Waypoints keep this distance from walls and unknown space (about half the track width minus a margin).
  double clearance{0.35};
  double min_spacing{0.5};
  int free_threshold{25};
  /// Driving distance between lap waypoints.
  double waypoint_spacing{1.0};
  /// Shorter "rings" are not a lap (e.g. a loop around a small obstacle in an open area).
  double min_loop_length{2.0};
  /// The lap is cut this far behind the robot, across at most +-cut_half_width.
  double cut_offset{0.3};
  double cut_half_width{1.5};
  /// Exploration goals farther than this driving distance are ignored (0 = unlimited).
  double max_exploration_distance{0.0};
};

/// Free cells at least `clearance_cells` (4-connected steps) away from occupied, unknown and out-of-map cells.
Mask safeFreeSpace(const Grid & grid, int clearance_cells, int free_threshold);

/// Cells of `passable` 4-connected to `seed`.
Mask reachableFrom(const Mask & passable, int width, int height, Cell seed);

/// 4-connected BFS steps from `seed` through `passable`; -1 where unreachable.
std::vector<int> drivingDistance(const Mask & passable, int width, int height, Cell seed);

/// Euclidean distance (in cells) from every cell of `free` to the nearest cell that is not free.
std::vector<double> distanceTransform(const Mask & free, int width, int height);

/// Nearest cell of `mask` to `p` (cell centres, squared distance, first in row-major order on ties).
std::optional<Cell> nearestCell(const Mask & mask, const Grid & grid, Point p);

struct Gate
{
  std::vector<Cell> cells;
  /// Both ends of the line reached a wall, so nothing can drive around the cut.
  bool sealed{false};
};

/// Wall across the track `offset` m behind `xy` (opposite to `direction`), perpendicular to it, until it reaches an
/// occupied cell or `max_half_width`; each sample also marks its four neighbours so the wall stays sealed. As a
/// global costmap keepout zone it cuts the ring behind the robot: every plan must go forward.
Gate rearGate(
  const Grid & grid, Point xy, double direction, double offset, double max_half_width, int occupied_threshold = 50);

/// Point `distance` metres back along the robot's travelled path (`trail`, oldest first, newest = the robot) and
/// the direction of travel there. Following the path, not the current heading, keeps a gate behind the robot across
/// the corridor it came from even in a corner: a line perpendicular to the exit direction would run along the
/// entry corridor and leave the way back open. nullopt with fewer than 2 points; the oldest point (and its
/// segment's direction) when the trail is shorter than `distance`.
std::optional<std::pair<Point, double>> trailPointBehind(const std::vector<Point> & trail, double distance);

/// Lap waypoints along the closed track ahead of the robot (driving-distance ordering, centre line); empty until
/// the ring is closed, i.e. the far side of a sealed cut behind the robot is reached the long way round.
std::vector<Waypoint> planLoop(const Grid & grid, Point robot, double heading, const LoopSettings & settings);

/// Next exploration goal. With `heading` (direction of travel): the track centre line `max_exploration_distance`
/// ahead in driving distance (or the frontier, if nearer), never behind the robot. Without: the farthest safe cell.
std::optional<Waypoint> planExplorationGoal(
  const Grid & grid, Point robot, const LoopSettings & settings, std::optional<double> heading);

/// planExplorationGoal along `heading`; at a dead end (nothing reachable ahead, e.g. the robot drove past a corner's
/// opening) it also looks 90 deg left and right and takes the farther goal. Never looks back (no turning round).
std::optional<Waypoint> planExplorationGoalOrTurn(
  const Grid & grid, Point robot, const LoopSettings & settings, double heading);

/// Orientation of the lap polygon (negative signed area = clockwise).
bool loopIsClockwise(const std::vector<Waypoint> & waypoints);

/// Rotate the lap so its first waypoint lies ahead of the robot, never just behind it.
std::vector<Waypoint> startAhead(std::vector<Waypoint> waypoints, Point xy, double heading);

/// Remove waypoints within `radius` behind the robot (between it and the rear gate).
std::vector<Waypoint> dropBehind(const std::vector<Waypoint> & waypoints, Point xy, double heading, double radius);

/// For the closed lap (segment i = waypoint i -> i + 1, the last one back to waypoint 0): true for segments of a
/// straight shorter than `max_length` [m]. The lap is split into straights at waypoints where it turns by more
/// than `corner_angle` [rad]; inside a corner every segment counts as its own (short) straight. A lap with no
/// corner is one straight. Fewer than 3 waypoints: all false.
std::vector<bool> shortStraightSegments(const std::vector<Waypoint> & lap, double corner_angle, double max_length);

/// For the closed lap: distance along the track from the end of segment i to the next waypoint where the lap turns
/// by more than `corner_angle` (0 when segment i ends at one). Infinite for a lap without corners.
std::vector<double> distanceToNextCorner(const std::vector<Waypoint> & lap, double corner_angle);

struct SectionSpeedSettings
{
  double race_speed{0.0};
  double corner_angle{0.35};          // [rad] turn at a waypoint that makes it a corner
  double short_straight_length{0.0};  // [m] straights shorter than this ...
  double short_straight_speed{0.0};   // ... are capped at this [m/s] (0: off)
  double corner_approach_distance{0.0};  // [m] segments ending this close before a corner ...
  double corner_approach_speed{0.0};     // ... are capped at this [m/s] (0: off)
};

/// Cruise speed per lap segment: race_speed, capped on short straights and before corners.
std::vector<double> sectionSpeeds(const std::vector<Waypoint> & lap, const SectionSpeedSettings & settings);

/// Segment of the closed lap nearest to `p` (index of its first waypoint); nullopt for fewer than 2 waypoints.
std::optional<std::size_t> nearestSegment(const std::vector<Waypoint> & lap, Point p);

/// Line through the start position, perpendicular to the start heading (the race direction).
struct StartLine
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};

  /// (along, lateral): signed distance ahead of the line and offset along it.
  [[nodiscard]] std::pair<double, double> local(Point p) const noexcept;
};

/// Start-line crossings with hysteresis, tracked only inside the start zone; the robot begins on the forward side.
class StartLineMonitor
{
public:
  explicit StartLineMonitor(StartLine line, double half_width = 1.0, double depth = 1.0, double margin = 0.10);

  /// +1 for a forward crossing (lap completed), -1 for a reverse crossing, 0 otherwise.
  int update(Point p);

  [[nodiscard]] const StartLine & line() const noexcept { return line_; }
  [[nodiscard]] double depth() const noexcept { return depth_; }
  /// Optional wall-aware zone test (see startZoneMask).
  std::function<bool(Point)> zone;

private:
  StartLine line_;
  double half_width_;
  double depth_;
  double margin_;
  std::optional<int> side_{1};
};

/// Free cells of the start box connected to the start position without crossing a wall (empty mask if none).
Mask startZoneMask(const Grid & grid, const StartLine & line, double half_width, double depth, int free_threshold);

}  // namespace autonom_patrol
