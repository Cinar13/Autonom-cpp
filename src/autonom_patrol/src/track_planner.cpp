#include "autonom_patrol/track_planner.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace autonom_patrol
{
namespace
{

constexpr int kRowStep[4] = {1, -1, 0, 0};
constexpr int kColStep[4] = {0, 0, 1, -1};

Mask freeMask(const Grid & grid, int free_threshold)
{
  Mask free(grid.data.size(), 0);
  for (std::size_t i = 0; i < free.size(); ++i) {
    free[i] = grid.isFree(i, free_threshold) ? 1 : 0;
  }
  return free;
}

/// "No obstacle" stand-in: large but finite, so the parabola arithmetic below stays well defined.
constexpr double kFar = 1e20;

/// 1D squared Euclidean distance transform (Felzenszwalb & Huttenlocher, 2012) of `f` into `d`.
void edt1d(const std::vector<double> & f, std::vector<double> & d, std::vector<int> & v, std::vector<double> & z)
{
  const int n = static_cast<int>(f.size());
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * (q - v[k]));
    while (s <= z[k]) {
      --k;
      s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const double dq = q - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

}  // namespace

bool Grid::valid() const noexcept
{
  return width > 0 && height > 0 && resolution > 0.0 &&
         data.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

Cell Grid::cellOf(Point p) const noexcept
{
  return {static_cast<int>(std::floor((p.y - origin_y) / resolution)),
    static_cast<int>(std::floor((p.x - origin_x) / resolution))};
}

Point Grid::centre(int row, int col) const noexcept
{
  return {origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution};
}

Mask safeFreeSpace(const Grid & grid, int clearance_cells, int free_threshold)
{
  const int w = grid.width;
  const int h = grid.height;
  std::vector<int> distance(grid.data.size(), -1);
  std::deque<int> queue;
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (!grid.isFree(i, free_threshold)) {
      distance[i] = 0;
      queue.push_back(static_cast<int>(i));
    }
  }
  while (!queue.empty()) {
    const int i = queue.front();
    queue.pop_front();
    if (distance[i] >= clearance_cells) {
      continue;
    }
    const int row = i / w;
    const int col = i % w;
    for (int k = 0; k < 4; ++k) {
      const int r = row + kRowStep[k];
      const int c = col + kColStep[k];
      if (grid.inside(r, c) && distance[grid.index(r, c)] < 0) {
        distance[grid.index(r, c)] = distance[i] + 1;
        queue.push_back(static_cast<int>(grid.index(r, c)));
      }
    }
  }
  Mask safe(grid.data.size(), 0);
  for (int row = 0; row < h; ++row) {
    for (int col = 0; col < w; ++col) {
      // Outside the map counts as an obstacle one cell beyond the border.
      const int border = std::min({row + 1, h - row, col + 1, w - col});
      const int d = distance[grid.index(row, col)];
      safe[grid.index(row, col)] = (d < 0 || d > clearance_cells) && border > clearance_cells ? 1 : 0;
    }
  }
  return safe;
}

std::vector<int> drivingDistance(const Mask & passable, int width, int height, Cell seed)
{
  std::vector<int> distance(passable.size(), -1);
  const auto index = [width](int r, int c) { return static_cast<std::size_t>(r) * width + c; };
  if (seed.row < 0 || seed.row >= height || seed.col < 0 || seed.col >= width) {
    return distance;
  }
  std::deque<Cell> queue{seed};
  distance[index(seed.row, seed.col)] = 0;
  while (!queue.empty()) {
    const Cell cell = queue.front();
    queue.pop_front();
    const int next = distance[index(cell.row, cell.col)] + 1;
    for (int k = 0; k < 4; ++k) {
      const int r = cell.row + kRowStep[k];
      const int c = cell.col + kColStep[k];
      if (r < 0 || r >= height || c < 0 || c >= width) {
        continue;
      }
      const std::size_t i = index(r, c);
      if (passable[i] && distance[i] < 0) {
        distance[i] = next;
        queue.push_back({r, c});
      }
    }
  }
  return distance;
}

Mask reachableFrom(const Mask & passable, int width, int height, Cell seed)
{
  const auto distance = drivingDistance(passable, width, height, seed);
  Mask reached(passable.size(), 0);
  for (std::size_t i = 0; i < reached.size(); ++i) {
    reached[i] = distance[i] >= 0 ? 1 : 0;
  }
  return reached;
}

std::vector<double> distanceTransform(const Mask & free, int width, int height)
{
  std::vector<double> squared(free.size());
  for (std::size_t i = 0; i < free.size(); ++i) {
    squared[i] = free[i] ? kFar : 0.0;
  }
  const int longest = std::max(width, height);
  std::vector<double> f(longest), d(longest), z(longest + 1);
  std::vector<int> v(longest);
  for (int col = 0; col < width; ++col) {
    f.resize(height);
    d.resize(height);
    for (int row = 0; row < height; ++row) {
      f[row] = squared[static_cast<std::size_t>(row) * width + col];
    }
    edt1d(f, d, v, z);
    for (int row = 0; row < height; ++row) {
      squared[static_cast<std::size_t>(row) * width + col] = d[row];
    }
  }
  for (int row = 0; row < height; ++row) {
    f.resize(width);
    d.resize(width);
    for (int col = 0; col < width; ++col) {
      f[col] = squared[static_cast<std::size_t>(row) * width + col];
    }
    edt1d(f, d, v, z);
    for (int col = 0; col < width; ++col) {
      squared[static_cast<std::size_t>(row) * width + col] = d[col];
    }
  }
  for (auto & value : squared) {
    value = std::sqrt(value);
  }
  return squared;
}

std::optional<Cell> nearestCell(const Mask & mask, const Grid & grid, Point p, std::optional<double> ahead_of)
{
  const double robot_col = (p.x - grid.origin_x) / grid.resolution - 0.5;
  const double robot_row = (p.y - grid.origin_y) / grid.resolution - 0.5;
  const double cos_h = ahead_of ? std::cos(*ahead_of) : 0.0;
  const double sin_h = ahead_of ? std::sin(*ahead_of) : 0.0;
  std::optional<Cell> best;
  std::optional<Cell> best_ahead;
  double best_distance = std::numeric_limits<double>::infinity();
  double best_ahead_distance = std::numeric_limits<double>::infinity();
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      if (!mask[grid.index(row, col)]) {
        continue;
      }
      const double dr = row - robot_row;
      const double dc = col - robot_col;
      const double d = dr * dr + dc * dc;
      if (d < best_distance) {
        best_distance = d;
        best = Cell{row, col};
      }
      if (ahead_of && dc * cos_h + dr * sin_h >= 0.0 && d < best_ahead_distance) {
        best_ahead_distance = d;
        best_ahead = Cell{row, col};
      }
    }
  }
  return best_ahead ? best_ahead : best;
}

Gate rearGate(const Grid & grid, Point xy, double direction, double offset, double max_half_width, int occupied_threshold)
{
  Gate gate;
  if (!grid.valid()) {
    return gate;
  }
  const double cos_d = std::cos(direction);
  const double sin_d = std::sin(direction);
  const Point centre{xy.x - offset * cos_d, xy.y - offset * sin_d};
  const double step = 0.5 * grid.resolution;
  Mask marked(grid.data.size(), 0);
  int walls_hit = 0;
  for (const double side : {1.0, -1.0}) {
    for (double distance = 0.0; distance <= max_half_width; distance += step) {
      const Cell cell = grid.cellOf({centre.x - side * distance * sin_d, centre.y + side * distance * cos_d});
      if (!grid.inside(cell.row, cell.col)) {
        break;
      }
      for (const auto & [dr, dc] : {std::pair{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
        const int r = cell.row + dr;
        const int c = cell.col + dc;
        if (grid.inside(r, c) && !marked[grid.index(r, c)]) {
          marked[grid.index(r, c)] = 1;
          gate.cells.push_back({r, c});
        }
      }
      if (grid.data[grid.index(cell.row, cell.col)] >= occupied_threshold) {
        ++walls_hit;
        break;
      }
    }
  }
  gate.sealed = walls_hit == 2;
  return gate;
}

std::vector<Waypoint> planLoop(const Grid & grid, Point robot, double heading, const LoopSettings & settings)
{
  if (!grid.valid() || settings.waypoint_spacing <= 0.0) {
    return {};
  }
  const double res = grid.resolution;
  const int clearance_cells = std::max(1, static_cast<int>(std::ceil(settings.clearance / res)));
  Mask passable = safeFreeSpace(grid, clearance_cells, settings.free_threshold);
  const Gate cut = rearGate(grid, robot, heading, settings.cut_offset, settings.cut_half_width);
  if (!cut.sealed) {
    return {};  // an open cut end would let a short detour count as a lap
  }
  for (const auto & cell : cut.cells) {
    passable[grid.index(cell.row, cell.col)] = 0;
  }
  const auto seed = nearestCell(passable, grid, robot, heading);
  if (!seed) {
    return {};
  }
  const auto distance = drivingDistance(passable, grid.width, grid.height, *seed);

  // The cell straight behind the cut, in the robot's own corridor: reached only by driving the whole ring.
  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);
  const double back = settings.cut_offset + 0.1;
  const auto target = nearestCell(passable, grid, {robot.x - back * cos_h, robot.y - back * sin_h});
  if (!target || distance[grid.index(target->row, target->col)] < 0) {
    return {};
  }
  const Point t = grid.centre(target->row, target->col);
  if ((t.x - robot.x) * cos_h + (t.y - robot.y) * sin_h > -settings.cut_offset) {
    return {};  // nearest passable cell is not behind the cut (cut sealed nothing)
  }
  const int lap_steps = distance[grid.index(target->row, target->col)];
  if (lap_steps * res < settings.min_loop_length) {
    return {};
  }

  const auto wall_distance = distanceTransform(freeMask(grid, settings.free_threshold), grid.width, grid.height);
  const int spacing_steps = std::max(2, static_cast<int>(std::lround(settings.waypoint_spacing / res)));
  std::vector<Point> points;
  for (int level = spacing_steps; level <= lap_steps - spacing_steps / 2; level += spacing_steps) {
    double best_value = -1.0;
    std::optional<std::size_t> best;
    for (std::size_t i = 0; i < distance.size(); ++i) {
      if (distance[i] >= level - 1 && distance[i] <= level + 1 && wall_distance[i] > best_value) {
        best_value = wall_distance[i];
        best = i;
      }
    }
    if (best) {
      points.push_back(grid.centre(static_cast<int>(*best / grid.width), static_cast<int>(*best % grid.width)));
    }
  }
  if (points.size() < 2) {
    return {};
  }
  std::vector<Waypoint> waypoints;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Point next = i + 1 < points.size() ? points[i + 1] : robot;
    waypoints.push_back({points[i].x, points[i].y, std::atan2(next.y - points[i].y, next.x - points[i].x)});
  }
  return waypoints;
}

std::optional<Waypoint> planExplorationGoal(
  const Grid & grid, Point robot, const LoopSettings & settings, std::optional<double> heading)
{
  if (!grid.valid()) {
    return std::nullopt;
  }
  const double res = grid.resolution;
  const int clearance_cells = std::max(1, static_cast<int>(std::ceil(settings.clearance / res)));
  Mask safe = safeFreeSpace(grid, clearance_cells, settings.free_threshold);
  if (heading) {
    for (const auto & cell : rearGate(grid, robot, *heading, settings.cut_offset, settings.cut_half_width).cells) {
      safe[grid.index(cell.row, cell.col)] = 0;
    }
  }
  const auto seed = nearestCell(safe, grid, robot, heading);
  if (!seed) {
    return std::nullopt;
  }
  const auto steps = drivingDistance(safe, grid.width, grid.height, *seed);

  if (!heading) {
    double best_distance = -1.0;
    Point best_point;
    for (std::size_t i = 0; i < steps.size(); ++i) {
      if (steps[i] < 0) {
        continue;
      }
      const Point p = grid.centre(static_cast<int>(i / grid.width), static_cast<int>(i % grid.width));
      double d = std::hypot(p.x - robot.x, p.y - robot.y);
      if (settings.max_exploration_distance > 0.0 && d > settings.max_exploration_distance) {
        d = -1.0;
      }
      if (d > best_distance) {
        best_distance = d;
        best_point = p;
      }
    }
    if (best_distance < settings.min_spacing) {
      return std::nullopt;
    }
    return Waypoint{best_point.x, best_point.y, std::atan2(best_point.y - robot.y, best_point.x - robot.x)};
  }

  // Cells behind the cut but still connected (an unsealed cut end) are never goals.
  const double cos_h = std::cos(*heading);
  const double sin_h = std::sin(*heading);
  std::vector<int> reach(steps.size(), -1);
  int max_reach = -1;
  for (std::size_t i = 0; i < steps.size(); ++i) {
    if (steps[i] < 0) {
      continue;
    }
    const Point p = grid.centre(static_cast<int>(i / grid.width), static_cast<int>(i % grid.width));
    if ((p.x - robot.x) * cos_h + (p.y - robot.y) * sin_h > 0.0) {
      reach[i] = steps[i];
      max_reach = std::max(max_reach, steps[i]);
    }
  }
  if (max_reach * res < settings.min_spacing) {
    return std::nullopt;
  }
  int target = max_reach;
  if (settings.max_exploration_distance > 0.0) {
    target = std::min(target, static_cast<int>(std::lround(settings.max_exploration_distance / res)));
  }
  const auto clearance = distanceTransform(freeMask(grid, settings.free_threshold), grid.width, grid.height);
  double best_value = -1.0;
  std::optional<std::size_t> best;
  for (std::size_t i = 0; i < reach.size(); ++i) {
    if (steps[i] >= 0 && std::abs(reach[i] - target) <= 1 && clearance[i] > best_value) {
      best_value = clearance[i];
      best = i;
    }
  }
  if (!best) {
    return std::nullopt;
  }
  const Point p = grid.centre(static_cast<int>(*best / grid.width), static_cast<int>(*best % grid.width));
  return Waypoint{p.x, p.y, std::atan2(p.y - robot.y, p.x - robot.x)};
}

bool loopIsClockwise(const std::vector<Waypoint> & waypoints)
{
  double area = 0.0;
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto & a = waypoints[i];
    const auto & b = waypoints[(i + 1) % waypoints.size()];
    area += a.x * b.y - b.x * a.y;
  }
  return area < 0.0;
}

std::vector<Waypoint> startAhead(std::vector<Waypoint> waypoints, Point xy, double heading)
{
  if (waypoints.size() < 2) {
    return waypoints;
  }
  const auto & first = waypoints.front();
  if ((first.x - xy.x) * std::cos(heading) + (first.y - xy.y) * std::sin(heading) >= 0.0) {
    return waypoints;
  }
  std::rotate(waypoints.begin(), waypoints.begin() + 1, waypoints.end());
  return waypoints;
}

std::vector<Waypoint> dropBehind(const std::vector<Waypoint> & waypoints, Point xy, double heading, double radius)
{
  std::vector<Waypoint> kept;
  for (const auto & w : waypoints) {
    const bool behind = (w.x - xy.x) * std::cos(heading) + (w.y - xy.y) * std::sin(heading) < 0.0;
    if (!(behind && std::hypot(w.x - xy.x, w.y - xy.y) < radius)) {
      kept.push_back(w);
    }
  }
  return kept;
}

std::pair<double, double> StartLine::local(Point p) const noexcept
{
  const double dx = p.x - x;
  const double dy = p.y - y;
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  return {dx * c + dy * s, -dx * s + dy * c};
}

StartLineMonitor::StartLineMonitor(StartLine line, double half_width, double depth, double margin)
: line_(line), half_width_(half_width), depth_(depth), margin_(margin)
{
}

int StartLineMonitor::update(Point p)
{
  const auto [along, lateral] = line_.local(p);
  if (std::abs(lateral) > half_width_ || std::abs(along) > depth_ || (zone && !zone(p))) {
    side_.reset();
    return 0;
  }
  if (std::abs(along) <= margin_) {
    return 0;
  }
  const int side = along > 0.0 ? 1 : -1;
  const std::optional<int> previous = side_;
  side_ = side;
  if (!previous || *previous == side) {
    return 0;
  }
  return side;
}

Mask startZoneMask(const Grid & grid, const StartLine & line, double half_width, double depth, int free_threshold)
{
  if (!grid.valid()) {
    return {};
  }
  Mask free(grid.data.size(), 0);
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const auto [along, lateral] = line.local(grid.centre(row, col));
      const std::size_t i = grid.index(row, col);
      free[i] = std::abs(along) <= depth && std::abs(lateral) <= half_width && grid.isFree(i, free_threshold) ? 1 : 0;
    }
  }
  const auto seed = nearestCell(free, grid, {line.x, line.y});
  if (!seed) {
    return {};
  }
  return reachableFrom(free, grid.width, grid.height, *seed);
}

}  // namespace autonom_patrol
