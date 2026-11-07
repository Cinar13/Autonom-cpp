#include "autonom_competitor/lane_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autonom_competitor
{
namespace
{

constexpr double kSampleStep = 0.01;   // m between centre-line samples when projecting the robot onto the loop
constexpr double kBacktrack = 0.1;     // m searched behind the last position (pose noise, overshoot)

}  // namespace

bool robotAhead(const Pose2D & self, const std::vector<Point> & others, double distance, double half_width)
{
  const double c = std::cos(self.yaw);
  const double s = std::sin(self.yaw);
  return std::any_of(others.begin(), others.end(), [&](const Point & p) {
    const double dx = p.x - self.x;
    const double dy = p.y - self.y;
    const double forward = c * dx + s * dy;
    const double lateral = -s * dx + c * dy;
    return forward > 0.0 && forward <= distance && std::abs(lateral) <= half_width;
  });
}

LaneFollower::LaneFollower(std::vector<Point> centre_line, const LaneFollowerSettings & settings)
: points_(std::move(centre_line)), settings_(settings)
{
  if (points_.size() < 3) {
    throw std::invalid_argument("LaneFollower needs a closed centre line of at least 3 points");
  }
  if (settings_.lookahead <= 0.0 || settings_.max_angular <= 0.0 || settings_.search_window <= 0.0) {
    throw std::invalid_argument("lookahead, max_angular and search_window must be positive");
  }
  station_.reserve(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i) {
    station_.push_back(length_);
    const auto & a = points_[i];
    const auto & b = points_[(i + 1) % points_.size()];
    length_ += std::hypot(b.x - a.x, b.y - a.y);
  }
  if (length_ <= 0.0) {
    throw std::invalid_argument("LaneFollower centre line has zero length");
  }
}

double LaneFollower::wrap(double s) const
{
  s = std::fmod(s, length_);
  return s < 0.0 ? s + length_ : s;
}

std::size_t LaneFollower::segmentAt(double s) const
{
  const auto it = std::upper_bound(station_.begin(), station_.end(), wrap(s));
  return static_cast<std::size_t>(std::distance(station_.begin(), it)) - 1;
}

Point LaneFollower::lanePoint(double s, double offset) const
{
  s = wrap(s);
  const auto i = segmentAt(s);
  const auto & a = points_[i];
  const auto & b = points_[(i + 1) % points_.size()];
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  if (len <= 0.0) {
    return a;
  }
  const double t = (s - station_[i]) / len;
  const double tx = (b.x - a.x) / len;
  const double ty = (b.y - a.y) / len;
  return {a.x + t * (b.x - a.x) - offset * ty, a.y + t * (b.y - a.y) + offset * tx};
}

double LaneFollower::lateralOffset(Point p, double s) const
{
  const auto c = lanePoint(s, 0.0);
  const auto n = lanePoint(s, 1.0);  // unit normal = n - c
  return (p.x - c.x) * (n.x - c.x) + (p.y - c.y) * (n.y - c.y);
}

double LaneFollower::project(Point p, double from, double window) const
{
  double best_s = wrap(from);
  double best_d = std::numeric_limits<double>::infinity();
  const auto steps = static_cast<int>(std::ceil(window / kSampleStep));
  for (int k = 0; k <= steps; ++k) {
    const double s = from + k * kSampleStep;
    const auto c = lanePoint(s, 0.0);
    const double d = std::hypot(p.x - c.x, p.y - c.y);
    if (d < best_d) {
      best_d = d;
      best_s = wrap(s);
    }
  }
  return best_s;
}

double LaneFollower::offsetAt(double travelled) const
{
  const double start = start_offset_.value_or(settings_.lane_offset);
  if (settings_.merge_distance <= 0.0) {
    return settings_.lane_offset;
  }
  const double f = std::min(travelled / settings_.merge_distance, 1.0);
  return start + (settings_.lane_offset - start) * f;
}

Command LaneFollower::update(const Pose2D & pose)
{
  const Point p{pose.x, pose.y};
  if (!start_offset_) {
    progress_ = project(p, 0.0, length_);
    start_offset_ = lateralOffset(p, progress_);
  } else {
    const double s = project(p, progress_ - kBacktrack, settings_.search_window + kBacktrack);
    double ds = s - progress_;
    if (ds > 0.5 * length_) {
      ds -= length_;
    } else if (ds < -0.5 * length_) {
      ds += length_;
    }
    if (ds > 0.0) {
      travelled_ += ds;
    }
    progress_ = s;
  }

  const auto target = lanePoint(progress_ + settings_.lookahead, offsetAt(travelled_));
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double c = std::cos(pose.yaw);
  const double sn = std::sin(pose.yaw);
  const double lx = c * dx + sn * dy;
  const double ly = -sn * dx + c * dy;
  const double alpha = std::atan2(ly, lx);
  if (std::abs(alpha) > settings_.turn_in_place_angle) {
    return {0.0, std::copysign(settings_.max_angular, alpha)};
  }
  const double dist2 = lx * lx + ly * ly;
  if (dist2 <= 1e-9) {
    return {settings_.speed, 0.0};
  }
  const double curvature = 2.0 * ly / dist2;
  double v = settings_.speed;
  double w = v * curvature;
  if (std::abs(w) > settings_.max_angular) {
    v = settings_.max_angular / std::abs(curvature);
    w = std::copysign(settings_.max_angular, curvature);
  }
  return {v, w};
}

}  // namespace autonom_competitor
