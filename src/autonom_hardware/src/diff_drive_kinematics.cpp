#include "autonom_hardware/diff_drive_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autonom_hardware
{
namespace
{

constexpr double kStraightLineThreshold = 1e-9;

}  // namespace

void Pose2D::integrate(const BodyMotion & displacement) noexcept
{
  const double distance = displacement.linear;
  const double rotation = displacement.angular;
  if (!std::isfinite(distance) || !std::isfinite(rotation)) {
    return;
  }
  if (std::abs(rotation) < kStraightLineThreshold) {
    const double heading = theta + 0.5 * rotation;
    x += distance * std::cos(heading);
    y += distance * std::sin(heading);
  } else {
    const double radius = distance / rotation;
    const double next_theta = theta + rotation;
    x += radius * (std::sin(next_theta) - std::sin(theta));
    y -= radius * (std::cos(next_theta) - std::cos(theta));
  }
  theta = std::remainder(theta + rotation, 2.0 * M_PI);
}

DiffDriveKinematics::DiffDriveKinematics(double wheel_radius, double wheel_separation)
: wheel_radius_(wheel_radius), wheel_separation_(wheel_separation)
{
  if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0) {
    throw std::invalid_argument("wheel_radius must be positive");
  }
  if (!std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0) {
    throw std::invalid_argument("wheel_separation must be positive");
  }
}

WheelPair DiffDriveKinematics::toWheels(const BodyMotion & body) const noexcept
{
  const double half_track_rotation = 0.5 * body.angular * wheel_separation_;
  return {(body.linear - half_track_rotation) / wheel_radius_, (body.linear + half_track_rotation) / wheel_radius_};
}

BodyMotion DiffDriveKinematics::toBody(const WheelPair & wheels) const noexcept
{
  const double left = wheels.left * wheel_radius_;
  const double right = wheels.right * wheel_radius_;
  return {0.5 * (left + right), (right - left) / wheel_separation_};
}

WheelPair DiffDriveKinematics::limit(const WheelPair & wheels, double max_wheel_speed) noexcept
{
  if (!std::isfinite(wheels.left) || !std::isfinite(wheels.right) || !(max_wheel_speed > 0.0)) {
    return {};
  }
  const double peak = std::max(std::abs(wheels.left), std::abs(wheels.right));
  if (peak <= max_wheel_speed) {
    return wheels;
  }
  const double scale = max_wheel_speed / peak;
  return {wheels.left * scale, wheels.right * scale};
}

}  // namespace autonom_hardware
