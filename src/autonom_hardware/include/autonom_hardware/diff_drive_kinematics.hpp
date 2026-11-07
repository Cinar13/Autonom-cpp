#pragma once

namespace autonom_hardware
{

/// Wheel angular quantities (rad or rad/s).
struct WheelPair
{
  double left{0.0};
  double right{0.0};
};

/// Body-frame linear (m or m/s) and angular (rad or rad/s) quantities.
struct BodyMotion
{
  double linear{0.0};
  double angular{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  /// Exact arc integration of a body-frame displacement; theta stays in (-pi, pi].
  void integrate(const BodyMotion & displacement) noexcept;
};

class DiffDriveKinematics
{
public:
  /// Throws std::invalid_argument for non-positive or non-finite geometry, preventing divisions by zero.
  DiffDriveKinematics(double wheel_radius, double wheel_separation);

  WheelPair toWheels(const BodyMotion & body) const noexcept;
  BodyMotion toBody(const WheelPair & wheels) const noexcept;

  /// Scales both wheels uniformly so neither exceeds `max_wheel_speed`, preserving path curvature.
  static WheelPair limit(const WheelPair & wheels, double max_wheel_speed) noexcept;

private:
  double wheel_radius_;
  double wheel_separation_;
};

}  // namespace autonom_hardware
