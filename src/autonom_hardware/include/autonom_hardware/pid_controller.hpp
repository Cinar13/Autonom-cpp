#pragma once

namespace autonom_hardware
{

struct PidGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  /// Velocity feed-forward: output contribution per unit of setpoint.
  double kf{0.0};
  /// Bound on the integral term, in output units.
  double integral_limit{1.0};
  /// Symmetric output saturation.
  double output_limit{1.0};
  /// Static-friction feed-forward: output added in the setpoint's direction whenever the setpoint is non-zero, so
  /// small commands start the gearmotor instead of waiting for the integral to wind up against static friction.
  double ks{0.0};
};

/// PID with feed-forward, derivative-on-measurement and conditional-integration anti-windup.
class PidController
{
public:
  explicit PidController(const PidGains & gains);

  /// Returns the saturated control output. Non-positive or non-finite `dt` yields feed-forward only.
  double update(double setpoint, double measurement, double dt) noexcept;
  void reset() noexcept;
  void setGains(const PidGains & gains);
  const PidGains & gains() const noexcept { return gains_; }

private:
  static void validate(const PidGains & gains);

  PidGains gains_;
  double integral_{0.0};
  double previous_measurement_{0.0};
  bool has_previous_measurement_{false};
  int previous_setpoint_sign_{0};
};

}  // namespace autonom_hardware
