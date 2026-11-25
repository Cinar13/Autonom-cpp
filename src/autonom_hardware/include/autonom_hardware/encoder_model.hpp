#pragma once

#include <cstdint>

namespace autonom_hardware
{

/// Counts per wheel revolution of a motor-shaft encoder behind a gearbox: the encoder turns `gear_ratio` times
/// per wheel turn. Throws std::invalid_argument for non-positive values or a non-integer product.
/// The robot's motors: N20, 28 CPR at the motor (7 PPR x4), 1:50 -> 1400 counts per wheel revolution.
std::int64_t countsPerWheelRevolution(double motor_counts_per_revolution, double gear_ratio);

/// Encoder count an ideal quadrature encoder reports at wheel angle `angle` [rad] (counts of whole steps,
/// rounding towards negative infinity like a counter that starts at 0 at angle 0).
std::int64_t angleToCounts(double angle, std::int64_t counts_per_revolution);

}  // namespace autonom_hardware
