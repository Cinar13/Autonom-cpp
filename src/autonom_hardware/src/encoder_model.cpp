#include "autonom_hardware/encoder_model.hpp"

#include <cmath>
#include <stdexcept>

namespace autonom_hardware
{

std::int64_t countsPerWheelRevolution(double motor_counts_per_revolution, double gear_ratio)
{
  if (!(motor_counts_per_revolution > 0.0) || !(gear_ratio > 0.0) || !std::isfinite(motor_counts_per_revolution) ||
    !std::isfinite(gear_ratio))
  {
    throw std::invalid_argument("encoder counts per revolution and gear ratio must be positive and finite");
  }
  const double counts = motor_counts_per_revolution * gear_ratio;
  const double rounded = std::round(counts);
  if (std::abs(counts - rounded) > 1e-6) {
    throw std::invalid_argument("encoder counts per revolution x gear ratio must be a whole number of counts");
  }
  return static_cast<std::int64_t>(rounded);
}

std::int64_t angleToCounts(double angle, std::int64_t counts_per_revolution)
{
  if (counts_per_revolution <= 0) {
    throw std::invalid_argument("counts_per_revolution must be positive");
  }
  return static_cast<std::int64_t>(std::floor(angle * static_cast<double>(counts_per_revolution) / (2.0 * M_PI)));
}

}  // namespace autonom_hardware
