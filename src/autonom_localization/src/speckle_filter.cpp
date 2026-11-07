#include "autonom_localization/speckle_filter.hpp"

#include <algorithm>
#include <cmath>

namespace autonom_localization
{
namespace
{

bool isValid(float range, float range_min, float range_max)
{
  return std::isfinite(range) && range >= range_min && range <= range_max;
}

}  // namespace

std::vector<std::size_t> findSpeckles(
  const std::vector<float> & ranges, float range_min, float range_max, const SpeckleFilterSettings & settings)
{
  std::vector<std::size_t> speckles;
  const std::size_t count = ranges.size();
  for (std::size_t i = 0; i < count; ++i) {
    const float range = ranges[i];
    if (!isValid(range, range_min, range_max)) {
      continue;
    }
    const float tolerance = settings.abs_tolerance + settings.rel_tolerance * range;
    const std::size_t first = i >= settings.window ? i - settings.window : 0;
    const std::size_t last = std::min(count - 1, i + settings.window);
    std::size_t support = 0;
    for (std::size_t j = first; j <= last && support < settings.min_neighbors; ++j) {
      if (j != i && isValid(ranges[j], range_min, range_max) && std::fabs(ranges[j] - range) <= tolerance) {
        ++support;
      }
    }
    if (support < settings.min_neighbors) {
      speckles.push_back(i);
    }
  }
  return speckles;
}

}  // namespace autonom_localization
