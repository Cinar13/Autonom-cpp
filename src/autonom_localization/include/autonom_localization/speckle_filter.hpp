#pragma once

#include <cstddef>
#include <vector>

namespace autonom_localization
{

struct SpeckleFilterSettings
{
  /// Beams on each side of a return that are inspected for support.
  std::size_t window{3};
  /// Supporting neighbours needed for a return to count as a real surface.
  std::size_t min_neighbors{2};
  /// A neighbour supports a return when |r_n - r| <= abs_tolerance + rel_tolerance * r.
  float abs_tolerance{0.05F};
  float rel_tolerance{0.02F};
};

/// Indices of returns without enough nearby returns at a similar range: single-beam noise, dust or edge
/// mixels. Nav2's obstacle layer marks every return as lethal, so one such point blocks a corridor.
/// Invalid ranges (non-finite or outside [range_min, range_max]) never count as support and are not reported.
std::vector<std::size_t> findSpeckles(
  const std::vector<float> & ranges, float range_min, float range_max, const SpeckleFilterSettings & settings);

}  // namespace autonom_localization
