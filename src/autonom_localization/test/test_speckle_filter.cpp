#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "autonom_localization/speckle_filter.hpp"

using autonom_localization::findSpeckles;
using autonom_localization::SpeckleFilterSettings;

namespace
{

constexpr float kMin = 0.05F;
constexpr float kMax = 40.0F;
const float kNone = std::numeric_limits<float>::infinity();

}  // namespace

TEST(SpeckleFilter, KeepsContinuousWall)
{
  std::vector<float> wall(50);
  for (std::size_t i = 0; i < wall.size(); ++i) {
    wall[i] = 1.0F + 0.004F * static_cast<float>(i);
  }
  EXPECT_TRUE(findSpeckles(wall, kMin, kMax, SpeckleFilterSettings{}).empty());
}

TEST(SpeckleFilter, RemovesSingleReturnInFreeSpace)
{
  std::vector<float> ranges(20, kNone);
  ranges[10] = 0.6F;
  const auto speckles = findSpeckles(ranges, kMin, kMax, SpeckleFilterSettings{});
  ASSERT_EQ(speckles.size(), 1U);
  EXPECT_EQ(speckles[0], 10U);
}

TEST(SpeckleFilter, RemovesSingleShortReturnInFrontOfWall)
{
  std::vector<float> ranges(20, 2.0F);
  ranges[7] = 0.5F;
  const auto speckles = findSpeckles(ranges, kMin, kMax, SpeckleFilterSettings{});
  ASSERT_EQ(speckles.size(), 1U);
  EXPECT_EQ(speckles[0], 7U);
}

TEST(SpeckleFilter, KeepsSmallObjectSpanningSeveralBeams)
{
  std::vector<float> ranges(20, kNone);
  for (std::size_t i = 8; i <= 10; ++i) {
    ranges[i] = 1.5F;
  }
  EXPECT_TRUE(findSpeckles(ranges, kMin, kMax, SpeckleFilterSettings{}).empty());
}

TEST(SpeckleFilter, HandlesEdgesAndInvalidRanges)
{
  std::vector<float> ranges{1.0F, 1.0F, 1.0F, std::nanf(""), 0.01F, 50.0F};
  EXPECT_TRUE(findSpeckles(ranges, kMin, kMax, SpeckleFilterSettings{}).empty());
  EXPECT_TRUE(findSpeckles({}, kMin, kMax, SpeckleFilterSettings{}).empty());
}
