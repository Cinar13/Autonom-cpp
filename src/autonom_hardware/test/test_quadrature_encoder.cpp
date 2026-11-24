#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <stdexcept>
#include <system_error>
#include <thread>

#include "autonom_hardware/quadrature_encoder.hpp"
#include "fake_gpiod.hpp"

using autonom_hardware::GpioChipPtr;
using autonom_hardware::openGpioChip;
using autonom_hardware::QuadratureEncoder;
using autonom_hardware::QuadratureEncoderConfig;
using fake_gpiod::Mode;

namespace
{

constexpr unsigned int kA = 17;
constexpr unsigned int kB = 27;

bool waitFor(const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

}  // namespace

class QuadratureEncoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fake_gpiod::reset();
    chip_ = openGpioChip("pinctrl-rp1");
    config_.pin_a = kA;
    config_.pin_b = kB;
    // Lines idle low at start-up.
    fake_gpiod::setInputLevel(kA, 0);
    fake_gpiod::setInputLevel(kB, 0);
  }

  // The decoder counts B leading A as positive: 00 -> 01 -> 11 -> 10 -> 00 is +4.
  void forwardCycle()
  {
    fake_gpiod::pushEdge(kB, true, next());
    fake_gpiod::pushEdge(kA, true, next());
    fake_gpiod::pushEdge(kB, false, next());
    fake_gpiod::pushEdge(kA, false, next());
  }
  void reverseCycle()
  {
    fake_gpiod::pushEdge(kA, true, next());
    fake_gpiod::pushEdge(kB, true, next());
    fake_gpiod::pushEdge(kA, false, next());
    fake_gpiod::pushEdge(kB, false, next());
  }
  std::int64_t next() { return time_ns_ += 1000; }

  GpioChipPtr chip_;
  QuadratureEncoderConfig config_;
  std::int64_t time_ns_{1'000'000'000};
};

TEST_F(QuadratureEncoderTest, RequestsBothLinesForEdgesWithPullUp)
{
  QuadratureEncoder encoder(chip_, config_, "left_encoder");
  EXPECT_EQ(fake_gpiod::line(kA).mode, Mode::kEdgeEvents);
  EXPECT_EQ(fake_gpiod::line(kB).mode, Mode::kEdgeEvents);
  EXPECT_TRUE(fake_gpiod::line(kA).pull_up);
  EXPECT_EQ(encoder.count(), 0);
  EXPECT_TRUE(encoder.healthy());
}

TEST_F(QuadratureEncoderTest, CountsFourTicksPerCycleInBothDirections)
{
  QuadratureEncoder encoder(chip_, config_, "left_encoder");
  for (int i = 0; i < 5; ++i) {
    forwardCycle();
  }
  ASSERT_TRUE(waitFor([&] { return encoder.count() == 20; })) << encoder.count();
  for (int i = 0; i < 2; ++i) {
    reverseCycle();
  }
  ASSERT_TRUE(waitFor([&] { return encoder.count() == 12; })) << encoder.count();
  EXPECT_EQ(encoder.missedTransitions(), 0u);
}

TEST_F(QuadratureEncoderTest, InvertedCountsNegative)
{
  config_.inverted = true;
  QuadratureEncoder encoder(chip_, config_, "right_encoder");
  forwardCycle();
  ASSERT_TRUE(waitFor([&] { return encoder.count() == -4; })) << encoder.count();
}

TEST_F(QuadratureEncoderTest, StartsFromTheCurrentLineLevels)
{
  fake_gpiod::setInputLevel(kA, 1);
  fake_gpiod::setInputLevel(kB, 1);
  QuadratureEncoder encoder(chip_, config_, "left_encoder");
  // From 11, forward continues with B falling then A falling.
  fake_gpiod::pushEdge(kB, false, next());
  fake_gpiod::pushEdge(kA, false, next());
  ASSERT_TRUE(waitFor([&] { return encoder.count() == 2; })) << encoder.count();
  EXPECT_EQ(encoder.missedTransitions(), 0u);
}

TEST_F(QuadratureEncoderTest, EdgesQueuedOnBothLinesAreReplayedInTimestampOrder)
{
  QuadratureEncoder encoder(chip_, config_, "left_encoder");
  // A's edges are queued before B's, so draining line by line yields A, A, B, B. By timestamp it is
  // B rise, A rise, B fall, A fall: one forward cycle. Decoding in queue order would instead miss edges.
  fake_gpiod::pushEdges({{kA, true, 2000}, {kA, false, 4000}, {kB, true, 1000}, {kB, false, 3000}});
  ASSERT_TRUE(waitFor([&] { return encoder.count() == 4; })) << encoder.count();
  EXPECT_EQ(encoder.missedTransitions(), 0u);
}

TEST_F(QuadratureEncoderTest, RepeatedLevelCountsAsMissedEdge)
{
  QuadratureEncoder encoder(chip_, config_, "left_encoder");
  fake_gpiod::pushEdge(kB, true, next());
  fake_gpiod::pushEdge(kB, true, next());  // the falling edge in between was lost
  ASSERT_TRUE(waitFor([&] { return encoder.missedTransitions() == 1; }));
  EXPECT_EQ(encoder.count(), 1);
}

TEST_F(QuadratureEncoderTest, WaitFailureMarksUnhealthy)
{
  QuadratureEncoder encoder(chip_, config_, "left_encoder");
  fake_gpiod::failNextWait();
  ASSERT_TRUE(waitFor([&] { return !encoder.healthy(); }));
}

TEST_F(QuadratureEncoderTest, ReleasesLinesOnDestruction)
{
  { QuadratureEncoder encoder(chip_, config_, "left_encoder"); }
  EXPECT_EQ(fake_gpiod::line(kA).mode, Mode::kFree);
  EXPECT_EQ(fake_gpiod::line(kB).mode, Mode::kFree);
}

TEST_F(QuadratureEncoderTest, SamePinForBothChannelsThrows)
{
  config_.pin_b = kA;
  EXPECT_THROW(QuadratureEncoder(chip_, config_, "left_encoder"), std::invalid_argument);
}

TEST_F(QuadratureEncoderTest, BusySecondLineReleasesTheFirst)
{
  autonom_hardware::GpioOutput squatter(chip_, kB, "other", false);
  EXPECT_THROW(QuadratureEncoder(chip_, config_, "left_encoder"), std::system_error);
  EXPECT_EQ(fake_gpiod::line(kA).mode, Mode::kFree) << "line A leaked after B failed";
}

TEST_F(QuadratureEncoderTest, InitialReadFailureReleasesBothLines)
{
  fake_gpiod::failIo(true);
  EXPECT_THROW(QuadratureEncoder(chip_, config_, "left_encoder"), std::system_error);
  fake_gpiod::failIo(false);
  EXPECT_EQ(fake_gpiod::line(kA).mode, Mode::kFree);
  EXPECT_EQ(fake_gpiod::line(kB).mode, Mode::kFree);
}
