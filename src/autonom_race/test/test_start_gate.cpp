#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "autonom_race/start_gate.hpp"

using autonom_race::RaceState;
using autonom_race::StartGate;
using std::chrono::milliseconds;
using std::chrono::seconds;

namespace
{

StartGate competitionGate() { return StartGate(seconds(5)); }

}  // namespace

TEST(StartGate, BlocksMotionUntilStarted)
{
  auto gate = competitionGate();
  EXPECT_EQ(gate.update(seconds(100)), RaceState::kIdle);
  EXPECT_FALSE(gate.motionAllowed());
}

TEST(StartGate, ReleasesExactlyAfterDelay)
{
  auto gate = competitionGate();
  ASSERT_TRUE(gate.requestStart(seconds(10)));
  EXPECT_EQ(gate.update(seconds(10)), RaceState::kCountdown);
  EXPECT_EQ(gate.update(seconds(15) - milliseconds(1)), RaceState::kCountdown);
  EXPECT_FALSE(gate.motionAllowed());
  EXPECT_EQ(gate.remaining(seconds(14)), seconds(1));
  EXPECT_EQ(gate.update(seconds(15)), RaceState::kRacing);
  EXPECT_TRUE(gate.motionAllowed());
  EXPECT_EQ(gate.remaining(seconds(16)), seconds(0));
}

TEST(StartGate, RepeatedStartDoesNotRestartCountdown)
{
  auto gate = competitionGate();
  ASSERT_TRUE(gate.requestStart(seconds(0)));
  EXPECT_FALSE(gate.requestStart(seconds(3)));
  EXPECT_EQ(gate.update(seconds(5)), RaceState::kRacing);
  EXPECT_FALSE(gate.requestStart(seconds(6)));
  EXPECT_EQ(gate.state(), RaceState::kRacing);
}

TEST(StartGate, StopBlocksAndRestartNeedsFullDelay)
{
  auto gate = competitionGate();
  EXPECT_FALSE(gate.requestStop());
  ASSERT_TRUE(gate.requestStart(seconds(0)));
  gate.update(seconds(5));
  ASSERT_TRUE(gate.requestStop());
  EXPECT_FALSE(gate.motionAllowed());
  EXPECT_FALSE(gate.requestStop());

  ASSERT_TRUE(gate.requestStart(seconds(20)));
  EXPECT_EQ(gate.update(seconds(24)), RaceState::kCountdown);
  EXPECT_EQ(gate.update(seconds(25)), RaceState::kRacing);
}

TEST(StartGate, StopDuringCountdownCancelsIt)
{
  auto gate = competitionGate();
  ASSERT_TRUE(gate.requestStart(seconds(0)));
  ASSERT_TRUE(gate.requestStop());
  EXPECT_EQ(gate.update(seconds(10)), RaceState::kStopped);
}

TEST(StartGate, BackwardClockJumpRestartsCountdown)
{
  auto gate = competitionGate();
  ASSERT_TRUE(gate.requestStart(seconds(100)));
  EXPECT_EQ(gate.update(seconds(2)), RaceState::kCountdown);
  EXPECT_EQ(gate.update(seconds(6)), RaceState::kCountdown);
  EXPECT_EQ(gate.update(seconds(7)), RaceState::kRacing);
}

TEST(StartGate, RejectsNegativeDelay)
{
  EXPECT_THROW(StartGate(seconds(-1)), std::invalid_argument);
  StartGate immediate(seconds(0));
  ASSERT_TRUE(immediate.requestStart(seconds(1)));
  EXPECT_EQ(immediate.update(seconds(1)), RaceState::kRacing);
}
