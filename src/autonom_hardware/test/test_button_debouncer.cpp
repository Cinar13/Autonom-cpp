#include <gtest/gtest.h>

#include "autonom_hardware/button_debouncer.hpp"

using autonom_hardware::ButtonDebouncer;

namespace
{

int countPresses(ButtonDebouncer & debouncer, bool pressed, int samples)
{
  int presses = 0;
  for (int i = 0; i < samples; ++i) {
    presses += debouncer.update(pressed) ? 1 : 0;
  }
  return presses;
}

}  // namespace

TEST(ButtonDebouncer, CleanPressFiresOnce)
{
  ButtonDebouncer debouncer(3);
  EXPECT_EQ(countPresses(debouncer, false, 5), 0);
  EXPECT_EQ(countPresses(debouncer, true, 10), 1);
  EXPECT_EQ(countPresses(debouncer, false, 5), 0);
  EXPECT_EQ(countPresses(debouncer, true, 5), 1);
}

TEST(ButtonDebouncer, BounceShorterThanWindowIsIgnored)
{
  ButtonDebouncer debouncer(3);
  countPresses(debouncer, false, 5);
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(debouncer.update(true));
    EXPECT_FALSE(debouncer.update(false));
  }
}

TEST(ButtonDebouncer, BouncyPressFiresOnce)
{
  ButtonDebouncer debouncer(3);
  countPresses(debouncer, false, 5);
  int presses = 0;
  for (bool sample : {true, false, true, true, false, true, true, true, true, false, true, true, true}) {
    presses += debouncer.update(sample) ? 1 : 0;
  }
  EXPECT_EQ(presses, 1);
}

TEST(ButtonDebouncer, HeldAtStartupNeverFiresUntilReleased)
{
  ButtonDebouncer debouncer(3);
  EXPECT_EQ(countPresses(debouncer, true, 20), 0);
  EXPECT_FALSE(debouncer.armed());
  EXPECT_EQ(countPresses(debouncer, false, 3), 0);
  EXPECT_TRUE(debouncer.armed());
  EXPECT_EQ(countPresses(debouncer, true, 3), 1);
}

TEST(ButtonDebouncer, RejectsZeroWindow)
{
  EXPECT_THROW(ButtonDebouncer(0), std::invalid_argument);
}
