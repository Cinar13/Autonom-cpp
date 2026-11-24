#include <gtest/gtest.h>

#include <system_error>

#include "autonom_hardware/gpio.hpp"
#include "fake_gpiod.hpp"

using autonom_hardware::GpioInput;
using autonom_hardware::GpioOutput;
using autonom_hardware::openGpioChip;
using fake_gpiod::Mode;

class GpioTest : public ::testing::Test
{
protected:
  void SetUp() override { fake_gpiod::reset(); }
};

TEST_F(GpioTest, OpensRp1ByLabelAndClosesOnLastReference)
{
  {
    auto chip = openGpioChip("pinctrl-rp1");
    auto copy = chip;
    EXPECT_EQ(fake_gpiod::openChips(), 1);
  }
  EXPECT_EQ(fake_gpiod::openChips(), 0);
}

TEST_F(GpioTest, UnknownChipThrows)
{
  EXPECT_THROW(openGpioChip("no-such-chip"), std::system_error);
}

TEST_F(GpioTest, OutputStartsAtInitialValueAndTracksSet)
{
  auto chip = openGpioChip("pinctrl-rp1");
  GpioOutput out(chip, 26, "test", true);
  EXPECT_EQ(fake_gpiod::line(26).mode, Mode::kOutput);
  EXPECT_EQ(fake_gpiod::line(26).consumer, "test");
  EXPECT_EQ(fake_gpiod::line(26).value, 1);
  EXPECT_TRUE(out.set(false));
  EXPECT_EQ(fake_gpiod::line(26).value, 0);
}

TEST_F(GpioTest, RepeatedSetDoesNotTouchTheLine)
{
  auto chip = openGpioChip("pinctrl-rp1");
  GpioOutput out(chip, 5, "test", false);
  fake_gpiod::clearWrites();
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(out.set(true));
  }
  EXPECT_EQ(fake_gpiod::writes().size(), 1u);
}

TEST_F(GpioTest, DestructorDrivesLowThenReleases)
{
  auto chip = openGpioChip("pinctrl-rp1");
  {
    GpioOutput out(chip, 20, "test", true);
    fake_gpiod::clearWrites();
  }
  const auto writes = fake_gpiod::writes();
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].offset, 20u);
  EXPECT_EQ(writes[0].value, 0);
  EXPECT_EQ(fake_gpiod::line(20).mode, Mode::kFree);
}

TEST_F(GpioTest, SecondRequestForTheSamePinFails)
{
  auto chip_a = openGpioChip("pinctrl-rp1");
  auto chip_b = openGpioChip("pinctrl-rp1");
  GpioOutput first(chip_a, 6, "first", false);
  EXPECT_THROW(GpioOutput(chip_b, 6, "second", false), std::system_error);
  EXPECT_THROW(GpioInput(chip_b, 6, "second", true), std::system_error);
  EXPECT_EQ(fake_gpiod::line(6).consumer, "first");
}

TEST_F(GpioTest, PinOutsideTheChipThrows)
{
  auto chip = openGpioChip("pinctrl-rp1");
  EXPECT_THROW(GpioOutput(chip, 64, "test", false), std::system_error);
  EXPECT_THROW(GpioInput(chip, 64, "test", false), std::system_error);
}

TEST_F(GpioTest, FailedWriteReportsFalseAndIsRetried)
{
  auto chip = openGpioChip("pinctrl-rp1");
  GpioOutput out(chip, 21, "test", false);
  fake_gpiod::failIo(true);
  EXPECT_FALSE(out.set(true));
  fake_gpiod::failIo(false);
  EXPECT_TRUE(out.set(true));  // the cached value must not have changed on failure
  EXPECT_EQ(fake_gpiod::line(21).value, 1);
}

TEST_F(GpioTest, InputWithPullUpReadsHighUntilPulledLow)
{
  auto chip = openGpioChip("pinctrl-rp1");
  GpioInput in(chip, 16, "button", true);
  EXPECT_EQ(fake_gpiod::line(16).mode, Mode::kInput);
  EXPECT_TRUE(fake_gpiod::line(16).pull_up);
  EXPECT_EQ(in.read(), 1);
  fake_gpiod::setInputLevel(16, 0);
  EXPECT_EQ(in.read(), 0);
  fake_gpiod::failIo(true);
  EXPECT_EQ(in.read(), -1);
}

TEST_F(GpioTest, InputIsReleasedOnDestruction)
{
  auto chip = openGpioChip("pinctrl-rp1");
  {
    GpioInput in(chip, 16, "button", false);
    EXPECT_FALSE(fake_gpiod::line(16).pull_up);
  }
  EXPECT_EQ(fake_gpiod::line(16).mode, Mode::kFree);
}
