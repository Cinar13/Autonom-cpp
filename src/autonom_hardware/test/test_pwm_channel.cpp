#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <system_error>

#include "autonom_hardware/pwm_channel.hpp"
#include "fake_pwm_chip.hpp"

using autonom_hardware::PwmChannel;
using fake_pwm::FakePwmChip;

TEST(PwmChannel, ExportsConfiguresAndEnables)
{
  FakePwmChip chip;
  {
    PwmChannel pwm(chip.path(), 1, 20000.0);
    ASSERT_TRUE(chip.exists(1));
    EXPECT_EQ(chip.readNumber(1, "period"), 50000);
    EXPECT_EQ(chip.readNumber(1, "duty_cycle"), 0);
    EXPECT_EQ(chip.readNumber(1, "enable"), 1);
  }
  EXPECT_TRUE(chip.waitUntilGone(1)) << "a channel we exported must be unexported again";
}

TEST(PwmChannel, DutyCycleIsAFractionOfThePeriodAndClamped)
{
  FakePwmChip chip;
  PwmChannel pwm(chip.path(), 0, 20000.0);
  EXPECT_TRUE(pwm.setDutyCycle(0.5));
  EXPECT_EQ(chip.readNumber(0, "duty_cycle"), 25000);
  EXPECT_TRUE(pwm.setDutyCycle(0.001));  // shorter value after a longer one
  EXPECT_EQ(chip.readNumber(0, "duty_cycle"), 50);
  EXPECT_TRUE(pwm.setDutyCycle(1.7));
  EXPECT_EQ(chip.readNumber(0, "duty_cycle"), 50000);
  EXPECT_TRUE(pwm.setDutyCycle(-0.3));
  EXPECT_EQ(chip.readNumber(0, "duty_cycle"), 0);
  EXPECT_TRUE(pwm.setDutyCycle(0.25));
  EXPECT_TRUE(pwm.setDutyCycle(std::nan("")));
  EXPECT_EQ(chip.readNumber(0, "duty_cycle"), 0);
}

TEST(PwmChannel, DestructorZeroesDutyAndDisablesButKeepsAForeignExport)
{
  FakePwmChip chip;
  chip.precreate(0);
  {
    PwmChannel pwm(chip.path(), 0, 20000.0);
    EXPECT_TRUE(pwm.setDutyCycle(0.8));
  }
  EXPECT_EQ(chip.readNumber(0, "duty_cycle"), 0);
  EXPECT_EQ(chip.readNumber(0, "enable"), 0);
  EXPECT_TRUE(chip.exists(0)) << "a channel exported by someone else must stay exported";
}

TEST(PwmChannel, RejectsBadFrequency)
{
  FakePwmChip chip;
  EXPECT_THROW(PwmChannel(chip.path(), 0, 0.0), std::invalid_argument);
  EXPECT_THROW(PwmChannel(chip.path(), 0, -5.0), std::invalid_argument);
  EXPECT_THROW(PwmChannel(chip.path(), 0, std::nan("")), std::invalid_argument);
  EXPECT_THROW(PwmChannel(chip.path(), 0, 1e10), std::invalid_argument);
}

TEST(PwmChannel, MissingChipThrows)
{
  EXPECT_THROW(PwmChannel("/nonexistent/pwmchip9", 0, 20000.0), std::system_error);
}

TEST(PwmChannel, ExportThatNeverAppearsTimesOut)
{
  FakePwmChip chip(false);  // nobody creates pwm0/ after the export write
  EXPECT_THROW(PwmChannel(chip.path(), 0, 20000.0), std::runtime_error);
}
