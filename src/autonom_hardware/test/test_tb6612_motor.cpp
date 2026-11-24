#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <system_error>

#include "autonom_hardware/tb6612_motor.hpp"
#include "fake_gpiod.hpp"
#include "fake_pwm_chip.hpp"

using autonom_hardware::GpioChipPtr;
using autonom_hardware::openGpioChip;
using autonom_hardware::Tb6612Config;
using autonom_hardware::Tb6612Motor;
using fake_gpiod::Mode;

namespace
{

constexpr unsigned int kIn1 = 5;
constexpr unsigned int kIn2 = 6;

}  // namespace

class Tb6612MotorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fake_gpiod::reset();
    chip_ = openGpioChip("pinctrl-rp1");
    config_.pwm_chip_path = pwm_.path();
    config_.pwm_channel = 0;
    config_.in1_pin = kIn1;
    config_.in2_pin = kIn2;
  }

  int in1() const { return fake_gpiod::line(kIn1).value; }
  int in2() const { return fake_gpiod::line(kIn2).value; }
  long long duty() const { return pwm_.readNumber(0, "duty_cycle"); }

  fake_pwm::FakePwmChip pwm_;
  GpioChipPtr chip_;
  Tb6612Config config_;
};

TEST_F(Tb6612MotorTest, StartsCoastingWithBothInputsLow)
{
  Tb6612Motor motor(chip_, config_, "left");
  EXPECT_EQ(fake_gpiod::line(kIn1).mode, Mode::kOutput);
  EXPECT_EQ(fake_gpiod::line(kIn1).consumer, "left_in1");
  EXPECT_EQ(fake_gpiod::line(kIn2).consumer, "left_in2");
  EXPECT_EQ(in1(), 0);
  EXPECT_EQ(in2(), 0);
  EXPECT_EQ(duty(), 0);
  EXPECT_EQ(pwm_.readNumber(0, "enable"), 1);
}

TEST_F(Tb6612MotorTest, ForwardAndReverseFollowTheTruthTable)
{
  Tb6612Motor motor(chip_, config_, "left");
  ASSERT_TRUE(motor.setEffort(0.4));
  EXPECT_EQ(in1(), 1);
  EXPECT_EQ(in2(), 0);
  EXPECT_EQ(duty(), 20000);

  ASSERT_TRUE(motor.setEffort(-0.6));
  EXPECT_EQ(in1(), 0);
  EXPECT_EQ(in2(), 1);
  EXPECT_EQ(duty(), 30000) << "TB6612 duty is the magnitude in both directions (no complement)";
}

TEST_F(Tb6612MotorTest, InvertedSwapsDirectionOnly)
{
  config_.inverted = true;
  Tb6612Motor motor(chip_, config_, "right");
  ASSERT_TRUE(motor.setEffort(0.5));
  EXPECT_EQ(in1(), 0);
  EXPECT_EQ(in2(), 1);
  EXPECT_EQ(duty(), 25000);
}

TEST_F(Tb6612MotorTest, EffortIsClamped)
{
  Tb6612Motor motor(chip_, config_, "left");
  ASSERT_TRUE(motor.setEffort(3.0));
  EXPECT_EQ(duty(), 50000);
  ASSERT_TRUE(motor.setEffort(-3.0));
  EXPECT_EQ(duty(), 50000);
  EXPECT_EQ(in2(), 1);
}

TEST_F(Tb6612MotorTest, ZeroAndNonFiniteEffortCoast)
{
  Tb6612Motor motor(chip_, config_, "left");
  for (double effort : {0.0, std::nan(""), double{INFINITY}, -double{INFINITY}}) {
    ASSERT_TRUE(motor.setEffort(0.7));
    ASSERT_TRUE(motor.setEffort(effort));
    EXPECT_EQ(in1(), 0) << effort;
    EXPECT_EQ(in2(), 0) << effort;
    EXPECT_EQ(duty(), 0) << effort;
  }
}

TEST_F(Tb6612MotorTest, DirectionChangeNeverRaisesBothInputs)
{
  Tb6612Motor motor(chip_, config_, "left");
  fake_gpiod::clearWrites();
  for (double effort : {0.5, -0.5, 0.3, -1.0, 1.0, 0.0, -0.2}) {
    ASSERT_TRUE(motor.setEffort(effort));
  }
  std::map<unsigned int, int> level{{kIn1, 0}, {kIn2, 0}};
  for (const auto & write : fake_gpiod::writes()) {
    level[write.offset] = write.value;
    EXPECT_FALSE(level[kIn1] == 1 && level[kIn2] == 1) << "IN1 and IN2 high together (short brake glitch)";
  }
}

TEST_F(Tb6612MotorTest, WriteFailureIsReported)
{
  Tb6612Motor motor(chip_, config_, "left");
  fake_gpiod::failIo(true);
  EXPECT_FALSE(motor.setEffort(0.5));
  fake_gpiod::failIo(false);
  EXPECT_TRUE(motor.setEffort(0.5));  // retried: the failed write must not be cached as done
  EXPECT_EQ(in1(), 1);
  fake_gpiod::failIo(true);
  EXPECT_FALSE(motor.coast());
  fake_gpiod::failIo(false);
  EXPECT_TRUE(motor.coast());
  EXPECT_EQ(in1(), 0);
}

TEST_F(Tb6612MotorTest, DestructionStopsPwmAndReleasesBothPinsLow)
{
  {
    Tb6612Motor motor(chip_, config_, "left");
    ASSERT_TRUE(motor.setEffort(-0.9));
  }
  EXPECT_EQ(duty(), 0);
  EXPECT_TRUE(pwm_.waitUntilGone(0)) << "channel exported by the motor is unexported again";
  EXPECT_EQ(fake_gpiod::line(kIn1).mode, Mode::kFree);
  EXPECT_EQ(fake_gpiod::line(kIn2).mode, Mode::kFree);
  EXPECT_EQ(in1(), 0);
  EXPECT_EQ(in2(), 0);
}

TEST_F(Tb6612MotorTest, PwmFailureLeavesDirectionPinsLowAndReleased)
{
  config_.pwm_chip_path = "/nonexistent/pwmchip0";
  EXPECT_THROW(Tb6612Motor(chip_, config_, "left"), std::system_error);
  EXPECT_EQ(fake_gpiod::line(kIn1).mode, Mode::kFree);
  EXPECT_EQ(fake_gpiod::line(kIn2).mode, Mode::kFree);
}

TEST_F(Tb6612MotorTest, DirectionPinConflictThrows)
{
  config_.in2_pin = kIn1;
  EXPECT_THROW(Tb6612Motor(chip_, config_, "left"), std::system_error);
}
