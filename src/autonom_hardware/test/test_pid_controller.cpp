#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "autonom_hardware/pid_controller.hpp"

using autonom_hardware::PidController;
using autonom_hardware::PidGains;

TEST(PidController, RejectsInvalidGains)
{
  EXPECT_THROW(PidController(PidGains{-1.0, 0.0, 0.0, 0.0, 1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(PidController(PidGains{1.0, 0.0, 0.0, 0.0, 1.0, 0.0}), std::invalid_argument);
}

TEST(PidController, FeedForwardOnlyWithoutValidDt)
{
  PidController pid(PidGains{1.0, 1.0, 0.0, 0.1, 1.0, 1.0});
  EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, 0.0), 0.5);
  EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, -1.0), 0.5);
}

TEST(PidController, OutputSaturates)
{
  PidController pid(PidGains{10.0, 0.0, 0.0, 0.0, 1.0, 0.8});
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, 0.02), 0.8);
  EXPECT_DOUBLE_EQ(pid.update(-1.0, 0.0, 0.02), -0.8);
}

TEST(PidController, IntegralDoesNotWindUpWhileSaturated)
{
  PidController pid(PidGains{1.0, 10.0, 0.0, 0.0, 10.0, 1.0});
  for (int i = 0; i < 1000; ++i) {
    pid.update(5.0, 0.0, 0.01);
  }
  // Without anti-windup the integral would sit at its 10.0 limit and keep the output saturated.
  EXPECT_LT(pid.update(0.0, 0.0, 0.01), 1.0);
}

TEST(PidController, ConvergesOnFirstOrderPlant)
{
  PidController pid(PidGains{0.05, 0.5, 0.0, 0.04, 0.5, 1.0});
  const double plant_gain = 20.0;
  const double time_constant = 0.1;
  const double dt = 0.02;
  double speed = 0.0;
  for (int i = 0; i < 500; ++i) {
    const double effort = pid.update(10.0, speed, dt);
    speed += (plant_gain * effort - speed) * dt / time_constant;
  }
  EXPECT_NEAR(speed, 10.0, 0.05);
}

TEST(PidController, NonFiniteInputResets)
{
  PidController pid(PidGains{1.0, 1.0, 0.0, 0.0, 1.0, 1.0});
  EXPECT_DOUBLE_EQ(pid.update(NAN, 0.0, 0.01), 0.0);
}

TEST(PidController, DirectionChangeClearsIntegral)
{
  // kp, ki, kd, kf, integral_limit, output_limit
  PidController pid(PidGains{0.0, 1.0, 0.0, 0.0, 1.0, 1.0});
  for (int i = 0; i < 50; ++i) {
    pid.update(5.0, 0.0, 0.02);  // wind the integral up forwards (stuck wheel)
  }
  ASSERT_GT(pid.update(5.0, 0.0, 0.02), 0.5);
  // Reversing must not keep pushing forwards with the old integral.
  EXPECT_LE(pid.update(-5.0, 0.0, 0.02), 0.0);
}
