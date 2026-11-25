#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "autonom_hardware/encoder_model.hpp"

using autonom_hardware::angleToCounts;
using autonom_hardware::countsPerWheelRevolution;

TEST(EncoderModel, RobotMotorsGive1372CountsPerWheelTurn)
{
  // 98 CPR encoder on the back of the motor, 14:1 gearbox (430 rpm at the output).
  EXPECT_EQ(countsPerWheelRevolution(98.0, 14.0), 1372);
}

TEST(EncoderModel, RejectsInvalidEncoderOrGearbox)
{
  EXPECT_THROW(countsPerWheelRevolution(0.0, 14.0), std::invalid_argument);
  EXPECT_THROW(countsPerWheelRevolution(98.0, -14.0), std::invalid_argument);
  EXPECT_THROW(countsPerWheelRevolution(98.0, 14.3), std::invalid_argument);  // 1401.4 counts
}

TEST(EncoderModel, CountsWholeStepsOnly)
{
  const double step = 2.0 * M_PI / 1372.0;
  EXPECT_EQ(angleToCounts(0.0, 1372), 0);
  EXPECT_EQ(angleToCounts(0.99 * step, 1372), 0);
  EXPECT_EQ(angleToCounts(1.01 * step, 1372), 1);
  EXPECT_EQ(angleToCounts(2.0 * M_PI, 1372), 1372);
  EXPECT_EQ(angleToCounts(-0.5 * step, 1372), -1);  // backwards past the start
  EXPECT_EQ(angleToCounts(-10.0 * M_PI, 1372), -5 * 1372);
}

TEST(EncoderModel, ResolutionAtTheWheel)
{
  // 1372 counts on a 32.5 mm wheel: 0.149 mm of travel per count.
  EXPECT_NEAR(2.0 * M_PI * 0.0325 / 1372.0, 1.488e-4, 1e-7);
}
