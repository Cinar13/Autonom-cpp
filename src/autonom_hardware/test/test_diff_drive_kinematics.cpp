#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "autonom_hardware/diff_drive_kinematics.hpp"

using autonom_hardware::BodyMotion;
using autonom_hardware::DiffDriveKinematics;
using autonom_hardware::Pose2D;
using autonom_hardware::WheelPair;

TEST(DiffDriveKinematics, RejectsDegenerateGeometry)
{
  EXPECT_THROW(DiffDriveKinematics(0.0, 0.2), std::invalid_argument);
  EXPECT_THROW(DiffDriveKinematics(0.03, 0.0), std::invalid_argument);
  EXPECT_THROW(DiffDriveKinematics(-0.03, 0.2), std::invalid_argument);
  EXPECT_THROW(DiffDriveKinematics(NAN, 0.2), std::invalid_argument);
}

TEST(DiffDriveKinematics, RoundTrip)
{
  const DiffDriveKinematics kinematics(0.05, 0.2);
  const BodyMotion body{0.3, -0.7};
  const BodyMotion recovered = kinematics.toBody(kinematics.toWheels(body));
  EXPECT_NEAR(recovered.linear, body.linear, 1e-12);
  EXPECT_NEAR(recovered.angular, body.angular, 1e-12);
}

TEST(DiffDriveKinematics, PureRotationSpinsWheelsOpposite)
{
  const DiffDriveKinematics kinematics(0.05, 0.2);
  const WheelPair wheels = kinematics.toWheels({0.0, 1.0});
  EXPECT_NEAR(wheels.left, -2.0, 1e-12);
  EXPECT_NEAR(wheels.right, 2.0, 1e-12);
}

TEST(DiffDriveKinematics, LimitPreservesCurvature)
{
  const WheelPair limited = DiffDriveKinematics::limit({10.0, 20.0}, 10.0);
  EXPECT_DOUBLE_EQ(limited.right, 10.0);
  EXPECT_DOUBLE_EQ(limited.left, 5.0);
  const WheelPair zero = DiffDriveKinematics::limit({1.0, 1.0}, 0.0);
  EXPECT_DOUBLE_EQ(zero.left, 0.0);
  EXPECT_DOUBLE_EQ(zero.right, 0.0);
}

TEST(Pose2D, QuarterCircleArc)
{
  Pose2D pose;
  const double radius = 1.0;
  for (int i = 0; i < 100; ++i) {
    pose.integrate({radius * M_PI / 2.0 / 100.0, M_PI / 2.0 / 100.0});
  }
  EXPECT_NEAR(pose.x, 1.0, 1e-9);
  EXPECT_NEAR(pose.y, 1.0, 1e-9);
  EXPECT_NEAR(pose.theta, M_PI / 2.0, 1e-9);
}

TEST(Pose2D, HeadingWrapsAndIgnoresNonFinite)
{
  Pose2D pose;
  pose.integrate({0.0, 3.0 * M_PI / 2.0});
  EXPECT_NEAR(pose.theta, -M_PI / 2.0, 1e-12);
  pose.integrate({NAN, 0.1});
  EXPECT_NEAR(pose.theta, -M_PI / 2.0, 1e-12);
}
