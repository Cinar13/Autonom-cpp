#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "autonom_competitor/lane_follower.hpp"

using autonom_competitor::Command;
using autonom_competitor::LaneFollower;
using autonom_competitor::LaneFollowerSettings;
using autonom_competitor::Point;
using autonom_competitor::Pose2D;

namespace
{

// Counter-clockwise 4 m x 4 m square centred on the origin, a point every 0.1 m.
std::vector<Point> square()
{
  std::vector<Point> pts;
  for (int i = 0; i < 40; ++i) {pts.push_back({-2.0 + 0.1 * i, -2.0});}
  for (int i = 0; i < 40; ++i) {pts.push_back({2.0, -2.0 + 0.1 * i});}
  for (int i = 0; i < 40; ++i) {pts.push_back({2.0 - 0.1 * i, 2.0});}
  for (int i = 0; i < 40; ++i) {pts.push_back({-2.0, 2.0 - 0.1 * i});}
  return pts;
}

// Hairpin: out along y = 0, a U-turn, back along y = 0.3 (two legs 0.3 m apart, like corridors behind a wall).
std::vector<Point> hairpin()
{
  std::vector<Point> pts;
  for (int i = 0; i <= 30; ++i) {pts.push_back({0.1 * i, 0.0});}
  for (int k = 1; k < 10; ++k) {
    const double a = -M_PI / 2 + M_PI * k / 10;
    pts.push_back({3.0 + 0.15 * std::cos(a), 0.15 + 0.15 * std::sin(a)});
  }
  for (int i = 30; i >= 0; --i) {pts.push_back({0.1 * i, 0.3});}
  for (int k = 1; k < 10; ++k) {
    const double a = M_PI / 2 + M_PI * k / 10;
    pts.push_back({0.15 * std::cos(a), 0.15 + 0.15 * std::sin(a)});
  }
  return pts;
}

LaneFollowerSettings settings()
{
  LaneFollowerSettings s;
  s.speed = 0.3;
  s.lookahead = 0.3;
  s.max_angular = 2.0;
  s.merge_distance = 0.0;
  return s;
}

}  // namespace

TEST(LaneFollower, RejectsShortCentreLine)
{
  EXPECT_THROW(LaneFollower({{0, 0}, {1, 0}}, settings()), std::invalid_argument);
}

TEST(LaneFollower, LoopLength)
{
  LaneFollower f(square(), settings());
  EXPECT_NEAR(f.length(), 16.0, 1e-9);
}

TEST(LaneFollower, StraightOnTheCentreLine)
{
  LaneFollower f(square(), settings());
  const Command c = f.update({-1.0, -2.0, 0.0});
  EXPECT_NEAR(c.linear, 0.3, 1e-9);
  EXPECT_NEAR(c.angular, 0.0, 1e-6);
}

TEST(LaneFollower, LeftLaneSteersLeft)
{
  auto s = settings();
  s.lane_offset = 0.2;
  LaneFollower f(square(), s);
  EXPECT_GT(f.update({-1.0, -2.0, 0.0}).angular, 0.1);
  s.lane_offset = -0.2;
  LaneFollower g(square(), s);
  EXPECT_LT(g.update({-1.0, -2.0, 0.0}).angular, -0.1);
}

TEST(LaneFollower, LanePointIsLeftOfTravel)
{
  LaneFollower f(square(), settings());
  const Point p = f.lanePoint(1.0, 0.25);  // bottom edge, travelling +x: left is +y
  EXPECT_NEAR(p.x, -1.0, 1e-9);
  EXPECT_NEAR(p.y, -1.75, 1e-9);
  EXPECT_NEAR(f.lateralOffset({-1.0, -1.75}, 1.0), 0.25, 1e-9);
}

TEST(LaneFollower, StartOffsetBlendsIntoLane)
{
  // Placed 0.2 m left of the centre line with lane 0 and a 1 m merge: the first target keeps the start offset,
  // so the robot drives straight instead of cutting across at once.
  auto s = settings();
  s.merge_distance = 1.0;
  LaneFollower f(square(), s);
  const Command c = f.update({-1.0, -1.8, 0.0});
  EXPECT_NEAR(c.angular, 0.0, 1e-6);
}

TEST(LaneFollower, TurnsInPlaceWhenTheLaneIsBehind)
{
  LaneFollower f(square(), settings());
  const Command c = f.update({-1.0, -2.0, M_PI});
  EXPECT_DOUBLE_EQ(c.linear, 0.0);
  EXPECT_NEAR(std::abs(c.angular), 2.0, 1e-9);
}

TEST(LaneFollower, ProgressNeverJumpsToTheCorridorBehindTheWall)
{
  // Start on the outbound leg, then report a pose that is closer to the return leg (0.3 m away, far along the loop):
  // the forward search window keeps the robot on its own leg.
  LaneFollower f(hairpin(), settings());
  (void)f.update({1.0, 0.0, 0.0});
  const double before = f.progress();
  (void)f.update({1.1, 0.16, 0.0});
  EXPECT_GT(f.progress(), before);
  EXPECT_LT(f.progress(), before + 0.5);
}

TEST(LaneFollower, RobotAheadOnlyCountsTheCorridorInFront)
{
  using autonom_competitor::robotAhead;
  const Pose2D self{0.0, 0.0, M_PI / 2};  // facing +y
  EXPECT_TRUE(robotAhead(self, {{0.05, 0.3}}, 0.35, 0.18));    // ahead, slightly off-centre
  EXPECT_FALSE(robotAhead(self, {{0.0, 0.5}}, 0.35, 0.18));    // ahead but further than the distance
  EXPECT_FALSE(robotAhead(self, {{0.2, 0.05}}, 0.35, 0.18));   // beside (start line neighbour)
  EXPECT_FALSE(robotAhead(self, {{0.0, -0.2}}, 0.35, 0.18));   // behind
  EXPECT_FALSE(robotAhead(self, {}, 0.35, 0.18));
}

TEST(LaneFollower, DrivesFullLapsOnItsLane)
{
  // Closed loop with unicycle kinematics: two laps of the square on a 0.15 m left lane, starting 0.25 m off it.
  auto s = settings();
  s.lane_offset = 0.15;
  s.merge_distance = 1.0;
  LaneFollower f(square(), s);
  Pose2D pose{-1.0, -1.9, 0.0};
  const double dt = 0.05;
  double worst = 0.0;
  for (int k = 0; k < 4000 && f.travelled() < 2.0 * f.length(); ++k) {
    const Command c = f.update(pose);
    pose.x += c.linear * std::cos(pose.yaw) * dt;
    pose.y += c.linear * std::sin(pose.yaw) * dt;
    pose.yaw += c.angular * dt;
    if (f.travelled() > 2.0) {
      worst = std::max(worst, std::abs(f.lateralOffset({pose.x, pose.y}, f.progress()) - 0.15));
    }
  }
  EXPECT_GE(f.travelled(), 2.0 * f.length());
  EXPECT_LT(worst, 0.25);  // corners of a sharp square are cut; straights hold the lane
}
