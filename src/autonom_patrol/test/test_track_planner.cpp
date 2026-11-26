#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "autonom_patrol/track_planner.hpp"

using namespace autonom_patrol;  // NOLINT

namespace
{

/// 200 x 200 cells, 0.05 m, origin (-5, -5): the 10 x 10 m test area of the original Python tests.
Grid makeGrid(int width = 200, int height = 200, std::int8_t fill_value = 0, double res = 0.05, double ox = -5.0,
  double oy = -5.0)
{
  Grid g;
  g.width = width;
  g.height = height;
  g.resolution = res;
  g.origin_x = ox;
  g.origin_y = oy;
  g.data.assign(static_cast<std::size_t>(width) * height, fill_value);
  return g;
}

/// Python `grid[r0:r1, c0:c1] = value`.
void fill(Grid & g, int r0, int r1, int c0, int c1, std::int8_t value)
{
  for (int r = std::max(r0, 0); r < std::min(r1, g.height); ++r) {
    for (int c = std::max(c0, 0); c < std::min(c1, g.width); ++c) {
      g.data[g.index(r, c)] = value;
    }
  }
}

Grid room(int size = 200, int wall = 2)
{
  Grid g = makeGrid(size, size);
  fill(g, 0, wall, 0, size, 100);
  fill(g, size - wall, size, 0, size, 100);
  fill(g, 0, size, 0, wall, 100);
  fill(g, 0, size, size - wall, size, 100);
  return g;
}

/// Square track: 10 m outer wall, 6 m inner block, 2 m wide corridor (centre line 4 m from the middle).
Grid ring()
{
  Grid g = room();
  fill(g, 40, 160, 40, 160, 100);
  return g;
}

Grid lCorridor()
{
  Grid g = makeGrid(200, 200, -1);
  fill(g, 90, 110, 40, 160, 0);
  fill(g, 90, 180, 140, 160, 0);
  return g;
}

/// The race map (Fibonacci-Autonomous-Map): 2.49 x 2.10 m box, U island open to the top, divider from the top
/// wall into the U's opening. Walls 2 cells (5 cm) thick, 0.025 m cells.
Grid fibonacciArena()
{
  Grid g = makeGrid(120, 100, 100, 0.025, -1.5, -1.25);
  const auto cell = [&g](double x, double y) {
    return std::pair<int, int>{static_cast<int>((y - g.origin_y) / 0.025), static_cast<int>((x - g.origin_x) / 0.025)};
  };
  const auto [r0, c0] = cell(-1.245, -1.05);
  const auto [r1, c1] = cell(1.245, 1.05);
  fill(g, r0 + 2, r1 - 1, c0 + 2, c1 - 1, 0);
  const auto wall = [&](double x0, double y0, double x1, double y1) {
    const auto [ra, ca] = cell(std::min(x0, x1), std::min(y0, y1));
    const auto [rb, cb] = cell(std::max(x0, x1), std::max(y0, y1));
    fill(g, ra - 1, rb + 1, ca - 1, cb + 1, 100);
  };
  wall(-0.62, -0.42, 0.62, -0.42);  // U bottom
  wall(-0.62, -0.42, -0.62, 0.42);  // U left arm
  wall(0.62, -0.42, 0.62, 0.42);    // U right arm
  wall(0.0, 0.21, 0.0, 1.05);       // divider from the top wall
  return g;
}

LoopSettings ringSettings()
{
  LoopSettings s;
  s.clearance = 0.3;
  s.waypoint_spacing = 1.0;
  return s;
}

LoopSettings arenaLoopSettings()
{
  LoopSettings s;
  s.clearance = 0.15;
  s.waypoint_spacing = 0.4;
  s.min_loop_length = 2.0;
  s.cut_offset = 0.2;
  s.cut_half_width = 0.6;
  return s;
}

std::vector<int> crossings(StartLineMonitor & monitor, const std::vector<Point> & path)
{
  std::vector<int> out;
  for (const auto & p : path) {
    if (const int c = monitor.update(p)) {
      out.push_back(c);
    }
  }
  return out;
}

int stepsTo(const Mask & free, int w, int h, Cell seed, Cell target)
{
  return drivingDistance(free, w, h, seed)[static_cast<std::size_t>(target.row) * w + target.col];
}

}  // namespace

TEST(TrackPlanner, ClearanceKeepsUnsafeCellsOut)
{
  const Grid g = room();
  const Mask safe = safeFreeSpace(g, 8, 25);
  for (int r = 0; r < 9; ++r) {
    for (int c = 0; c < 200; ++c) {
      EXPECT_FALSE(safe[g.index(r, c)]);
      EXPECT_FALSE(safe[g.index(c, r)]);
    }
  }
  EXPECT_TRUE(safe[g.index(100, 100)]);
}

TEST(TrackPlanner, UnknownSpaceIsNotSafe)
{
  const Grid g = makeGrid(50, 50, -1);
  const Mask safe = safeFreeSpace(g, 1, 25);
  for (auto v : safe) {
    EXPECT_FALSE(v);
  }
  EXPECT_TRUE(planLoop(g, {0.0, 0.0}, 0.0, ringSettings()).empty());
}

TEST(TrackPlanner, DegenerateInputsReturnEmpty)
{
  Grid g = ring();
  g.resolution = 0.0;
  EXPECT_TRUE(planLoop(g, {0.0, -4.0}, 0.0, ringSettings()).empty());
}

TEST(TrackPlanner, DistanceTransformMeasuresEuclideanDistanceToWalls)
{
  const Grid g = room(21, 1);  // free 19 x 19 interior
  Mask free(g.data.size());
  for (std::size_t i = 0; i < free.size(); ++i) {
    free[i] = g.data[i] == 0;
  }
  const auto d = distanceTransform(free, g.width, g.height);
  EXPECT_DOUBLE_EQ(d[g.index(0, 5)], 0.0);
  EXPECT_DOUBLE_EQ(d[g.index(1, 5)], 1.0);
  EXPECT_DOUBLE_EQ(d[g.index(10, 10)], 10.0);
  EXPECT_NEAR(d[g.index(2, 2)], 2.0, 1e-9);
}

TEST(TrackPlanner, RingLapRunsAheadAlongTheCentreLine)
{
  const auto waypoints = planLoop(ring(), {0.0, -4.0}, 0.0, ringSettings());
  ASSERT_GE(waypoints.size(), 15U);
  EXPECT_GT(waypoints.front().x, 0.0);
  for (const auto & w : waypoints) {
    const double offset = std::max(std::abs(w.x), std::abs(w.y));
    EXPECT_GE(offset, 3.5);
    EXPECT_LE(offset, 4.5);
  }
  EXPECT_FALSE(loopIsClockwise(waypoints));
}

TEST(TrackPlanner, LapDirectionFollowsHeading)
{
  EXPECT_TRUE(loopIsClockwise(planLoop(ring(), {0.0, -4.0}, M_PI, ringSettings())));
}

TEST(TrackPlanner, PartialTrackIsNotALap)
{
  Grid u_shape = ring();
  fill(u_shape, 160, 198, 90, 110, -1);
  EXPECT_TRUE(planLoop(u_shape, {0.0, -4.0}, 0.0, ringSettings()).empty());
}

TEST(TrackPlanner, UnsealedCutIsNotALap)
{
  LoopSettings s = ringSettings();
  s.cut_half_width = 0.5;
  EXPECT_TRUE(planLoop(ring(), {0.0, -4.0}, 0.0, s).empty());
}

TEST(TrackPlanner, FibonacciArenaLapWindsThroughTheU)
{
  const Grid g = fibonacciArena();
  const auto waypoints = planLoop(g, {0.0, -0.74}, 0.0, arenaLoopSettings());
  ASSERT_GE(waypoints.size(), 10U);
  bool inside_u = false, right = false, left = false;
  for (const auto & w : waypoints) {
    inside_u |= std::abs(w.x) < 0.5 && w.y > -0.4 && w.y < 0.2;
    right |= w.x > 0.7;
    left |= w.x < -0.7;
    const Cell c = g.cellOf({w.x, w.y});
    EXPECT_EQ(g.data[g.index(c.row, c.col)], 0);
  }
  EXPECT_TRUE(inside_u);
  EXPECT_TRUE(right);
  EXPECT_TRUE(left);
}

TEST(TrackPlanner, ExplorationGoalHeadsToFarEndOfCorridor)
{
  Grid g = makeGrid(200, 200, -1);
  fill(g, 95, 105, 40, 120, 0);
  LoopSettings s;
  s.clearance = 0.1;
  const auto goal = planExplorationGoal(g, {-2.5, 0.0}, s, std::nullopt);
  ASSERT_TRUE(goal);
  EXPECT_GT(goal->x, 0.5);
  EXPECT_LT(std::abs(goal->y), 0.3);
  EXPECT_LT(std::abs(goal->yaw), 0.2);
}

TEST(TrackPlanner, ExplorationGoalNoneWhenNothingIsFarEnough)
{
  Grid g = makeGrid(200, 200, -1);
  fill(g, 98, 103, 98, 103, 0);
  LoopSettings s;
  s.clearance = 0.05;
  s.min_spacing = 1.0;
  EXPECT_FALSE(planExplorationGoal(g, {0.0, 0.0}, s, std::nullopt));
}

TEST(TrackPlanner, ExplorationGoalPrefersCellsAheadOfRobot)
{
  LoopSettings s;
  s.clearance = 0.2;
  const auto facing_back = planExplorationGoal(lCorridor(), {1.0, 0.0}, s, M_PI);
  ASSERT_TRUE(facing_back);
  EXPECT_LT(facing_back->x, 0.0);
  const auto facing_corner = planExplorationGoal(lCorridor(), {1.0, 0.0}, s, 0.0);
  ASSERT_TRUE(facing_corner);
  EXPECT_GT(facing_corner->x, 1.0);
}

TEST(TrackPlanner, ExplorationGoalIgnoresShadowBesideTheRobot)
{
  // 0.5 m corridor with occupied walls along +x; an unknown shadow band on the right (the cables' shadow) must not
  // push the goal off the centre line (it used to sit ~4 cm left at every goal, so the robot turned left).
  Grid g = makeGrid(200, 200, -1);
  fill(g, 95, 105, 60, 160, 0);     // free corridor, rows 95..104 (centre line y = 0)
  fill(g, 94, 95, 60, 160, 100);    // right wall
  fill(g, 105, 106, 60, 160, 100);  // left wall
  fill(g, 95, 98, 96, 130, -1);     // 15 cm shadow band along the right wall, from the robot to past the goal
  LoopSettings s;
  s.clearance = 0.13;
  s.min_spacing = 0.3;
  s.max_exploration_distance = 0.6;
  const auto goal = planExplorationGoal(g, {0.0, 0.0}, s, 0.0);
  ASSERT_TRUE(goal);
  EXPECT_GT(goal->x, 0.3);
  EXPECT_NEAR(goal->y, 0.0, 0.03);
}

TEST(TrackPlanner, ExplorationDistanceCap)
{
  Grid g = makeGrid(200, 200, -1);
  fill(g, 95, 105, 40, 160, 0);
  LoopSettings s;
  s.clearance = 0.1;
  s.max_exploration_distance = 2.0;
  const auto goal = planExplorationGoal(g, {-2.5, 0.0}, s, std::nullopt);
  ASSERT_TRUE(goal);
  EXPECT_LE(std::hypot(goal->x + 2.5, goal->y), 2.0 + 1e-9);
}

TEST(TrackPlanner, ExplorationNeverFallsBackToCellsBehind)
{
  Grid g = makeGrid(200, 200, -1);
  fill(g, 95, 105, 40, 120, 0);
  LoopSettings s;
  s.clearance = 0.1;
  // At the corridor's open end, facing unknown space: everything known lies behind.
  EXPECT_FALSE(planExplorationGoal(g, {0.8, 0.0}, s, 0.0));
}

TEST(TrackPlanner, ExplorationGoalInsideTheUNeverJumpsThroughTheArm)
{
  LoopSettings s;
  s.clearance = 0.12;
  s.min_spacing = 0.3;
  s.cut_offset = 0.2;
  s.cut_half_width = 0.6;
  s.max_exploration_distance = 1.0;
  const auto goal = planExplorationGoal(fibonacciArena(), {0.3, 0.3}, s, -M_PI / 2);
  ASSERT_TRUE(goal);
  EXPECT_LT(std::abs(goal->x), 0.6);
  EXPECT_LT(goal->y, 0.42);
}

TEST(TrackPlanner, ExplorationFollowsCentreLineRoundTheArmHairpin)
{
  LoopSettings s;
  s.clearance = 0.12;
  s.min_spacing = 0.3;
  s.cut_offset = 0.2;
  s.cut_half_width = 0.6;
  s.max_exploration_distance = 0.6;
  const Grid g = fibonacciArena();
  const auto goal = planExplorationGoal(g, {-0.31, 0.35}, s, M_PI / 2);
  ASSERT_TRUE(goal);
  Mask free(g.data.size());
  for (std::size_t i = 0; i < free.size(); ++i) {
    free[i] = g.data[i] == 0;
  }
  const auto wall = distanceTransform(free, g.width, g.height);
  const Cell c = g.cellOf({goal->x, goal->y});
  EXPECT_GE(wall[g.index(c.row, c.col)] * g.resolution, 0.2);
  EXPECT_LT(goal->y, 1.0);
}

TEST(TrackPlanner, JitterAtStartIsNotACrossing)
{
  StartLineMonitor m({0.0, 0.0, 0.0});
  EXPECT_TRUE(crossings(m, {{0.0, 0.0}, {-0.02, 0.0}, {0.03, 0.01}, {-0.05, 0.0}, {0.5, 0.0}}).empty());
}

TEST(TrackPlanner, BackingOutOfStartIsReverse)
{
  StartLineMonitor m({0.0, 0.0, 0.0});
  EXPECT_EQ(crossings(m, {{0.0, 0.0}, {-0.3, 0.0}}), std::vector<int>{-1});
}

TEST(TrackPlanner, FullLapArrivingFromBehindIsForward)
{
  StartLineMonitor m({0.0, 0.0, 0.0});
  EXPECT_EQ(crossings(m, {{0.5, 0.0}, {2.0, 0.0}, {2.0, 3.0}, {-2.0, 3.0}, {-2.0, 0.0}, {-0.5, 0.0}, {0.5, 0.0}}),
    std::vector<int>{1});
}

TEST(TrackPlanner, FarSideOfTrackIsIgnored)
{
  StartLineMonitor m({0.0, 0.0, 0.0}, 1.0);
  EXPECT_TRUE(crossings(m, {{0.5, 0.0}, {2.0, 0.0}, {0.5, 3.0}, {-0.5, 3.0}}).empty());
}

TEST(TrackPlanner, PassageBehindAWallIsNotInTheStartZone)
{
  const Grid g = fibonacciArena();
  const StartLine line{0.0, -0.735, 0.0};
  StartLineMonitor m(line, 0.6, 1.0);
  const Mask zone = startZoneMask(g, line, 0.6, 1.0, 25);
  ASSERT_FALSE(zone.empty());
  m.zone = [&g, &zone](Point p) {
    const Cell c = g.cellOf(p);
    return g.inside(c.row, c.col) && zone[g.index(c.row, c.col)];
  };
  EXPECT_TRUE(m.zone({0.3, -0.735}));
  EXPECT_FALSE(m.zone({0.0, -0.2}));
  const std::vector<Point> u_passage_westwards{{0.4, -0.25}, {0.2, -0.25}, {0.0, -0.25}, {-0.2, -0.25}, {-0.4, -0.25}};
  EXPECT_TRUE(crossings(m, u_passage_westwards).empty());
  StartLineMonitor without_zone(line, 0.6, 1.0);
  std::vector<Point> path{{0.3, -0.735}};
  path.insert(path.end(), u_passage_westwards.begin(), u_passage_westwards.end());
  const auto c = crossings(without_zone, path);
  EXPECT_NE(std::find(c.begin(), c.end(), -1), c.end());
}

TEST(TrackPlanner, StartAheadSkipsWaypointBehindRobot)
{
  const std::vector<Waypoint> loop{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}};
  EXPECT_DOUBLE_EQ(startAhead(loop, {0.2, 0.0}, 0.0).front().x, 1.0);
  EXPECT_DOUBLE_EQ(startAhead(loop, {-0.2, 0.0}, 0.0).front().x, 0.0);
}

TEST(TrackPlanner, DropBehindKeepsFarAndForwardWaypoints)
{
  const auto kept = dropBehind({{1.0, 0.0, 0.0}, {-0.5, 0.2, 0.0}, {-3.0, 0.0, 0.0}}, {0.0, 0.0}, 0.0, 2.0);
  ASSERT_EQ(kept.size(), 2U);
  EXPECT_DOUBLE_EQ(kept[0].x, 1.0);
  EXPECT_DOUBLE_EQ(kept[1].x, -3.0);
}

TEST(TrackPlanner, RearGateSpansCorridorBetweenWalls)
{
  Grid g = makeGrid();
  fill(g, 80, 82, 0, 200, 100);
  fill(g, 118, 120, 0, 200, 100);
  const Gate gate = rearGate(g, {0.0, 0.0}, 0.0, 0.4, 1.5);
  std::set<int> rows, cols;
  for (const auto & c : gate.cells) {
    rows.insert(c.row);
    cols.insert(c.col);
  }
  EXPECT_LE(*rows.begin(), 81);
  EXPECT_GE(*rows.rbegin(), 118);
  EXPECT_LT(*cols.rbegin(), 100);
  for (int c : cols) {
    EXPECT_GE(c, 90);
    EXPECT_LE(c, 93);
  }
  EXPECT_TRUE(gate.sealed);
}

TEST(TrackPlanner, RearGateForcesTheLongWayRound)
{
  const Grid g = ring();
  const Point robot{0.0, -4.0};
  const Cell seed{static_cast<int>((robot.y + 5.0) / 0.05), static_cast<int>((robot.x + 5.0) / 0.05)};
  const Cell behind{seed.row, seed.col - 16};
  const int open_track = stepsTo(safeFreeSpace(g, 1, 25), g.width, g.height, seed, behind);
  Grid gated = g;
  for (const auto & c : rearGate(g, robot, 0.0, 0.4, 1.5).cells) {
    gated.data[gated.index(c.row, c.col)] = 100;
  }
  const int gated_track = stepsTo(safeFreeSpace(gated, 1, 25), g.width, g.height, seed, behind);
  EXPECT_GE(open_track, 0);
  EXPECT_LE(open_track, 20);
  EXPECT_GT(gated_track, 400);
}

namespace
{

/// Closed rectangle lap (counterclockwise) with a waypoint every `step` metres, starting at (0, 0).
std::vector<autonom_patrol::Waypoint> rectangleLap(double length, double width, double step)
{
  std::vector<autonom_patrol::Waypoint> lap;
  const auto side = [&lap, step](double x0, double y0, double dx, double dy, double size) {
    const int n = static_cast<int>(std::lround(size / step));
    for (int i = 0; i < n; ++i) {
      lap.push_back({x0 + dx * step * i, y0 + dy * step * i, 0.0});
    }
  };
  side(0.0, 0.0, 1.0, 0.0, length);
  side(length, 0.0, 0.0, 1.0, width);
  side(length, width, -1.0, 0.0, length);
  side(0.0, width, 0.0, -1.0, width);
  return lap;
}

}  // namespace

TEST(TrackPlanner, ShortStraightsAreTheSidesShorterThanTheLimit)
{
  // 2.0 m straights joined by 0.6 m ones, waypoints every 0.2 m: segments 0-9 long side, 10-12 short side, ...
  const auto lap = rectangleLap(2.0, 0.6, 0.2);
  ASSERT_EQ(lap.size(), 26U);
  const auto short_segment = autonom_patrol::shortStraightSegments(lap, 0.35, 0.8);
  for (std::size_t i = 0; i < lap.size(); ++i) {
    const bool on_short_side = (i >= 10 && i < 13) || i >= 23;
    EXPECT_EQ(short_segment[i], on_short_side) << "segment " << i;
  }
}

TEST(TrackPlanner, NoShortStraightsWhenEverySideIsLongEnough)
{
  const auto short_segment = autonom_patrol::shortStraightSegments(rectangleLap(2.0, 1.0, 0.2), 0.35, 0.8);
  EXPECT_TRUE(std::none_of(short_segment.begin(), short_segment.end(), [](bool s) { return s; }));
}

TEST(TrackPlanner, EveryCornerSegmentIsItsOwnShortStraight)
{
  // A ring of 12 waypoints (30 deg turns, radius 0.5 m): each segment is a corner, 0.26 m long.
  std::vector<autonom_patrol::Waypoint> ring_lap;
  for (int i = 0; i < 12; ++i) {
    ring_lap.push_back({0.5 * std::cos(i * M_PI / 6.0), 0.5 * std::sin(i * M_PI / 6.0), 0.0});
  }
  const auto tight = autonom_patrol::shortStraightSegments(ring_lap, 0.35, 0.8);
  EXPECT_TRUE(std::all_of(tight.begin(), tight.end(), [](bool s) { return s; }));
  // Above the corner angle the ring is one 3.1 m straight: not short.
  const auto gentle = autonom_patrol::shortStraightSegments(ring_lap, 0.6, 0.8);
  EXPECT_TRUE(std::none_of(gentle.begin(), gentle.end(), [](bool s) { return s; }));
}

TEST(TrackPlanner, ShortStraightsNeedALap)
{
  EXPECT_EQ(autonom_patrol::shortStraightSegments({{0.0, 0.0, 0.0}, {0.3, 0.0, 0.0}}, 0.35, 0.8),
    std::vector<bool>(2, false));
  EXPECT_FALSE(autonom_patrol::nearestSegment({{0.0, 0.0, 0.0}}, {0.0, 0.0}).has_value());
}

TEST(TrackPlanner, NearestSegmentIncludesTheClosingSegment)
{
  const auto lap = rectangleLap(2.0, 0.6, 0.2);
  EXPECT_EQ(autonom_patrol::nearestSegment(lap, {0.51, -0.05}), 2U);
  EXPECT_EQ(autonom_patrol::nearestSegment(lap, {2.05, 0.3}), 11U);
  EXPECT_EQ(autonom_patrol::nearestSegment(lap, {-0.05, 0.1}), 25U);  // last waypoint back to the first
}

TEST(TrackPlanner, DistanceToNextCornerCountsDownAlongEachStraight)
{
  const auto lap = rectangleLap(2.0, 0.6, 0.2);  // corners at waypoints 0, 10, 13, 23
  const auto d = autonom_patrol::distanceToNextCorner(lap, 0.35);
  EXPECT_NEAR(d[0], 1.8, 1e-9);   // segment 0 ends at waypoint 1: 9 segments to waypoint 10
  EXPECT_NEAR(d[9], 0.0, 1e-9);   // ends at the corner
  EXPECT_NEAR(d[10], 0.4, 1e-9);
  EXPECT_NEAR(d[25], 0.0, 1e-9);  // closing segment ends at waypoint 0, a corner
  EXPECT_NEAR(d[23], 0.4, 1e-9);
}

TEST(TrackPlanner, SectionSpeedsBrakeBeforeCornersAndOnShortStraights)
{
  autonom_patrol::SectionSpeedSettings s;
  s.race_speed = 1.4;
  s.short_straight_length = 0.8;
  s.short_straight_speed = 1.0;
  s.corner_approach_distance = 0.5;
  s.corner_approach_speed = 0.8;
  const auto v = autonom_patrol::sectionSpeeds(rectangleLap(2.0, 0.6, 0.2), s);
  EXPECT_DOUBLE_EQ(v[0], 1.4);  // long straight, far from the corner
  EXPECT_DOUBLE_EQ(v[6], 1.4);  // ends 0.6 m before the corner
  EXPECT_DOUBLE_EQ(v[7], 0.8);  // ends 0.4 m before it: braking
  EXPECT_DOUBLE_EQ(v[9], 0.8);
  EXPECT_DOUBLE_EQ(v[11], 0.8);  // short straight that is also a corner approach: the lower cap
  s.corner_approach_speed = 0.0;
  const auto no_approach = autonom_patrol::sectionSpeeds(rectangleLap(2.0, 0.6, 0.2), s);
  EXPECT_DOUBLE_EQ(no_approach[7], 1.4);
  EXPECT_DOUBLE_EQ(no_approach[11], 1.0);
}

TEST(TrackPlanner, NoCornerMeansNoApproachCap)
{
  const std::vector<autonom_patrol::Waypoint> line{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  // Degenerate "lap" that doubles back: the closing segment turns 180 deg, so there is a corner; a lap without any
  // corner needs a gentle ring.
  std::vector<autonom_patrol::Waypoint> ring_lap;
  for (int i = 0; i < 12; ++i) {
    ring_lap.push_back({0.5 * std::cos(i * M_PI / 6.0), 0.5 * std::sin(i * M_PI / 6.0), 0.0});
  }
  const auto d = autonom_patrol::distanceToNextCorner(ring_lap, 0.6);
  EXPECT_TRUE(std::all_of(d.begin(), d.end(), [](double x) { return std::isinf(x); }));
  EXPECT_FALSE(std::isinf(autonom_patrol::distanceToNextCorner(line, 0.35)[0]));
}

TEST(TrackPlanner, TrailPointBehindFollowsTheCornerBack)
{
  // Drove +x along the bottom straight, then turned up (+y): 0.25 m back along the path is still on the straight.
  const std::vector<autonom_patrol::Point> trail{{0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0}, {0.6, 0.2}};
  const auto behind = autonom_patrol::trailPointBehind(trail, 0.25);
  ASSERT_TRUE(behind.has_value());
  EXPECT_NEAR(behind->first.x, 0.55, 1e-9);
  EXPECT_NEAR(behind->first.y, 0.0, 1e-9);
  EXPECT_NEAR(behind->second, 0.0, 1e-9);  // across the straight's direction, so the gate cuts that corridor
  // Within the newest segment: the exit direction.
  const auto near = autonom_patrol::trailPointBehind(trail, 0.1);
  ASSERT_TRUE(near.has_value());
  EXPECT_NEAR(near->first.y, 0.1, 1e-9);
  EXPECT_NEAR(near->second, M_PI / 2.0, 1e-9);
}

TEST(TrackPlanner, TrailPointBehindStopsAtTheOldestPoint)
{
  const std::vector<autonom_patrol::Point> trail{{0.0, 0.0}, {0.2, 0.0}};
  const auto behind = autonom_patrol::trailPointBehind(trail, 1.0);
  ASSERT_TRUE(behind.has_value());
  EXPECT_NEAR(behind->first.x, 0.0, 1e-9);
  EXPECT_FALSE(autonom_patrol::trailPointBehind({{0.0, 0.0}}, 0.25).has_value());
  EXPECT_FALSE(autonom_patrol::trailPointBehind({{0.0, 0.0}, {0.0, 0.0}}, 0.25).has_value());
}

TEST(TrackPlanner, GateOnTheTrailSealsTheCorridorTheRobotCameFrom)
{
  // The ring track's bottom straight: the gate placed from the trail point behind the robot, across the travel
  // direction there, reaches both walls.
  const autonom_patrol::Grid g = ring();
  const std::vector<autonom_patrol::Point> trail{{0.0, -4.0}, {0.5, -4.0}, {1.0, -4.0}, {1.2, -4.0}};
  const auto behind = autonom_patrol::trailPointBehind(trail, 0.4);
  ASSERT_TRUE(behind.has_value());
  const auto gate = autonom_patrol::rearGate(g, behind->first, behind->second, 0.0, 1.5);
  EXPECT_TRUE(gate.sealed);
}
