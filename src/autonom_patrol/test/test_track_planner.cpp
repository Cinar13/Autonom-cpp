#include <gtest/gtest.h>

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

TEST(TrackPlanner, RivalTraceInTheCorridorIsCleared)
{
  Grid g = makeGrid(200, 200, -1);
  fill(g, 94, 106, 40, 160, 0);
  fill(g, 93, 94, 40, 160, 100);
  fill(g, 106, 107, 40, 160, 100);
  fill(g, 99, 102, 120, 123, 100);  // 3 x 3 cells (0.15 m) left behind by a rival
  EXPECT_EQ(clearRivalTraces(g, 0.25, 30, 25), 9);
  EXPECT_EQ(g.data[g.index(100, 121)], 0);
  EXPECT_EQ(g.data[g.index(93, 100)], 100);  // walls stay
}

TEST(TrackPlanner, BlobTouchingAWallOrUnknownSpaceStays)
{
  Grid g = makeGrid(200, 200, -1);
  fill(g, 94, 106, 40, 160, 0);
  fill(g, 93, 94, 40, 160, 100);
  fill(g, 94, 96, 100, 102, 100);   // bump on the wall: part of the wall component
  fill(g, 100, 102, 158, 160, 100); // small blob at the corridor's open end, next to unknown space
  EXPECT_EQ(clearRivalTraces(g, 0.25, 30, 25), 0);
}

TEST(TrackPlanner, LargeIslandsStay)
{
  Grid g = makeGrid(200, 200, 0);
  fill(g, 90, 92, 60, 140, 100);  // free-standing wall 4 m long, 0.1 m thick
  fill(g, 120, 127, 100, 107, 100);  // 0.35 m block: larger than a rival
  EXPECT_EQ(clearRivalTraces(g, 0.25, 30, 25), 0);
}

TEST(TrackPlanner, ArenaWallsSurviveTraceClearing)
{
  // The arena's U is a free-standing island; the divider hangs from the top wall. Neither may be cleared.
  Grid g = fibonacciArena();
  const auto before = g.data;
  fill(g, 20, 23, 20, 23, 100);  // one rival trace in the bottom-left corridor
  EXPECT_EQ(clearRivalTraces(g, 0.25, 48, 25), 9);
  EXPECT_EQ(g.data, before);
}

TEST(TrackPlanner, RivalsBesideTheStartDoNotTrapTheRobot)
{
  // Start line with a rival robot on each side (docs/07): the clearance around them leaves a band of unsafe cells
  // across the 0.6 m corridor through the robot's own cell. The nearest safe cell overall lies behind the robot,
  // in the pocket in front of the rear gate; the search must start from the cells ahead instead.
  Grid g = makeGrid(200, 200, -1);
  fill(g, 94, 106, 40, 160, 0);     // corridor y -0.3..0.3, x -3..3
  fill(g, 93, 94, 40, 160, 100);
  fill(g, 106, 107, 40, 160, 100);
  fill(g, 103, 105, 99, 102, 100);  // left rival, y 0.15..0.25
  fill(g, 95, 97, 99, 102, 100);    // right rival, y -0.25..-0.15
  LoopSettings s;
  s.clearance = 0.12;
  s.cut_offset = 0.25;
  s.cut_half_width = 0.6;
  s.max_exploration_distance = 0.6;
  const Point robot{0.0, 0.0};
  const Mask safe = safeFreeSpace(g, 3, s.free_threshold);
  ASSERT_FALSE(safe[g.index(g.cellOf(robot).row, g.cellOf(robot).col)]);
  const auto any = nearestCell(safe, g, robot);
  ASSERT_TRUE(any);
  EXPECT_LT(g.centre(any->row, any->col).x, 0.0);  // the trap: nearest safe cell is behind
  const auto ahead = nearestCell(safe, g, robot, 0.0);
  ASSERT_TRUE(ahead);
  EXPECT_GT(g.centre(ahead->row, ahead->col).x, 0.0);
  const auto goal = planExplorationGoal(g, robot, s, 0.0);
  ASSERT_TRUE(goal);
  EXPECT_GT(goal->x, 0.3);
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
