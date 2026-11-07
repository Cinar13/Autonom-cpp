import math

import numpy as np

from autonom_bringup.loop_planner import (
    GridInfo, LoopSettings, StartLine, StartLineMonitor, loop_follows_heading, start_ahead, plan_exploration_goal, plan_loop, safe_free_space)

INFO = GridInfo(resolution=0.05, origin_x=-5.0, origin_y=-5.0)


def _room(size_cells=200, wall=2):
    grid = np.zeros((size_cells, size_cells), dtype=np.int16)
    grid[:wall, :] = grid[-wall:, :] = grid[:, :wall] = grid[:, -wall:] = 100
    return grid


def test_clearance_keeps_waypoints_away_from_walls():
    settings = LoopSettings(clearance=0.4)
    waypoints = plan_loop(_room(), INFO, (0.0, 0.0), settings)
    assert len(waypoints) >= 3
    for x, y, _ in waypoints:
        assert -5.0 + 0.4 <= x <= 5.0 - 0.4
        assert -5.0 + 0.4 <= y <= 5.0 - 0.4


def test_loop_starts_nearest_robot_and_heads_to_next_waypoint():
    waypoints = plan_loop(_room(), INFO, (4.0, 4.0), LoopSettings())
    distances = [math.dist((x, y), (4.0, 4.0)) for x, y, _ in waypoints]
    assert distances[0] == min(distances)
    x0, y0, yaw0 = waypoints[0]
    x1, y1, _ = waypoints[1]
    assert math.isclose(yaw0, math.atan2(y1 - y0, x1 - x0))


def test_unreachable_region_is_ignored():
    grid = _room()
    grid[:, 100:104] = 100
    waypoints = plan_loop(grid, INFO, (-2.5, 0.0), LoopSettings())
    assert waypoints
    assert all(x < 0.0 for x, _, _ in waypoints)


def test_unknown_space_is_not_safe():
    grid = np.full((50, 50), -1, dtype=np.int16)
    assert not safe_free_space(grid, 1, 25).any()
    assert plan_loop(grid, INFO, (0.0, 0.0), LoopSettings()) == []


def test_degenerate_inputs_return_empty():
    assert plan_loop(_room(), GridInfo(0.0, 0.0, 0.0), (0.0, 0.0), LoopSettings()) == []
    assert plan_loop(_room(), INFO, (0.0, 0.0), LoopSettings(waypoint_count=2)) == []


def test_exploration_goal_heads_to_far_end_of_corridor():
    grid = np.full((200, 200), -1, dtype=np.int16)
    grid[95:105, 40:120] = 0
    goal = plan_exploration_goal(grid, INFO, (-2.5, 0.0), LoopSettings(clearance=0.1))
    assert goal is not None
    x, y, yaw = goal
    assert x > 0.5
    assert abs(y) < 0.3
    assert abs(yaw) < 0.2


def test_exploration_goal_none_when_nothing_is_far_enough():
    grid = np.full((200, 200), -1, dtype=np.int16)
    grid[98:103, 98:103] = 0
    assert plan_exploration_goal(grid, INFO, (0.0, 0.0), LoopSettings(clearance=0.05, min_spacing=1.0)) is None


def _signed_area(waypoints):
    points = [(x, y) for x, y, _ in waypoints]
    return 0.5 * sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]))


def test_loop_direction_follows_setting():
    ccw = plan_loop(_room(), INFO, (4.0, 4.0), LoopSettings(clockwise=False))
    cw = plan_loop(_room(), INFO, (4.0, 4.0), LoopSettings(clockwise=True))
    assert _signed_area(ccw) > 0.0
    assert _signed_area(cw) < 0.0
    assert cw[0][:2] == ccw[0][:2]


def _l_corridor():
    grid = np.full((200, 200), -1, dtype=np.int16)
    grid[90:110, 40:160] = 0
    grid[90:180, 140:160] = 0
    return grid


def test_partial_map_is_not_a_loop():
    assert plan_loop(_l_corridor(), INFO, (-2.5, 0.0), LoopSettings(clearance=0.2)) == []


def test_exploration_goal_prefers_cells_ahead_of_robot():
    settings = LoopSettings(clearance=0.2)
    facing_back = plan_exploration_goal(_l_corridor(), INFO, (1.0, 0.0), settings, heading=math.pi)
    assert facing_back is not None and facing_back[0] < 0.0
    facing_corner = plan_exploration_goal(_l_corridor(), INFO, (1.0, 0.0), settings, heading=0.0)
    assert facing_corner is not None and facing_corner[0] > 1.0


def _run(monitor, path):
    return [c for c in (monitor.update(xy) for xy in path) if c]


def test_jitter_at_start_is_not_a_crossing():
    monitor = StartLineMonitor(StartLine(0.0, 0.0, 0.0))
    assert _run(monitor, [(0.0, 0.0), (-0.02, 0.0), (0.03, 0.01), (-0.05, 0.0), (0.5, 0.0)]) == []


def test_backing_out_of_start_is_reverse():
    monitor = StartLineMonitor(StartLine(0.0, 0.0, 0.0))
    assert _run(monitor, [(0.0, 0.0), (-0.3, 0.0)]) == [-1]


def test_full_lap_arriving_from_behind_is_forward():
    monitor = StartLineMonitor(StartLine(0.0, 0.0, 0.0))
    lap = [(0.5, 0.0), (2.0, 0.0), (2.0, 3.0), (-2.0, 3.0), (-2.0, 0.0), (-0.5, 0.0), (0.5, 0.0)]
    assert _run(monitor, lap) == [1]


def test_far_side_of_track_is_ignored():
    monitor = StartLineMonitor(StartLine(0.0, 0.0, 0.0), half_width=1.0)
    assert _run(monitor, [(0.5, 0.0), (2.0, 0.0), (0.5, 3.0), (-0.5, 3.0)]) == []


def test_loop_orientation_matches_start_heading():
    ccw = plan_loop(_room(), INFO, (4.0, 4.0), LoopSettings())
    x0, y0, _ = ccw[0]
    assert loop_follows_heading(ccw, (x0, y0), ccw[0][2])
    assert not loop_follows_heading(ccw, (x0, y0), ccw[0][2] + math.pi)
    cw = plan_loop(_room(), INFO, (4.0, 4.0), LoopSettings(clockwise=True))
    assert loop_follows_heading(cw, (x0, y0), ccw[0][2] + math.pi)


def test_exploration_distance_cap():
    grid = np.full((200, 200), -1, dtype=np.int16)
    grid[95:105, 40:160] = 0
    goal = plan_exploration_goal(grid, INFO, (-2.5, 0.0), LoopSettings(clearance=0.1, max_exploration_distance=2.0))
    assert goal is not None
    assert math.dist(goal[:2], (-2.5, 0.0)) <= 2.0 + 1e-9


def test_start_ahead_skips_waypoint_behind_robot():
    loop = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
    assert start_ahead(loop, (0.2, 0.0), 0.0)[0] == (1.0, 0.0, 0.0)
    assert start_ahead(loop, (-0.2, 0.0), 0.0) == loop
