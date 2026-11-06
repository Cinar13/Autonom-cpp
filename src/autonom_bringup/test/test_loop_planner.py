import math

import numpy as np

from autonom_bringup.loop_planner import GridInfo, LoopSettings, plan_loop, safe_free_space

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
