import math

import numpy as np

from autonom_bringup.loop_planner import (
    GridInfo, LoopSettings, StartLine, StartLineMonitor, drop_behind, loop_is_clockwise, plan_exploration_goal,
    plan_loop, rear_gate_cells, safe_free_space, start_ahead)

INFO = GridInfo(resolution=0.05, origin_x=-5.0, origin_y=-5.0)
RING = LoopSettings(clearance=0.3, waypoint_spacing=1.0)


def _room(size_cells=200, wall=2):
    grid = np.zeros((size_cells, size_cells), dtype=np.int16)
    grid[:wall, :] = grid[-wall:, :] = grid[:, :wall] = grid[:, -wall:] = 100
    return grid


def _ring():
    """Square track: 10 m outer wall, 6 m inner block, 2 m wide corridor (centre line 4 m from the middle)."""
    grid = _room()
    grid[40:160, 40:160] = 100
    return grid


ARENA_INFO = GridInfo(resolution=0.025, origin_x=-1.5, origin_y=-1.25)


def _fibonacci_arena():
    """The race map (Fibonacci-Autonomous-Map): 2.49 x 2.10 m box, U island open to the top, divider from the
    top wall into the U's opening. Walls 2 cells (5 cm) thick."""
    grid = np.full((100, 120), 100, dtype=np.int16)
    def cell(x, y):
        return int((y - ARENA_INFO.origin_y) / 0.025), int((x - ARENA_INFO.origin_x) / 0.025)
    r0, c0 = cell(-1.245, -1.05)
    r1, c1 = cell(1.245, 1.05)
    grid[r0 + 2:r1 - 1, c0 + 2:c1 - 1] = 0
    def wall(x0, y0, x1, y1):
        ra, ca = cell(min(x0, x1), min(y0, y1))
        rb, cb = cell(max(x0, x1), max(y0, y1))
        grid[ra - 1:rb + 1, ca - 1:cb + 1] = 100
    wall(-0.62, -0.42, 0.62, -0.42)   # U bottom
    wall(-0.62, -0.42, -0.62, 0.42)   # U left arm
    wall(0.62, -0.42, 0.62, 0.42)     # U right arm
    wall(0.0, 0.21, 0.0, 1.05)        # divider from the top wall
    return grid


def test_clearance_keeps_unsafe_cells_out():
    safe = safe_free_space(_room(), 8, 25)
    assert not safe[:9, :].any() and not safe[:, :9].any()
    assert safe[100, 100]


def test_unknown_space_is_not_safe():
    grid = np.full((50, 50), -1, dtype=np.int16)
    assert not safe_free_space(grid, 1, 25).any()
    assert plan_loop(grid, INFO, (0.0, 0.0), 0.0, RING) == []


def test_degenerate_inputs_return_empty():
    assert plan_loop(_ring(), GridInfo(0.0, 0.0, 0.0), (0.0, -4.0), 0.0, RING) == []


def test_ring_lap_runs_ahead_along_the_centre_line():
    waypoints = plan_loop(_ring(), INFO, (0.0, -4.0), 0.0, RING)
    assert len(waypoints) >= 15                       # ~32 m lap, 1 m spacing (4-connected steps)
    assert waypoints[0][0] > 0.0                      # first waypoint ahead (east)
    for x, y, _ in waypoints:
        assert 3.5 <= max(abs(x), abs(y)) <= 4.5     # corridor 3..5 m from the middle, centre line at 4 m
    assert not loop_is_clockwise(waypoints)          # east along the bottom = counter-clockwise


def test_lap_direction_follows_heading():
    assert loop_is_clockwise(plan_loop(_ring(), INFO, (0.0, -4.0), math.pi, RING))


def test_partial_track_is_not_a_lap():
    u_shape = _ring()
    u_shape[160:198, 90:110] = -1                     # top corridor not mapped yet
    assert plan_loop(u_shape, INFO, (0.0, -4.0), 0.0, RING) == []


def test_fibonacci_arena_lap_winds_through_the_u():
    settings = LoopSettings(clearance=0.15, waypoint_spacing=0.4, min_loop_length=2.0, cut_offset=0.2,
                            cut_half_width=0.6)
    grid = _fibonacci_arena()
    waypoints = plan_loop(grid, ARENA_INFO, (0.0, -0.74), 0.0, settings)   # bottom corridor, heading east
    assert len(waypoints) >= 10
    # The lap passes inside the U, under the divider tip.
    assert any(abs(x) < 0.5 and -0.4 < y < 0.2 for x, y, _ in waypoints)
    # and uses the outer side corridors.
    assert any(x > 0.7 for x, _, _ in waypoints) and any(x < -0.7 for x, _, _ in waypoints)
    for x, y, _ in waypoints:
        r = int((y - ARENA_INFO.origin_y) / 0.025)
        c = int((x - ARENA_INFO.origin_x) / 0.025)
        assert grid[r, c] == 0


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


def _l_corridor():
    grid = np.full((200, 200), -1, dtype=np.int16)
    grid[90:110, 40:160] = 0
    grid[90:180, 140:160] = 0
    return grid


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


def _ring():
    """Square track: 10 m outer wall, 6 m inner block, 2 m wide corridor."""
    grid = _room()
    grid[40:160, 40:160] = 100
    return grid


def test_exploration_never_falls_back_to_cells_behind():
    grid = np.full((200, 200), -1, dtype=np.int16)
    grid[95:105, 40:120] = 0
    # Robot at the corridor's open end, facing into unknown space: everything known lies behind it.
    goal = plan_exploration_goal(grid, INFO, (0.8, 0.0), LoopSettings(clearance=0.1), heading=0.0)
    assert goal is None



def test_rear_gate_spans_corridor_between_walls():
    grid = np.zeros((200, 200), dtype=np.int16)
    grid[80:82, :] = 100   # wall at y = -1.0
    grid[118:120, :] = 100  # wall at y = +0.9
    cells = rear_gate_cells(grid, INFO, (0.0, 0.0), direction=0.0, offset=0.4)
    rows = {r for r, _ in cells}
    cols = {c for _, c in cells}
    assert min(rows) <= 81 and max(rows) >= 118      # reaches both walls
    assert max(cols) < 100                          # behind the robot (x < 0)
    assert all(90 <= c <= 93 for c in cols)         # at x = -0.4 (column 92)


def _steps_to(free, seed, target, limit=2000):
    from autonom_bringup.loop_planner import _dilate
    reached = np.zeros_like(free)
    reached[seed] = True
    for step in range(limit):
        if reached[target]:
            return step
        grown = _dilate(reached, 1, border=False) & free
        if np.array_equal(grown, reached):
            return None
        reached = grown
    return None


def test_rear_gate_forces_the_long_way_round():
    from autonom_bringup.loop_planner import safe_free_space
    grid = _ring()
    robot = (0.0, -4.0)  # bottom corridor, driving east
    seed = (int((robot[1] + 5.0) / 0.05), int((robot[0] + 5.0) / 0.05))
    behind = (seed[0], seed[1] - 16)  # 0.8 m behind the robot, past the gate
    open_track = _steps_to(safe_free_space(grid, 1, 25), seed, behind)
    gated = grid.copy()
    for r, c in rear_gate_cells(grid, INFO, robot, direction=0.0):
        gated[r, c] = 100
    gated_track = _steps_to(safe_free_space(gated, 1, 25), seed, behind)
    assert open_track is not None and open_track <= 20       # straight back: 0.8 m
    assert gated_track is not None and gated_track > 400     # only forward round the whole ring (>20 m)



def test_drop_behind_keeps_far_and_forward_waypoints():
    loop = [(1.0, 0.0, 0.0), (-0.5, 0.2, 0.0), (-3.0, 0.0, 0.0)]
    assert drop_behind(loop, (0.0, 0.0), 0.0, 2.0) == [(1.0, 0.0, 0.0), (-3.0, 0.0, 0.0)]


def test_exploration_goal_inside_the_u_never_jumps_through_the_arm():
    # Regression: inside the U heading south, the straight-line farthest "ahead" cell was in the right outer
    # corridor, only reachable by turning back out of the U. Driving distance keeps the goal inside the U.
    settings = LoopSettings(clearance=0.12, min_spacing=0.3, cut_offset=0.2, cut_half_width=0.6,
                            max_exploration_distance=1.0)
    goal = plan_exploration_goal(_fibonacci_arena(), ARENA_INFO, (0.3, 0.3), settings, -math.pi / 2)
    assert goal is not None
    x, y, _ = goal
    assert abs(x) < 0.6 and y < 0.42      # still inside the U



def test_unsealed_cut_is_not_a_lap():
    # Cut half-width too short to reach both walls of the 2 m corridor: a detour round the cut end must not
    # be mistaken for the lap.
    settings = LoopSettings(clearance=0.3, waypoint_spacing=1.0, cut_half_width=0.5)
    assert plan_loop(_ring(), INFO, (0.0, -4.0), 0.0, settings) == []



def test_exploration_follows_centre_line_round_the_arm_hairpin():
    # Regression: leaving the U's left half heading north, the farthest cell was the top-left corner pocket;
    # the robot drove into it facing the wall and could then only continue by turning back. The centre-line
    # goal stays in the open middle of the turn, well clear of the walls.
    settings = LoopSettings(clearance=0.12, min_spacing=0.3, cut_offset=0.2, cut_half_width=0.6,
                            max_exploration_distance=0.6)
    goal = plan_exploration_goal(_fibonacci_arena(), ARENA_INFO, (-0.31, 0.35), settings, math.pi / 2)
    assert goal is not None
    x, y, _ = goal
    grid = _fibonacci_arena()
    free = grid == 0
    from scipy.ndimage import distance_transform_edt
    wall_distance = distance_transform_edt(free) * ARENA_INFO.resolution
    r = int((y - ARENA_INFO.origin_y) / ARENA_INFO.resolution)
    c = int((x - ARENA_INFO.origin_x) / ARENA_INFO.resolution)
    assert wall_distance[r, c] >= 0.2       # centre of the passage, not a corner pocket
    assert y < 1.0                          # not pressed against the top wall
