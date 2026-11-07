"""Plan race laps and exploration goals on a growing occupancy grid.

A lap follows the track itself, so it works for any closed track shape (the Fibonacci arena winds into a U and
around a divider, which no angular ordering around a centre point can follow):

1. Cut the track with a line just behind the robot (the same line as the rear gate).
2. Flood the safe free space forward from the robot; every cell gets its driving distance from the cut.
3. The track is a closed ring only if the flood reaches the far side of the cut the long way round; that
   distance is the lap length.
4. Every `waypoint_spacing` metres of driving distance, the cell farthest from the walls is a waypoint (track
   centre line). Laps therefore always run in the robot's direction of travel.

Until the ring is closed, `plan_exploration_goal` pushes the map frontier out ahead of the robot.
"""

from dataclasses import dataclass
import math
from typing import Callable, List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

Waypoint = Tuple[float, float, float]


@dataclass(frozen=True)
class GridInfo:
    resolution: float
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class LoopSettings:
    # Waypoints keep this distance from walls and unknown space (about half the track width minus a margin).
    clearance: float = 0.35
    min_spacing: float = 0.5
    free_threshold: int = 25
    # Driving distance between lap waypoints.
    waypoint_spacing: float = 1.0
    # Shorter "rings" are not a lap (e.g. a loop around a small obstacle in an open area).
    min_loop_length: float = 2.0
    # The lap is cut this far behind the robot, across at most +-cut_half_width.
    cut_offset: float = 0.3
    cut_half_width: float = 1.5
    # Exploration goals farther than this are ignored (0 = unlimited). On a closed track a far goal can be
    # closer the other way round, and Nav2 would then drive back through the start point.
    max_exploration_distance: float = 0.0


def _dilate(mask: np.ndarray, iterations: int, border: bool) -> np.ndarray:
    """4-neighbour binary dilation; `border` is the value assumed outside the grid."""
    result = mask
    for _ in range(iterations):
        padded = np.pad(result, 1, constant_values=border)
        result = (
            padded[1:-1, 1:-1] | padded[:-2, 1:-1] | padded[2:, 1:-1] | padded[1:-1, :-2] | padded[1:-1, 2:]
        )
    return result


def safe_free_space(grid: np.ndarray, clearance_cells: int, free_threshold: int) -> np.ndarray:
    """Free cells at least `clearance_cells` away from occupied, unknown and out-of-map cells."""
    free = (grid >= 0) & (grid < free_threshold)
    return ~_dilate(~free, clearance_cells, border=True)


def reachable_from(safe: np.ndarray, seed: Tuple[int, int]) -> np.ndarray:
    reached = np.zeros_like(safe)
    reached[seed] = True
    while True:
        grown = _dilate(reached, 1, border=False) & safe
        if np.array_equal(grown, reached):
            return reached
        reached = grown


def driving_distance(passable: np.ndarray, seed: Tuple[int, int]) -> np.ndarray:
    """4-connected wavefront steps from `seed` through `passable` cells; -1 where unreachable."""
    distance = np.full(passable.shape, -1, dtype=np.int32)
    distance[seed] = 0
    reached = np.zeros_like(passable)
    reached[seed] = True
    frontier = reached.copy()
    step = 0
    while frontier.any():
        step += 1
        grown = _dilate(frontier, 1, border=False) & passable & ~reached
        distance[grown] = step
        reached |= grown
        frontier = grown
    return distance


def plan_loop(
    grid: np.ndarray, info: GridInfo, robot_xy: Tuple[float, float], heading: float, settings: LoopSettings
) -> List[Waypoint]:
    """Lap waypoints (x, y, yaw) along the closed track ahead of the robot; empty until the ring is closed."""
    if grid.ndim != 2 or info.resolution <= 0.0 or settings.waypoint_spacing <= 0.0:
        return []
    res = info.resolution
    clearance_cells = max(1, math.ceil(settings.clearance / res))
    safe = safe_free_space(grid, clearance_cells, settings.free_threshold)
    cut, sealed = _rear_gate(grid, info, robot_xy, heading, settings.cut_offset, settings.cut_half_width, 50)
    if not sealed:
        return []  # an open cut end would let a short detour count as a lap
    passable = safe.copy()
    for row, col in cut:
        passable[row, col] = False
    seed = _nearest_cell(passable, info, robot_xy)
    if seed is None:
        return []
    distance = driving_distance(passable, seed)

    # The cell straight behind the cut, in the robot's own corridor: reached only by driving the whole ring.
    # (A sideways window could reach across a wall into a neighbouring part of the track.)
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    back = settings.cut_offset + 0.1
    target = _nearest_cell(passable, info, (robot_xy[0] - back * cos_h, robot_xy[1] - back * sin_h))
    if target is None or distance[target] < 0:
        return []
    tx, ty = info.origin_x + (target[1] + 0.5) * res, info.origin_y + (target[0] + 0.5) * res
    if (tx - robot_xy[0]) * cos_h + (ty - robot_xy[1]) * sin_h > -settings.cut_offset:
        return []  # nearest passable cell is not behind the cut (cut sealed nothing)
    lap_steps = int(distance[target])
    if lap_steps * res < settings.min_loop_length:
        return []

    xs, ys = _cell_centres(np.ones_like(safe), info)
    xs, ys = xs.reshape(safe.shape), ys.reshape(safe.shape)

    free = (grid >= 0) & (grid < settings.free_threshold)
    wall_distance = distance_transform_edt(free)
    spacing_steps = max(2, int(round(settings.waypoint_spacing / res)))
    points: List[Tuple[float, float]] = []
    for level in range(spacing_steps, lap_steps - spacing_steps // 2 + 1, spacing_steps):
        band = (distance >= level - 1) & (distance <= level + 1)
        if not band.any():
            continue
        flat = int(np.argmax(np.where(band, wall_distance, -1.0)))
        row, col = divmod(flat, safe.shape[1])
        points.append((float(xs[row, col]), float(ys[row, col])))
    if len(points) < 2:
        return []
    waypoints = []
    for i, (x, y) in enumerate(points):
        nx, ny = points[i + 1] if i + 1 < len(points) else robot_xy
        waypoints.append((x, y, math.atan2(ny - y, nx - x)))
    return waypoints


def loop_is_clockwise(waypoints: List[Waypoint]) -> bool:
    """Orientation of the lap polygon (negative signed area = clockwise)."""
    points = [(x, y) for x, y, _ in waypoints]
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]))
    return area < 0.0


def plan_exploration_goal(
    grid: np.ndarray, info: GridInfo, robot_xy: Tuple[float, float], settings: LoopSettings,
    heading: Optional[float] = None,
) -> Optional[Waypoint]:
    """Next exploration goal; None if nothing is `min_spacing` away.

    Safe cells keep `clearance` from unknown space, so goals stay inside the known map.
    Without `heading`: the farthest reachable safe cell (straight-line distance).
    With `heading` (direction of travel): the track is cut behind the robot and the goal is the track centre
    line `max_exploration_distance` ahead in *driving* distance (or the frontier, if nearer): among cells at that
    driving distance, the one farthest from walls and unknown space. Following the centre line, the robot takes
    hairpins smoothly instead of driving into corner pockets, and a cell that is close in a straight line but
    only reachable by turning back never wins.
    """
    if grid.ndim != 2 or info.resolution <= 0.0:
        return None
    res = info.resolution
    clearance_cells = max(1, math.ceil(settings.clearance / res))
    safe = safe_free_space(grid, clearance_cells, settings.free_threshold)
    if heading is not None:
        for row, col in rear_gate_cells(grid, info, robot_xy, heading, offset=settings.cut_offset,
                                        max_half_width=settings.cut_half_width):
            safe[row, col] = False
    seed = _nearest_cell(safe, info, robot_xy)
    if seed is None:
        return None
    if heading is None:
        reachable = reachable_from(safe, seed)
        xs, ys = _cell_centres(reachable, info)
        distances = np.hypot(xs - robot_xy[0], ys - robot_xy[1])
        if settings.max_exploration_distance > 0.0:
            distances = np.where(distances <= settings.max_exploration_distance, distances, -1.0)
        if distances.size == 0:
            return None
        best = int(np.argmax(distances))
        if distances[best] < settings.min_spacing:
            return None
        x, y = float(xs[best]), float(ys[best])
        return x, y, math.atan2(y - robot_xy[1], x - robot_xy[0])

    steps = driving_distance(safe, seed)
    rows, cols = np.nonzero(steps >= 0)
    xs = info.origin_x + (cols + 0.5) * res
    ys = info.origin_y + (rows + 0.5) * res
    # Cells behind the cut but still connected (an unsealed cut end) are never goals.
    ahead = (xs - robot_xy[0]) * math.cos(heading) + (ys - robot_xy[1]) * math.sin(heading) > 0.0
    reach = np.where(ahead, steps[rows, cols], -1)
    if reach.max() * res < settings.min_spacing:
        return None
    target = int(reach.max())
    if settings.max_exploration_distance > 0.0:
        target = min(target, int(round(settings.max_exploration_distance / res)))
    band = np.abs(reach - target) <= 1
    known_free = (grid >= 0) & (grid < settings.free_threshold)
    clearance = distance_transform_edt(known_free)[rows, cols]
    best = int(np.argmax(np.where(band, clearance, -1.0)))
    x, y = float(xs[best]), float(ys[best])
    return x, y, math.atan2(y - robot_xy[1], x - robot_xy[0])


def _reachable_mask(
    grid: np.ndarray, info: GridInfo, robot_xy: Tuple[float, float], settings: LoopSettings
) -> Optional[np.ndarray]:
    """Safe cells connected to the robot, or None when there are none."""
    if grid.ndim != 2 or info.resolution <= 0.0:
        return None
    clearance_cells = max(1, math.ceil(settings.clearance / info.resolution))
    safe = safe_free_space(grid, clearance_cells, settings.free_threshold)
    seed = _nearest_cell(safe, info, robot_xy)
    if seed is None:
        return None
    return reachable_from(safe, seed)


def _cell_centres(mask: np.ndarray, info: GridInfo) -> Tuple[np.ndarray, np.ndarray]:
    rows, cols = np.nonzero(mask)
    return info.origin_x + (cols + 0.5) * info.resolution, info.origin_y + (rows + 0.5) * info.resolution


def _cell_of(info: GridInfo, xy: Tuple[float, float]) -> Tuple[int, int]:
    return int(math.floor((xy[1] - info.origin_y) / info.resolution)), int(
        math.floor((xy[0] - info.origin_x) / info.resolution))


def _reachable_points(
    grid: np.ndarray, info: GridInfo, robot_xy: Tuple[float, float], settings: LoopSettings
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Map-frame centres of safe cells connected to the robot, or None when there are none."""
    mask = _reachable_mask(grid, info, robot_xy, settings)
    return None if mask is None else _cell_centres(mask, info)


def _nearest_cell(mask: np.ndarray, info: GridInfo, xy: Tuple[float, float]) -> Optional[Tuple[int, int]]:
    rows, cols = np.nonzero(mask)
    if rows.size == 0:
        return None
    robot_col = (xy[0] - info.origin_x) / info.resolution - 0.5
    robot_row = (xy[1] - info.origin_y) / info.resolution - 0.5
    nearest = int(np.argmin((rows - robot_row) ** 2 + (cols - robot_col) ** 2))
    return int(rows[nearest]), int(cols[nearest])


@dataclass(frozen=True)
class StartLine:
    """Line through the start position, perpendicular to the start heading (the race direction)."""

    x: float
    y: float
    heading: float

    def local(self, xy: Tuple[float, float]) -> Tuple[float, float]:
        """(along, lateral): signed distance ahead of the line and offset along it."""
        dx, dy = xy[0] - self.x, xy[1] - self.y
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        return dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h


class StartLineMonitor:
    """Detects start-line crossings with hysteresis.

    The side of the line is tracked only inside the start zone (|lateral| <= half_width, |along| <= depth), so
    the far side of the track, which lies on the same infinite line, is ignored. A side only changes once the
    robot is more than `margin` past the line, so localisation jitter at the start is not a crossing. The robot
    begins on the forward side: backing out of the start position counts as a reverse crossing.
    """

    def __init__(self, line: StartLine, half_width: float = 1.0, depth: float = 1.0, margin: float = 0.10):
        self.line = line
        self.half_width = half_width
        self.depth = depth
        self.margin = margin
        # Optional wall-aware zone test (see start_zone_mask): positions inside the box but behind a wall,
        # e.g. a parallel passage on the other side of a thin wall, are not part of the start zone.
        self.zone: Optional[Callable[[Tuple[float, float]], bool]] = None
        self._side: Optional[int] = 1

    def update(self, xy: Tuple[float, float]) -> int:
        """+1 for a forward crossing (lap completed), -1 for a reverse crossing, 0 otherwise."""
        along, lateral = self.line.local(xy)
        if abs(lateral) > self.half_width or abs(along) > self.depth or (self.zone is not None and not self.zone(xy)):
            self._side = None
            return 0
        if abs(along) <= self.margin:
            return 0
        side = 1 if along > 0.0 else -1
        previous, self._side = self._side, side
        if previous is None or previous == side:
            return 0
        return side


def start_ahead(waypoints: List[Waypoint], xy: Tuple[float, float], heading: float) -> List[Waypoint]:
    """Rotate the loop so its first waypoint lies ahead of the robot, never just behind it."""
    if len(waypoints) < 2:
        return waypoints
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    first = waypoints[0]
    if (first[0] - xy[0]) * cos_h + (first[1] - xy[1]) * sin_h >= 0.0:
        return waypoints
    return waypoints[1:] + waypoints[:1]


def rear_gate_cells(
    grid: np.ndarray, info: GridInfo, xy: Tuple[float, float], direction: float,
    offset: float = 0.4, max_half_width: float = 1.5, occupied_threshold: int = 50,
) -> List[Tuple[int, int]]:
    return _rear_gate(grid, info, xy, direction, offset, max_half_width, occupied_threshold)[0]


def _rear_gate(
    grid: np.ndarray, info: GridInfo, xy: Tuple[float, float], direction: float,
    offset: float, max_half_width: float, occupied_threshold: int,
) -> Tuple[List[Tuple[int, int]], bool]:
    """Cells of a wall across the track `offset` metres behind the robot (opposite to `direction`).

    The line runs perpendicular to `direction` from the gate centre in both directions until it reaches an
    occupied cell (the track walls) or `max_half_width`; each sample also marks its four neighbours, so the
    wall stays sealed against 4- and 8-connected planners. Fed to the global costmap as a keepout zone, it cuts
    the ring behind the robot: every plan must go forward, so the robot never turns back along the track.
    """
    if grid.ndim != 2 or info.resolution <= 0.0:
        return [], False
    cos_d, sin_d = math.cos(direction), math.sin(direction)
    centre = (xy[0] - offset * cos_d, xy[1] - offset * sin_d)
    step = 0.5 * info.resolution
    cells = set()
    walls_hit = 0
    for side in (1.0, -1.0):
        distance = 0.0
        while distance <= max_half_width:
            px = centre[0] - side * distance * sin_d
            py = centre[1] + side * distance * cos_d
            row, col = _cell_of(info, (px, py))
            if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
                break
            for d_row, d_col in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
                t_row, t_col = row + d_row, col + d_col
                if 0 <= t_row < grid.shape[0] and 0 <= t_col < grid.shape[1]:
                    cells.add((t_row, t_col))
            if grid[row, col] >= occupied_threshold:
                walls_hit += 1
                break
            distance += step
    # Sealed: both ends of the line reached a wall, so nothing can drive around the cut.
    return sorted(cells), walls_hit == 2


def drop_behind(waypoints: List[Waypoint], xy: Tuple[float, float], heading: float, radius: float) -> List[Waypoint]:
    """Remove waypoints within `radius` behind the robot: they sit between it and the rear gate and could only be
    reached by turning back."""
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    return [w for w in waypoints
            if not ((w[0] - xy[0]) * cos_h + (w[1] - xy[1]) * sin_h < 0.0 and math.dist(w[:2], xy) < radius)]


def start_zone_mask(
    grid: np.ndarray, info: GridInfo, line: StartLine, half_width: float, depth: float, free_threshold: int = 25,
) -> Optional[np.ndarray]:
    """Free cells of the start box that connect to the start position without crossing a wall.

    On a compact track a parallel passage can run within the start box on the other side of a thin wall (the
    Fibonacci arena's U interior runs 0.3-0.4 m from the start line, westwards); without this test, driving
    through it reads as a reverse crossing.
    """
    if grid.ndim != 2 or info.resolution <= 0.0:
        return None
    xs, ys = _cell_centres(np.ones(grid.shape, dtype=bool), info)
    xs, ys = xs.reshape(grid.shape), ys.reshape(grid.shape)
    along, lateral = line.local((xs, ys))
    box = (np.abs(along) <= depth) & (np.abs(lateral) <= half_width)
    free = (grid >= 0) & (grid < free_threshold) & box
    seed = _nearest_cell(free, info, (line.x, line.y))
    if seed is None:
        return None
    return reachable_from(free, seed)


def mask_lookup(mask: np.ndarray, info: GridInfo) -> Callable[[Tuple[float, float]], bool]:
    def inside(xy: Tuple[float, float]) -> bool:
        row, col = _cell_of(info, xy)
        return 0 <= row < mask.shape[0] and 0 <= col < mask.shape[1] and bool(mask[row, col])
    return inside
