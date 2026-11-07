"""Derive a closed patrol loop from the reachable free space of a growing occupancy grid.

The explored region is split into equal angular sectors around its centroid; in each sector the
reachable cell farthest from the centroid (while keeping `clearance` from obstacles and unknown
space) becomes a waypoint. As slam_toolbox expands the map, successive laps grow outward.
A loop is only returned once every sector holds a waypoint, i.e. the explored space surrounds the centroid
(a closed track); on a partial map such as one L-shaped corridor a "loop" would need U-turns in the corridor.
Until then `plan_exploration_goal` pushes the map frontier out ahead of the robot.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

import numpy as np

Waypoint = Tuple[float, float, float]


@dataclass(frozen=True)
class GridInfo:
    resolution: float
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class LoopSettings:
    waypoint_count: int = 8
    clearance: float = 0.35
    max_radius: float = 0.0
    min_spacing: float = 0.5
    free_threshold: int = 25
    clockwise: bool = False
    # Waypoint per sector: the cell at the median distance from the centroid (track centre line, so the
    # shortest path between waypoints may cut the inside of corners) instead of the farthest one (outer wall).
    centre_line: bool = True
    # A loop is only planned once the reachable track encloses its centroid (closed ring). Otherwise a partial
    # map wrapped around the centroid (U or L shape) yields a "loop" that turns 180 degrees at a dead end.
    require_closed_track: bool = True
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


def plan_loop(
    grid: np.ndarray, info: GridInfo, robot_xy: Tuple[float, float], settings: LoopSettings
) -> List[Waypoint]:
    """Return loop waypoints (x, y, yaw) in the map frame, starting nearest the robot; empty if none fit."""
    if settings.waypoint_count < 3:
        return []
    mask = _reachable_mask(grid, info, robot_xy, settings)
    if mask is None:
        return []
    xs, ys = _cell_centres(mask, info)
    centre_x, centre_y = float(xs.mean()), float(ys.mean())
    if settings.require_closed_track and not encloses(mask, _cell_of(info, (centre_x, centre_y))):
        return []
    radii = np.hypot(xs - centre_x, ys - centre_y)
    angles = np.arctan2(ys - centre_y, xs - centre_x)

    if settings.max_radius > 0.0:
        radii = np.where(radii <= settings.max_radius, radii, -1.0)

    sectors = np.minimum(
        ((angles + math.pi) / (2.0 * math.pi) * settings.waypoint_count).astype(int), settings.waypoint_count - 1
    )
    points: List[Tuple[float, float]] = []
    for sector in range(settings.waypoint_count):
        in_sector = np.flatnonzero((sectors == sector) & (radii >= 0.0))
        if in_sector.size == 0:
            return []
        sector_radii = radii[in_sector]
        target = float(np.median(sector_radii)) if settings.centre_line else float(sector_radii.max())
        best = int(in_sector[np.argmin(np.abs(sector_radii - target))])
        points.append((float(xs[best]), float(ys[best])))

    points = _enforce_spacing(points, settings.min_spacing)
    if len(points) < 3:
        return []
    # Sectors run counter-clockwise around the centroid; laps only score in the designated direction (AC-5.1).
    if settings.clockwise:
        points.reverse()

    start = min(range(len(points)), key=lambda i: math.dist(points[i], robot_xy))
    points = points[start:] + points[:start]
    return [
        (x, y, math.atan2(points[(i + 1) % len(points)][1] - y, points[(i + 1) % len(points)][0] - x))
        for i, (x, y) in enumerate(points)
    ]


def plan_exploration_goal(
    grid: np.ndarray, info: GridInfo, robot_xy: Tuple[float, float], settings: LoopSettings,
    heading: Optional[float] = None,
) -> Optional[Waypoint]:
    """Farthest reachable safe cell from the robot, facing away from it; None if nothing is `min_spacing` away.

    Safe cells keep `clearance` from unknown space, so the farthest one sits just inside the map frontier.
    With `heading` (robot yaw), cells in front of the robot are preferred so it keeps driving forward along
    the track instead of turning back in a corridor; cells behind are used only when nothing ahead qualifies.
    """
    reachable = _reachable_points(grid, info, robot_xy, settings)
    if reachable is None:
        return None
    xs, ys = reachable
    distances = np.hypot(xs - robot_xy[0], ys - robot_xy[1])
    if settings.max_exploration_distance > 0.0:
        distances = np.where(distances <= settings.max_exploration_distance, distances, -1.0)
    if heading is not None:
        ahead = (xs - robot_xy[0]) * math.cos(heading) + (ys - robot_xy[1]) * math.sin(heading) > 0.0
        if np.any(ahead & (distances >= settings.min_spacing)):
            distances = np.where(ahead, distances, -1.0)
    best = int(np.argmax(distances))
    if distances[best] < settings.min_spacing:
        return None
    x, y = float(xs[best]), float(ys[best])
    return x, y, math.atan2(y - robot_xy[1], x - robot_xy[0])


def encloses(region: np.ndarray, cell: Tuple[int, int]) -> bool:
    """Whether `region` (4-connected) fully surrounds `cell`: the cell is outside the region and a
    4-connected flood fill through the rest of the grid cannot reach the grid border from it."""
    row, col = cell
    if not (0 <= row < region.shape[0] and 0 <= col < region.shape[1]) or region[row, col]:
        return False
    outside = reachable_from(~region, (row, col))
    return not (outside[0, :].any() or outside[-1, :].any() or outside[:, 0].any() or outside[:, -1].any())


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


def _enforce_spacing(points: List[Tuple[float, float]], min_spacing: float) -> List[Tuple[float, float]]:
    spaced: List[Tuple[float, float]] = []
    for point in points:
        if not spaced or math.dist(point, spaced[-1]) >= min_spacing:
            spaced.append(point)
    if len(spaced) > 1 and math.dist(spaced[0], spaced[-1]) < min_spacing:
        spaced.pop()
    return spaced


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
        self._side: Optional[int] = 1

    def update(self, xy: Tuple[float, float]) -> int:
        """+1 for a forward crossing (lap completed), -1 for a reverse crossing, 0 otherwise."""
        along, lateral = self.line.local(xy)
        if abs(lateral) > self.half_width or abs(along) > self.depth:
            self._side = None
            return 0
        if abs(along) <= self.margin:
            return 0
        side = 1 if along > 0.0 else -1
        previous, self._side = self._side, side
        if previous is None or previous == side:
            return 0
        return side


def loop_follows_heading(waypoints: List[Waypoint], xy: Tuple[float, float], heading: float) -> bool:
    """Whether the loop, near `xy`, runs in the direction of `heading`."""
    if len(waypoints) < 3:
        return True
    points = [(x, y) for x, y, _ in waypoints]
    i = min(range(len(points)), key=lambda k: math.dist(points[k], xy))
    after, before = points[(i + 1) % len(points)], points[i - 1]
    return (after[0] - before[0]) * math.cos(heading) + (after[1] - before[1]) * math.sin(heading) >= 0.0


def start_ahead(waypoints: List[Waypoint], xy: Tuple[float, float], heading: float) -> List[Waypoint]:
    """Rotate the loop so its first waypoint lies ahead of the robot, never just behind it."""
    if len(waypoints) < 2:
        return waypoints
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    first = waypoints[0]
    if (first[0] - xy[0]) * cos_h + (first[1] - xy[1]) * sin_h >= 0.0:
        return waypoints
    return waypoints[1:] + waypoints[:1]
