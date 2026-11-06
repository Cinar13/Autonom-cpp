"""Derive a closed patrol loop from the reachable free space of a growing occupancy grid.

The explored region is split into equal angular sectors around its centroid; in each sector the
reachable cell farthest from the centroid (while keeping `clearance` from obstacles and unknown
space) becomes a waypoint. As slam_toolbox expands the map, successive laps grow outward.
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
    if grid.ndim != 2 or info.resolution <= 0.0 or settings.waypoint_count < 3:
        return []

    clearance_cells = max(1, math.ceil(settings.clearance / info.resolution))
    safe = safe_free_space(grid, clearance_cells, settings.free_threshold)
    seed = _nearest_cell(safe, info, robot_xy)
    if seed is None:
        return []

    rows, cols = np.nonzero(reachable_from(safe, seed))
    xs = info.origin_x + (cols + 0.5) * info.resolution
    ys = info.origin_y + (rows + 0.5) * info.resolution
    centre_x, centre_y = float(xs.mean()), float(ys.mean())
    radii = np.hypot(xs - centre_x, ys - centre_y)
    angles = np.arctan2(ys - centre_y, xs - centre_x)

    if settings.max_radius > 0.0:
        radii = np.where(radii <= settings.max_radius, radii, -1.0)

    sectors = np.minimum(
        ((angles + math.pi) / (2.0 * math.pi) * settings.waypoint_count).astype(int), settings.waypoint_count - 1
    )
    points: List[Tuple[float, float]] = []
    for sector in range(settings.waypoint_count):
        sector_radii = np.where(sectors == sector, radii, -1.0)
        best = int(np.argmax(sector_radii))
        if sector_radii[best] >= 0.0:
            points.append((float(xs[best]), float(ys[best])))

    points = _enforce_spacing(points, settings.min_spacing)
    if len(points) < 3:
        return []

    start = min(range(len(points)), key=lambda i: math.dist(points[i], robot_xy))
    points = points[start:] + points[:start]
    return [
        (x, y, math.atan2(points[(i + 1) % len(points)][1] - y, points[(i + 1) % len(points)][0] - x))
        for i, (x, y) in enumerate(points)
    ]


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
