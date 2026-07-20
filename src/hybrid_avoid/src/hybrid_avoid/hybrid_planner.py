#!/usr/bin/env python3
"""Pure planning core for rolling-costmap A-star avoidance.

The module deliberately has no ROS imports so collision checking, A-star,
smoothing and CSV-zone semantics can be unit-tested without a running master.
"""

import bisect
import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]
Cell = Tuple[int, int]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PathProjection:
    s: float
    d: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VehicleGeometry:
    length: float = 0.80
    width: float = 0.70
    lidar_x: float = 0.40
    lidar_y: float = 0.0
    lidar_z: float = 0.50

    @property
    def half_length(self) -> float:
        return 0.5 * self.length

    @property
    def half_width(self) -> float:
        return 0.5 * self.width

    @property
    def half_diagonal(self) -> float:
        return math.hypot(self.half_length, self.half_width)


class ReferencePath:
    """Piecewise-linear path parameterized by cumulative arc length."""

    def __init__(self, points: Sequence[Point]):
        cleaned: List[Point] = []
        for value in points:
            point = (float(value[0]), float(value[1]))
            if not cleaned or math.hypot(point[0] - cleaned[-1][0], point[1] - cleaned[-1][1]) > 1e-4:
                cleaned.append(point)
        if len(cleaned) < 2:
            raise ValueError("reference path requires at least two different points")
        self.points = cleaned
        self.cumulative_s = [0.0]
        for index in range(1, len(cleaned)):
            self.cumulative_s.append(
                self.cumulative_s[-1]
                + math.hypot(cleaned[index][0] - cleaned[index - 1][0], cleaned[index][1] - cleaned[index - 1][1])
            )

    @property
    def length(self) -> float:
        return self.cumulative_s[-1]

    def sample(self, s_value: float) -> Tuple[float, float, float]:
        s_value = clamp(s_value, 0.0, self.length)
        index = bisect.bisect_right(self.cumulative_s, s_value) - 1
        index = min(max(index, 0), len(self.points) - 2)
        s0 = self.cumulative_s[index]
        s1 = self.cumulative_s[index + 1]
        x0, y0 = self.points[index]
        x1, y1 = self.points[index + 1]
        ratio = (s_value - s0) / max(s1 - s0, 1e-9)
        return (
            x0 + ratio * (x1 - x0),
            y0 + ratio * (y1 - y0),
            math.atan2(y1 - y0, x1 - x0),
        )

    def project(self, x: float, y: float, min_s: float = 0.0, max_s: Optional[float] = None) -> PathProjection:
        maximum = self.length if max_s is None else clamp(max_s, 0.0, self.length)
        minimum = clamp(min_s, 0.0, maximum)
        start = max(0, bisect.bisect_right(self.cumulative_s, minimum) - 2)
        end = min(len(self.points) - 2, bisect.bisect_left(self.cumulative_s, maximum) + 1)
        best: Optional[PathProjection] = None
        best_distance = float("inf")
        for index in range(start, end + 1):
            ax, ay = self.points[index]
            bx, by = self.points[index + 1]
            vx, vy = bx - ax, by - ay
            length_sq = vx * vx + vy * vy
            if length_sq < 1e-12:
                continue
            ratio = clamp(((x - ax) * vx + (y - ay) * vy) / length_sq, 0.0, 1.0)
            px, py = ax + ratio * vx, ay + ratio * vy
            dx, dy = x - px, y - py
            distance = dx * dx + dy * dy
            if distance >= best_distance:
                continue
            yaw = math.atan2(vy, vx)
            best = PathProjection(
                self.cumulative_s[index] + ratio * math.sqrt(length_sq),
                -math.sin(yaw) * dx + math.cos(yaw) * dy,
                px,
                py,
                yaw,
            )
            best_distance = distance
        if best is not None:
            return best
        px, py, yaw = self.sample(minimum)
        return PathProjection(minimum, 0.0, px, py, yaw)

    def segment(self, start_s: float, end_s: float, step: float = 0.10) -> List[Point]:
        start_s = clamp(start_s, 0.0, self.length)
        end_s = clamp(end_s, start_s, self.length)
        output: List[Point] = []
        s_value = start_s
        while s_value < end_s - 1e-6:
            x, y, _ = self.sample(s_value)
            output.append((x, y))
            s_value += max(0.02, step)
        x, y, _ = self.sample(end_s)
        output.append((x, y))
        return output


def smooth_reference_points(points: Sequence[Point], window: int = 5) -> List[Point]:
    if len(points) < 3 or window <= 1:
        return [(float(x), float(y)) for x, y in points]
    radius = max(1, int(window) // 2)
    output: List[Point] = []
    for index, point in enumerate(points):
        if index in (0, len(points) - 1):
            output.append((float(point[0]), float(point[1])))
            continue
        values = points[max(0, index - radius) : min(len(points), index + radius + 1)]
        output.append((sum(p[0] for p in values) / len(values), sum(p[1] for p in values) / len(values)))
    return output


def extract_avoidance_zones(
    reference: ReferencePath,
    raw_points: Sequence[Point],
    tasks: Sequence[str],
    start_task: str = "avoid_start",
    end_task: str = "avoid_end",
) -> List[Tuple[float, float]]:
    zones: List[Tuple[float, float]] = []
    active_start: Optional[float] = None
    search_s = 0.0
    for point, task_value in zip(raw_points, tasks):
        task = str(task_value or "none").strip().lower()
        projection = reference.project(point[0], point[1], max(0.0, search_s - 0.5), reference.length)
        search_s = max(search_s, projection.s)
        if task == start_task:
            if active_start is not None:
                raise ValueError("nested avoid_start markers are not allowed")
            active_start = search_s
        elif task == end_task:
            if active_start is None:
                raise ValueError("avoid_end appears before avoid_start")
            if search_s - active_start < 0.5:
                raise ValueError("avoidance zone is shorter than 0.5 m")
            zones.append((active_start, search_s))
            active_start = None
    if active_start is not None:
        raise ValueError("avoid_start has no following avoid_end")
    return zones


@dataclass
class CostmapConfig:
    size_x: float = 10.0
    size_y: float = 10.0
    resolution: float = 0.10
    collision_margin: float = 0.12
    soft_inflation: float = 0.25
    cost_weight: float = 2.0
    reference_weight: float = 0.35
    lane_half_width: float = 0.0
    lane_boundary_margin: float = 0.08
    fallback_center_limit: float = 1.80
    goal_search_distance: float = 2.00
    goal_search_step: float = 0.10
    minimum_goal_distance: float = 1.20


class RollingCostmap:
    FREE = 0
    LETHAL = 100

    def __init__(self, center_x: float, center_y: float, config: CostmapConfig):
        self.config = config
        self.resolution = max(0.04, config.resolution)
        self.width = max(10, int(math.ceil(config.size_x / self.resolution)))
        self.height = max(10, int(math.ceil(config.size_y / self.resolution)))
        self.origin_x = center_x - 0.5 * self.width * self.resolution
        self.origin_y = center_y - 0.5 * self.height * self.resolution
        self.data = [self.FREE] * (self.width * self.height)

    def index(self, cell: Cell) -> int:
        return cell[1] * self.width + cell[0]

    def inside(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        cell = (int(math.floor((x - self.origin_x) / self.resolution)), int(math.floor((y - self.origin_y) / self.resolution)))
        return cell if self.inside(cell) else None

    def cell_to_world(self, cell: Cell) -> Point:
        return (
            self.origin_x + (cell[0] + 0.5) * self.resolution,
            self.origin_y + (cell[1] + 0.5) * self.resolution,
        )

    def cost(self, cell: Cell) -> int:
        return self.data[self.index(cell)] if self.inside(cell) else self.LETHAL

    def set_cost(self, cell: Cell, value: int):
        if self.inside(cell):
            index = self.index(cell)
            self.data[index] = max(self.data[index], int(clamp(value, 0, self.LETHAL)))

    def add_obstacles(self, points: Iterable[Point], lethal_radius: float, inflation_radius: float):
        total_radius = max(lethal_radius, inflation_radius)
        cell_radius = int(math.ceil(total_radius / self.resolution))
        for x, y in points:
            center = self.world_to_cell(x, y)
            if center is None:
                continue
            for dx in range(-cell_radius, cell_radius + 1):
                for dy in range(-cell_radius, cell_radius + 1):
                    cell = (center[0] + dx, center[1] + dy)
                    if not self.inside(cell):
                        continue
                    wx, wy = self.cell_to_world(cell)
                    distance = math.hypot(wx - x, wy - y)
                    if distance <= lethal_radius:
                        self.set_cost(cell, self.LETHAL)
                    elif distance <= total_radius:
                        ratio = (total_radius - distance) / max(total_radius - lethal_radius, 1e-6)
                        self.set_cost(cell, 1 + int(79.0 * ratio))

    def clear_vehicle_footprint(self, pose: Pose2D, vehicle: VehicleGeometry, padding: float = 0.03):
        """Clear returns from the area physically occupied by the current vehicle.

        A rolling costmap normally clears the robot footprint before planning.
        Without this operation, a few self returns or remembered points can
        inflate over the start cell and make every other planning cycle fail.
        Only the current body rectangle is cleared; obstacles in front of the
        bumper and beside the body remain untouched.
        """
        radius = vehicle.half_diagonal + max(0.0, padding)
        center = self.world_to_cell(pose.x, pose.y)
        if center is None:
            return
        cell_radius = int(math.ceil(radius / self.resolution)) + 1
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                cell = (center[0] + dx, center[1] + dy)
                if not self.inside(cell):
                    continue
                wx, wy = self.cell_to_world(cell)
                offset_x, offset_y = wx - pose.x, wy - pose.y
                x_body = cosine * offset_x + sine * offset_y
                y_body = -sine * offset_x + cosine * offset_y
                if (
                    abs(x_body) <= vehicle.half_length + padding
                    and abs(y_body) <= vehicle.half_width + padding
                ):
                    self.data[self.index(cell)] = self.FREE

    def add_lane_boundaries(self, reference: ReferencePath, progress_s: float, vehicle: VehicleGeometry):
        if self.config.lane_half_width > 0.0:
            center_limit = self.config.lane_half_width - vehicle.half_width - self.config.lane_boundary_margin
        else:
            # A zero lane width means that the legal boundary has not yet been
            # measured, not that A* may cross an entire room.  Keep a generous
            # fallback corridor so an isolated obstacle can still be passed,
            # while rejecting the observed 3--4 m escape paths.
            center_limit = self.config.fallback_center_limit
        if center_limit <= 0.0 and self.config.lane_half_width <= 0.0:
            return
        if center_limit <= 0.0:
            self.data = [self.LETHAL] * len(self.data)
            return
        min_s = max(0.0, progress_s - 1.5)
        max_s = min(reference.length, progress_s + self.config.size_x + 2.0)
        for gy in range(self.height):
            for gx in range(self.width):
                point = self.cell_to_world((gx, gy))
                projection = reference.project(point[0], point[1], min_s, max_s)
                if abs(projection.d) > center_limit:
                    self.set_cost((gx, gy), self.LETHAL)

    @staticmethod
    def _bresenham(start: Cell, end: Cell) -> List[Cell]:
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx - dy
        output = []
        while True:
            output.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            twice = 2 * error
            if twice > -dy:
                error -= dy
                x0 += sx
            if twice < dx:
                error += dx
                y0 += sy
        return output

    def line_free(self, start: Cell, end: Cell, maximum_cost: int = 99) -> bool:
        # Sample the continuous segment as well as the Bresenham cells.  Plain
        # Bresenham can miss a lethal cell touched only at a grid corner, which
        # is unacceptable when shortening a path around an inflated obstacle.
        if any(self.cost(cell) > maximum_cost for cell in self._bresenham(start, end)):
            return False
        first = self.cell_to_world(start)
        second = self.cell_to_world(end)
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        count = max(1, int(math.ceil(length / (0.5 * self.resolution))))
        for index in range(count + 1):
            ratio = index / count
            cell = self.world_to_cell(
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
            if cell is None or self.cost(cell) > maximum_cost:
                return False
        return True

    def nearest_free(self, cell: Cell, maximum_radius: int = 6) -> Optional[Cell]:
        if self.inside(cell) and self.cost(cell) < self.LETHAL:
            return cell
        for radius in range(1, maximum_radius + 1):
            candidates = []
            for dx in range(-radius, radius + 1):
                candidates.append((cell[0] + dx, cell[1] - radius))
                candidates.append((cell[0] + dx, cell[1] + radius))
            for dy in range(-radius + 1, radius):
                candidates.append((cell[0] - radius, cell[1] + dy))
                candidates.append((cell[0] + radius, cell[1] + dy))
            valid = [candidate for candidate in candidates if self.inside(candidate) and self.cost(candidate) < self.LETHAL]
            if valid:
                return min(valid, key=lambda value: (value[0] - cell[0]) ** 2 + (value[1] - cell[1]) ** 2)
        return None


@dataclass
class HybridPlanResult:
    status: str
    path: List[Point]
    costmap: RollingCostmap
    goal_s: float
    reason: str
    expanded: int = 0


class HybridAStarPlanner:
    def __init__(self, vehicle: VehicleGeometry, config: Optional[CostmapConfig] = None):
        self.vehicle = vehicle
        self.config = config or CostmapConfig()

    @staticmethod
    def _neighbors(cell: Cell) -> Iterable[Tuple[Cell, float]]:
        for dx, dy, cost in (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
        ):
            yield (cell[0] + dx, cell[1] + dy), cost

    def _astar(
        self,
        grid: RollingCostmap,
        start: Cell,
        goal: Cell,
        reference: ReferencePath,
        progress_s: float,
    ) -> Tuple[List[Cell], int]:
        queue = [(0.0, 0.0, start)]
        parents: Dict[Cell, Cell] = {}
        best_cost: Dict[Cell, float] = {start: 0.0}
        expanded = 0
        reference_cache: Dict[Cell, float] = {}
        min_s = max(0.0, progress_s - 1.0)
        max_s = min(reference.length, progress_s + self.config.size_x + 2.0)
        while queue:
            _, current_cost, current = heapq.heappop(queue)
            if current_cost > best_cost.get(current, float("inf")) + 1e-9:
                continue
            expanded += 1
            if current == goal:
                path = [current]
                while path[-1] != start:
                    path.append(parents[path[-1]])
                path.reverse()
                return path, expanded
            for neighbor, step_cost in self._neighbors(current):
                if not grid.inside(neighbor) or grid.cost(neighbor) >= grid.LETHAL:
                    continue
                dx, dy = neighbor[0] - current[0], neighbor[1] - current[1]
                if dx and dy:
                    if grid.cost((current[0] + dx, current[1])) >= grid.LETHAL:
                        continue
                    if grid.cost((current[0], current[1] + dy)) >= grid.LETHAL:
                        continue
                if neighbor not in reference_cache:
                    wx, wy = grid.cell_to_world(neighbor)
                    reference_cache[neighbor] = abs(reference.project(wx, wy, min_s, max_s).d)
                cell_penalty = self.config.cost_weight * grid.cost(neighbor) / 100.0
                reference_penalty = self.config.reference_weight * reference_cache[neighbor]
                candidate = current_cost + step_cost + cell_penalty + reference_penalty
                if candidate >= best_cost.get(neighbor, float("inf")):
                    continue
                best_cost[neighbor] = candidate
                parents[neighbor] = current
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(queue, (candidate + heuristic, candidate, neighbor))
        return [], expanded

    @staticmethod
    def _shortcut(cells: Sequence[Cell], grid: RollingCostmap) -> List[Cell]:
        if len(cells) < 3:
            return list(cells)
        output = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            candidate = len(cells) - 1
            while candidate > anchor + 1 and not grid.line_free(cells[anchor], cells[candidate]):
                candidate -= 1
            output.append(cells[candidate])
            anchor = candidate
        return output

    @staticmethod
    def _chaikin(points: Sequence[Point], passes: int = 2) -> List[Point]:
        output = list(points)
        for _ in range(max(0, passes)):
            if len(output) < 3:
                break
            refined = [output[0]]
            for first, second in zip(output[:-1], output[1:]):
                refined.append((0.75 * first[0] + 0.25 * second[0], 0.75 * first[1] + 0.25 * second[1]))
                refined.append((0.25 * first[0] + 0.75 * second[0], 0.25 * first[1] + 0.75 * second[1]))
            refined.append(output[-1])
            output = refined
        return output

    @staticmethod
    def _resample(points: Sequence[Point], step: float = 0.10) -> List[Point]:
        if len(points) < 2:
            return list(points)
        output = [points[0]]
        for first, second in zip(points[:-1], points[1:]):
            length = math.hypot(second[0] - first[0], second[1] - first[1])
            count = max(1, int(math.ceil(length / max(0.04, step))))
            for index in range(1, count + 1):
                ratio = index / count
                output.append((first[0] + ratio * (second[0] - first[0]), first[1] + ratio * (second[1] - first[1])))
        return output

    @staticmethod
    def _world_path_free(points: Sequence[Point], grid: RollingCostmap) -> bool:
        if not points:
            return False
        for first, second in zip(points[:-1], points[1:]):
            length = math.hypot(second[0] - first[0], second[1] - first[1])
            count = max(1, int(math.ceil(length / (0.5 * grid.resolution))))
            for index in range(count + 1):
                ratio = index / count
                cell = grid.world_to_cell(
                    first[0] + ratio * (second[0] - first[0]),
                    first[1] + ratio * (second[1] - first[1]),
                )
                if cell is None or grid.cost(cell) >= grid.LETHAL:
                    return False
        return True

    @classmethod
    def remaining_path_status(
        cls,
        points: Sequence[Point],
        pose: Pose2D,
        grid: RollingCostmap,
    ) -> Tuple[bool, float, str]:
        """Check the complete untravelled part of a previously accepted path."""
        if len(points) < 2:
            return False, 0.0, "no_previous_path"
        nearest = min(
            range(len(points)),
            key=lambda index: (points[index][0] - pose.x) ** 2 + (points[index][1] - pose.y) ** 2,
        )
        connector = math.hypot(points[nearest][0] - pose.x, points[nearest][1] - pose.y)
        if connector > 0.75:
            return False, 0.0, "previous_path_too_far"
        remaining = [(pose.x, pose.y)] + list(points[nearest:])
        distance = sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(remaining[:-1], remaining[1:])
        )
        if not cls._world_path_free(remaining, grid):
            return False, distance, "previous_path_occupied"
        return True, distance, "previous_path_clear"

    def _select_reference_goal(
        self,
        grid: RollingCostmap,
        reference: ReferencePath,
        progress_s: float,
        requested_goal_s: float,
    ) -> Tuple[Optional[float], Optional[Cell], Optional[Point], str]:
        """Select a free route point near the requested rolling goal.

        A wall or obstacle can temporarily cover the exact point 4.5 m ahead.
        The goal is therefore searched along the reference line instead of
        moving it sideways to an arbitrary free cell.  Forward candidates are
        preferred so A* can reconnect beyond an obstacle; backward candidates
        allow motion to continue until that forward point enters the costmap.
        """
        requested = clamp(requested_goal_s, progress_s, reference.length)
        step = max(grid.resolution, self.config.goal_search_step)
        count = int(math.ceil(max(0.0, self.config.goal_search_distance) / step))
        minimum = min(reference.length, progress_s + max(0.0, self.config.minimum_goal_distance))
        candidates = [requested]
        for index in range(1, count + 1):
            candidates.append(min(reference.length, requested + index * step))
            candidates.append(max(minimum, requested - index * step))

        checked = set()
        had_inside_candidate = False
        for candidate_s in candidates:
            # Quantise arc length as well as the duplicate key.  Values such as
            # 4.6 + 3 * 0.1 otherwise become 4.899999..., fall into the previous
            # grid cell and can falsely report an occupied goal at a boundary.
            candidate_s = round(clamp(candidate_s, progress_s, reference.length), 6)
            key = candidate_s
            if key in checked:
                continue
            checked.add(key)
            x_value, y_value, _ = reference.sample(candidate_s)
            cell = grid.world_to_cell(x_value, y_value)
            if cell is None:
                continue
            had_inside_candidate = True
            if grid.cost(cell) < grid.LETHAL:
                return candidate_s, cell, (x_value, y_value), "reference_goal_free"
        reason = "reference_goal_not_free" if had_inside_candidate else "reference_goal_outside_costmap"
        return None, None, None, reason

    def plan(
        self,
        pose: Pose2D,
        progress_s: float,
        reference: ReferencePath,
        obstacle_points: Sequence[Point],
        goal_s: float,
    ) -> HybridPlanResult:
        grid = RollingCostmap(pose.x, pose.y, self.config)
        lethal_radius = self.vehicle.half_diagonal + self.config.collision_margin
        grid.add_obstacles(obstacle_points, lethal_radius, lethal_radius + self.config.soft_inflation)
        grid.clear_vehicle_footprint(pose, self.vehicle)
        grid.add_lane_boundaries(reference, progress_s, self.vehicle)

        start = grid.world_to_cell(pose.x, pose.y)
        if start is None:
            return HybridPlanResult("blocked", [], grid, goal_s, "start_outside_costmap")
        if grid.cost(start) >= grid.LETHAL:
            return HybridPlanResult("blocked", [], grid, goal_s, "start_not_free")
        selected_goal_s, goal, goal_point, goal_reason = self._select_reference_goal(
            grid, reference, progress_s, goal_s
        )
        if selected_goal_s is None or goal is None or goal_point is None:
            return HybridPlanResult("blocked", [], grid, goal_s, goal_reason)
        cells, expanded = self._astar(grid, start, goal, reference, progress_s)
        if not cells:
            return HybridPlanResult("blocked", [], grid, selected_goal_s, "astar_no_path", expanded)

        full_points = [grid.cell_to_world(cell) for cell in cells]
        full_points[0] = (pose.x, pose.y)
        full_points[-1] = goal_point
        shortened = self._shortcut(cells, grid)
        shortcut_points = [grid.cell_to_world(cell) for cell in shortened]
        shortcut_points[0] = full_points[0]
        shortcut_points[-1] = full_points[-1]
        smoothed = self._resample(self._chaikin(shortcut_points, 2), max(0.06, self.config.resolution))
        if not self._world_path_free(smoothed, grid):
            smoothed = self._resample(full_points, max(0.06, self.config.resolution))
        if not self._world_path_free(smoothed, grid):
            return HybridPlanResult("blocked", [], grid, selected_goal_s, "smoothed_path_not_safe", expanded)
        return HybridPlanResult("path", smoothed, grid, selected_goal_s, "astar_costmap_path", expanded)


def base_pose_from_lidar_odometry(lidar_pose: Pose2D, vehicle: VehicleGeometry) -> Pose2D:
    cosine, sine = math.cos(lidar_pose.yaw), math.sin(lidar_pose.yaw)
    return Pose2D(
        lidar_pose.x - cosine * vehicle.lidar_x + sine * vehicle.lidar_y,
        lidar_pose.y - sine * vehicle.lidar_x - cosine * vehicle.lidar_y,
        lidar_pose.yaw,
    )


def lidar_point_to_world(point: Tuple[float, float, float], base_pose: Pose2D, vehicle: VehicleGeometry) -> Tuple[float, float, float]:
    x_base, y_base = point[0] + vehicle.lidar_x, point[1] + vehicle.lidar_y
    cosine, sine = math.cos(base_pose.yaw), math.sin(base_pose.yaw)
    return (
        base_pose.x + cosine * x_base - sine * y_base,
        base_pose.y + sine * x_base + cosine * y_base,
        point[2] + vehicle.lidar_z,
    )


def raw_lidar_clearances(
    points: Iterable[Tuple[float, float, float]],
    vehicle: VehicleGeometry,
    min_height: float,
    max_height: float,
    max_range: float,
    lateral_padding: float,
    minimum_points: int,
) -> Tuple[float, float]:
    front: List[float] = []
    radial: List[float] = []
    for x_lidar, y_lidar, z_lidar in points:
        if not all(math.isfinite(value) for value in (x_lidar, y_lidar, z_lidar)):
            continue
        if math.hypot(x_lidar, y_lidar) > max_range:
            continue
        x_base, y_base, z_base = x_lidar + vehicle.lidar_x, y_lidar + vehicle.lidar_y, z_lidar + vehicle.lidar_z
        if not min_height <= z_base <= max_height:
            continue
        if abs(x_base) <= vehicle.half_length + 0.05 and abs(y_base) <= vehicle.half_width + 0.05:
            continue
        radial.append(math.hypot(x_base, y_base))
        clearance = x_base - vehicle.half_length
        if clearance >= 0.0 and abs(y_base) <= vehicle.half_width + lateral_padding:
            front.append(clearance)
    front.sort()
    radial.sort()
    required = max(1, int(minimum_points))
    return (
        front[required - 1] if len(front) >= required else float("inf"),
        radial[required - 1] if len(radial) >= required else float("inf"),
    )
