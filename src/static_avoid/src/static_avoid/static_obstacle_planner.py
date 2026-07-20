#!/usr/bin/env python3
"""Reference-path-aware local planning for static obstacles.

The planner works in a path-relative coordinate system.  It samples smooth
left/right detours, checks the full Bunker footprint against obstacle points,
and reconnects to a forward point on the original reference path with zero
lateral offset.  This module intentionally has no ROS dependency so that the
geometry and planner can be unit-tested without hardware or roscore.
"""

import bisect
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]


def extract_avoidance_zones(
    reference,
    raw_points: Sequence[Point],
    tasks: Sequence[str],
    start_task: str = "avoid_start",
    end_task: str = "avoid_end",
) -> List[Tuple[float, float]]:
    """Convert paired CSV task markers into reference-path arc intervals."""
    zones: List[Tuple[float, float]] = []
    pending_start = None
    for point, raw_task in zip(raw_points, tasks):
        task = str(raw_task or "none").strip().lower()
        if task == start_task:
            if pending_start is not None:
                raise ValueError("Nested avoid_start markers are not allowed")
            pending_start = reference.project(point[0], point[1]).s
        elif task == end_task:
            if pending_start is None:
                raise ValueError("avoid_end appears before avoid_start")
            end_s = reference.project(point[0], point[1]).s
            if end_s <= pending_start + 0.50:
                raise ValueError("Avoidance zone must be longer than 0.50 m")
            zones.append((pending_start, end_s))
            pending_start = None
    if pending_start is not None:
        raise ValueError("avoid_start has no following avoid_end")
    return zones


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def smootherstep(value: float) -> float:
    """C2-continuous interpolation on [0, 1]."""
    u = clamp(value, 0.0, 1.0)
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def raw_lidar_safety_distances(
    points: Iterable[Tuple[float, float, float]],
    vehicle,
    min_height: float,
    max_height: float,
    max_range: float,
    lateral_padding: float,
    self_filter_padding: float,
    min_points: int,
) -> Tuple[float, float]:
    """Return robust front-bumper and radial clearances from raw lidar points."""
    front_clearances: List[float] = []
    radial_distances: List[float] = []
    lateral_limit = vehicle.half_width + lateral_padding
    for x_lidar, y_lidar, z_lidar in points:
        if not (math.isfinite(x_lidar) and math.isfinite(y_lidar) and math.isfinite(z_lidar)):
            continue
        if math.hypot(x_lidar, y_lidar) > max_range:
            continue
        x_base = x_lidar + vehicle.lidar_x
        y_base = y_lidar + vehicle.lidar_y
        z_base = z_lidar + vehicle.lidar_z
        if not min_height <= z_base <= max_height:
            continue
        if (
            abs(x_base) <= vehicle.half_length + self_filter_padding
            and abs(y_base) <= vehicle.half_width + self_filter_padding
        ):
            continue
        radial_distances.append(math.hypot(x_base, y_base))
        front_clearance = x_base - vehicle.half_length
        if front_clearance >= 0.0 and abs(y_base) <= lateral_limit:
            front_clearances.append(front_clearance)

    front_clearances.sort()
    radial_distances.sort()
    required = max(1, int(min_points))
    min_front = front_clearances[required - 1] if len(front_clearances) >= required else float("inf")
    min_radial = radial_distances[required - 1] if len(radial_distances) >= required else float("inf")
    return min_front, min_radial


def smooth_path_points(points: Sequence[Point], window: int = 5) -> List[Point]:
    """Remove centimetre-scale recorder jitter without changing the source CSV.

    The first and last points are preserved.  The function deliberately uses a
    short moving average rather than a global spline so it cannot create a large
    shortcut across a real corner.
    """
    values = [(float(point[0]), float(point[1])) for point in points]
    if len(values) < 3 or window <= 1:
        return values
    window = max(3, int(window))
    if window % 2 == 0:
        window += 1
    half = window // 2
    output: List[Point] = []
    for index in range(len(values)):
        start = max(0, index - half)
        end = min(len(values), index + half + 1)
        local = values[start:end]
        output.append(
            (
                sum(point[0] for point in local) / len(local),
                sum(point[1] for point in local) / len(local),
            )
        )
    output[0] = values[0]
    output[-1] = values[-1]
    return output


@dataclass(frozen=True)
class Pose2D:
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


@dataclass(frozen=True)
class PathProjection:
    s: float
    d: float
    x: float
    y: float
    yaw: float
    segment_index: int


@dataclass
class PlannerConfig:
    planning_horizon: float = 5.0
    path_sample_step: float = 0.10
    collision_margin: float = 0.16
    obstacle_check_extra: float = 0.05
    prepare_distance: float = 1.40
    obstacle_pass_distance: float = 0.55
    rejoin_distance: float = 1.80
    min_transition_length: float = 1.00
    transition_length_per_meter: float = 1.80
    rejoin_length_per_meter: float = 1.80
    lateral_offsets: Tuple[float, ...] = (0.65, 0.85, 1.05, 1.20)
    max_lateral_offset: float = 1.25
    lane_half_width: float = 0.0  # 0 disables the boundary constraint.
    lane_boundary_margin: float = 0.08
    max_curvature: float = 2.20
    min_clearance: float = 0.12
    weight_length: float = 1.0
    weight_deviation: float = 0.65
    weight_curvature: float = 0.35
    weight_curvature_change: float = 0.15
    weight_clearance: float = 0.55
    enable_multi_obstacle_lattice: bool = True
    lattice_lateral_step: float = 0.05
    lattice_smoothing_passes: int = 4
    lattice_weight_deviation: float = 0.30
    lattice_weight_slope: float = 1.20
    lattice_weight_acceleration: float = 10.0


@dataclass
class PlanResult:
    status: str
    points: List[Point]
    rejoin_s: float
    side: str = "none"
    lateral_offset: float = 0.0
    cost: float = float("inf")
    reason: str = ""
    blocked_start_s: float = 0.0
    blocked_end_s: float = 0.0


class ReferencePath:
    """Piecewise-linear reference path parameterized by arc length."""

    def __init__(self, points: Sequence[Point]):
        cleaned: List[Point] = []
        for value in points:
            point = (float(value[0]), float(value[1]))
            if not cleaned or math.hypot(point[0] - cleaned[-1][0], point[1] - cleaned[-1][1]) > 1e-4:
                cleaned.append(point)
        if len(cleaned) < 2:
            raise ValueError("Reference path requires at least two different points")

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
        segment_length = max(s1 - s0, 1e-9)
        ratio = (s_value - s0) / segment_length
        x = x0 + ratio * (x1 - x0)
        y = y0 + ratio * (y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        return x, y, yaw

    def xy_from_sd(self, s_value: float, d_value: float) -> Point:
        x, y, yaw = self.sample(s_value)
        return x - math.sin(yaw) * d_value, y + math.cos(yaw) * d_value

    def project(self, x: float, y: float, min_s: float = 0.0, max_s: Optional[float] = None) -> PathProjection:
        max_s = self.length if max_s is None else clamp(max_s, 0.0, self.length)
        min_s = clamp(min_s, 0.0, max_s)
        start_index = max(0, bisect.bisect_right(self.cumulative_s, min_s) - 2)
        end_index = min(len(self.points) - 2, bisect.bisect_left(self.cumulative_s, max_s) + 1)

        best = None
        best_distance_sq = float("inf")
        for index in range(start_index, end_index + 1):
            ax, ay = self.points[index]
            bx, by = self.points[index + 1]
            vx = bx - ax
            vy = by - ay
            length_sq = vx * vx + vy * vy
            if length_sq <= 1e-12:
                continue
            ratio = clamp(((x - ax) * vx + (y - ay) * vy) / length_sq, 0.0, 1.0)
            px = ax + ratio * vx
            py = ay + ratio * vy
            dx = x - px
            dy = y - py
            distance_sq = dx * dx + dy * dy
            if distance_sq >= best_distance_sq:
                continue
            segment_length = math.sqrt(length_sq)
            yaw = math.atan2(vy, vx)
            signed_d = -math.sin(yaw) * dx + math.cos(yaw) * dy
            best = PathProjection(
                s=self.cumulative_s[index] + ratio * segment_length,
                d=signed_d,
                x=px,
                y=py,
                yaw=yaw,
                segment_index=index,
            )
            best_distance_sq = distance_sq

        if best is None:
            x0, y0, yaw = self.sample(min_s)
            return PathProjection(min_s, 0.0, x0, y0, yaw, 0)
        return best

    def segment(self, start_s: float, end_s: float, step: float, start_pose: Optional[Pose2D] = None) -> List[Point]:
        start_s = clamp(start_s, 0.0, self.length)
        end_s = clamp(end_s, start_s, self.length)
        output: List[Point] = []
        if start_pose is not None:
            output.append((start_pose.x, start_pose.y))
        s_value = start_s
        while s_value < end_s - 1e-6:
            output.append(self.xy_from_sd(s_value, 0.0))
            s_value += max(step, 0.02)
        output.append(self.xy_from_sd(end_s, 0.0))
        return remove_near_duplicates(output)


def remove_near_duplicates(points: Sequence[Point], threshold: float = 0.015) -> List[Point]:
    output: List[Point] = []
    for point in points:
        if not output or math.hypot(point[0] - output[-1][0], point[1] - output[-1][1]) >= threshold:
            output.append((float(point[0]), float(point[1])))
    return output


class ObstacleIndex:
    """Small spatial hash used for footprint and clearance queries."""

    def __init__(self, points: Iterable[Point], cell_size: float = 0.35):
        self.cell_size = max(0.05, float(cell_size))
        self.cells: Dict[Tuple[int, int], List[Point]] = {}
        self.point_count = 0
        for point in points:
            value = (float(point[0]), float(point[1]))
            key = self._key(value[0], value[1])
            self.cells.setdefault(key, []).append(value)
            self.point_count += 1

    def _key(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor(x / self.cell_size)), int(math.floor(y / self.cell_size))

    def nearby(self, x: float, y: float, radius: float) -> Iterable[Point]:
        min_key = self._key(x - radius, y - radius)
        max_key = self._key(x + radius, y + radius)
        for gx in range(min_key[0], max_key[0] + 1):
            for gy in range(min_key[1], max_key[1] + 1):
                for point in self.cells.get((gx, gy), ()):
                    yield point


class StaticObstaclePlanner:
    def __init__(self, reference_path: ReferencePath, vehicle: VehicleGeometry, config: Optional[PlannerConfig] = None):
        self.reference = reference_path
        self.vehicle = vehicle
        self.config = config or PlannerConfig()

    def footprint_collision(self, pose: Pose2D, obstacles: ObstacleIndex, extra_margin: float = 0.0) -> bool:
        margin = self.config.collision_margin + max(0.0, extra_margin)
        query_radius = self.vehicle.half_diagonal + margin + 0.08
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        for ox, oy in obstacles.nearby(pose.x, pose.y, query_radius):
            dx = ox - pose.x
            dy = oy - pose.y
            xb = cosine * dx + sine * dy
            yb = -sine * dx + cosine * dy
            if abs(xb) <= self.vehicle.half_length + margin and abs(yb) <= self.vehicle.half_width + margin:
                return True
        return False

    def _blocked_interval(self, start_s: float, obstacles: ObstacleIndex) -> Optional[Tuple[float, float]]:
        if obstacles.point_count == 0:
            return None
        end_s = min(self.reference.length, start_s + self.config.planning_horizon)
        blocked_values: List[float] = []
        s_value = start_s
        while s_value <= end_s + 1e-6:
            x, y, yaw = self.reference.sample(s_value)
            if self.footprint_collision(
                Pose2D(x, y, yaw), obstacles, extra_margin=self.config.obstacle_check_extra
            ):
                blocked_values.append(s_value)
            s_value += max(0.04, self.config.path_sample_step)
        if not blocked_values:
            return None
        return blocked_values[0], blocked_values[-1]

    def _detour_points(
        self,
        current_pose: Pose2D,
        projection: PathProjection,
        blocked_start_s: float,
        blocked_end_s: float,
        lateral_offset: float,
    ) -> Tuple[List[Point], float]:
        cfg = self.config
        current_s = projection.s
        transition_length = max(
            cfg.min_transition_length,
            abs(lateral_offset) * cfg.transition_length_per_meter,
        )
        prepare_distance = max(cfg.prepare_distance, transition_length + 0.15)
        shift_start_s = max(current_s, blocked_start_s - prepare_distance)
        shift_end_s = shift_start_s + transition_length
        hold_end_s = max(shift_end_s, blocked_end_s + cfg.obstacle_pass_distance)
        return_length = max(cfg.rejoin_distance, abs(lateral_offset) * cfg.rejoin_length_per_meter)
        rejoin_s = min(self.reference.length, hold_end_s + return_length)

        if rejoin_s - hold_end_s < 0.45 or shift_end_s >= rejoin_s - 0.30:
            return [], rejoin_s

        points: List[Point] = [(current_pose.x, current_pose.y)]
        s_value = current_s
        step = max(0.04, cfg.path_sample_step)
        while s_value <= rejoin_s + 1e-6:
            if s_value <= shift_start_s:
                if shift_start_s <= current_s + 1e-6:
                    d_value = projection.d
                else:
                    ratio = (s_value - current_s) / max(shift_start_s - current_s, 1e-6)
                    d_value = projection.d * (1.0 - smootherstep(ratio))
            elif s_value < shift_end_s:
                ratio = (s_value - shift_start_s) / max(shift_end_s - shift_start_s, 1e-6)
                d_value = lateral_offset * smootherstep(ratio)
            elif s_value <= hold_end_s:
                d_value = lateral_offset
            else:
                ratio = (s_value - hold_end_s) / max(rejoin_s - hold_end_s, 1e-6)
                d_value = lateral_offset * (1.0 - smootherstep(ratio))
            points.append(self.reference.xy_from_sd(s_value, d_value))
            s_value += step
        points.append(self.reference.xy_from_sd(rejoin_s, 0.0))
        return remove_near_duplicates(points), rejoin_s

    @staticmethod
    def _path_yaws(points: Sequence[Point]) -> List[float]:
        if len(points) < 2:
            return []
        yaws = []
        for index in range(len(points) - 1):
            dx = points[index + 1][0] - points[index][0]
            dy = points[index + 1][1] - points[index][1]
            if math.hypot(dx, dy) < 1e-6:
                yaws.append(yaws[-1] if yaws else 0.0)
            else:
                yaws.append(math.atan2(dy, dx))
        yaws.append(yaws[-1])
        return yaws

    def _validate_and_score(
        self,
        points: Sequence[Point],
        obstacles: ObstacleIndex,
        lateral_offset: float,
    ) -> Tuple[bool, float, str]:
        if len(points) < 4:
            return False, float("inf"), "candidate_too_short"

        cfg = self.config
        allowed_lateral = float("inf")
        if cfg.lane_half_width > 0.0:
            allowed_lateral = max(
                0.0,
                cfg.lane_half_width - self.vehicle.half_width - cfg.lane_boundary_margin,
            )
            if abs(lateral_offset) > allowed_lateral:
                return False, float("inf"), "lane_boundary"

        yaws = self._path_yaws(points)
        length = 0.0
        deviation_cost = 0.0
        curvature_cost = 0.0
        curvature_change_cost = 0.0
        clearance_cost = 0.0
        previous_curvature = 0.0
        search_radius = self.vehicle.half_diagonal + cfg.collision_margin + 1.2

        for index, point in enumerate(points):
            pose = Pose2D(point[0], point[1], yaws[index])
            if self.footprint_collision(pose, obstacles):
                return False, float("inf"), "footprint_collision"

            projection = self.reference.project(point[0], point[1])
            if abs(projection.d) > allowed_lateral + 1e-6:
                return False, float("inf"), "lane_boundary"
            deviation_cost += projection.d * projection.d * cfg.path_sample_step

            nearest = float("inf")
            for ox, oy in obstacles.nearby(point[0], point[1], search_radius):
                nearest = min(nearest, math.hypot(ox - point[0], oy - point[1]))
            if nearest < float("inf"):
                center_clearance = nearest - self.vehicle.half_diagonal
                if center_clearance < cfg.min_clearance:
                    clearance_cost += 25.0 * (cfg.min_clearance - center_clearance)
                clearance_cost += 1.0 / max(0.08, center_clearance + 0.08)

            if index == 0:
                continue
            segment = math.hypot(point[0] - points[index - 1][0], point[1] - points[index - 1][1])
            length += segment
            if segment > 1e-5:
                curvature = abs(wrap_angle(yaws[index] - yaws[index - 1])) / segment
                if curvature > cfg.max_curvature:
                    return False, float("inf"), "curvature_limit"
                curvature_cost += curvature * curvature * segment
                curvature_change_cost += abs(curvature - previous_curvature)
                previous_curvature = curvature

        total = (
            cfg.weight_length * length
            + cfg.weight_deviation * deviation_cost
            + cfg.weight_curvature * curvature_cost
            + cfg.weight_curvature_change * curvature_change_cost
            + cfg.weight_clearance * clearance_cost * cfg.path_sample_step
        )
        return True, total, "ok"

    @staticmethod
    def _smooth_lateral_profile(values: Sequence[float], passes: int) -> List[float]:
        output = [float(value) for value in values]
        if len(output) < 7:
            return output
        for _ in range(max(0, passes)):
            previous = output
            output = list(previous)
            for index in range(2, len(previous) - 2):
                output[index] = (
                    previous[index - 2]
                    + 2.0 * previous[index - 1]
                    + 3.0 * previous[index]
                    + 2.0 * previous[index + 1]
                    + previous[index + 2]
                ) / 9.0
            output[0] = values[0]
            output[1] = values[1]
            output[-2] = values[-2]
            output[-1] = values[-1]
        return output

    def _multi_obstacle_lattice(
        self,
        current_pose: Pose2D,
        projection: PathProjection,
        obstacles: ObstacleIndex,
        blocked_end_s: float,
    ) -> Optional[PlanResult]:
        """Forward-only Frenet lattice used when a single-side detour fails.

        The state keeps the previous two lateral cells, which lets the dynamic
        program penalize both lateral slope and abrupt slope changes.  The full
        oriented Bunker footprint is checked again after reconstructing and
        smoothing the candidate.
        """
        cfg = self.config
        if not cfg.enable_multi_obstacle_lattice:
            return None
        s_step = max(0.08, cfg.path_sample_step)
        lateral_step = max(0.04, cfg.lattice_lateral_step)
        max_lateral = cfg.max_lateral_offset
        if cfg.lane_half_width > 0.0:
            max_lateral = min(
                max_lateral,
                max(0.0, cfg.lane_half_width - self.vehicle.half_width - cfg.lane_boundary_margin),
            )
        if max_lateral < lateral_step:
            return None

        current_s = projection.s
        # planning_horizon limits how far ahead the reference path is searched
        # for a blockage.  It must not also force the lattice to rejoin exactly
        # at that boundary: an obstacle near the edge still needs longitudinal
        # room for a smooth return to the recorded route.
        rejoin_buffer = max(
            cfg.rejoin_distance,
            cfg.max_lateral_offset * cfg.rejoin_length_per_meter,
        )
        end_s = min(
            self.reference.length,
            max(current_s + cfg.planning_horizon, blocked_end_s + rejoin_buffer),
        )
        if end_s - current_s < 2.0:
            return None
        sample_count = int(math.floor((end_s - current_s) / s_step)) + 1
        s_values = [current_s + index * s_step for index in range(sample_count)]
        if s_values[-1] < end_s - 0.02:
            s_values.append(end_s)

        cell_count = int(math.floor(max_lateral / lateral_step))
        lateral_values = [index * lateral_step for index in range(-cell_count, cell_count + 1)]
        zero_index = cell_count
        start_index = int(round(projection.d / lateral_step)) + zero_index
        start_index = min(max(start_index, 0), len(lateral_values) - 1)

        feasible: List[List[bool]] = []
        for s_value in s_values:
            _, _, reference_yaw = self.reference.sample(s_value)
            row = []
            for d_value in lateral_values:
                x, y = self.reference.xy_from_sd(s_value, d_value)
                row.append(not self.footprint_collision(Pose2D(x, y, reference_yaw), obstacles))
            feasible.append(row)
        if not feasible[0][start_index]:
            return None

        # state=(previous cell, current cell), value=(cost, complete cell path)
        states = {(start_index, start_index): (0.0, [start_index])}
        for s_index in range(1, len(s_values)):
            next_states = {}
            for (previous_previous, previous), (cost, path) in states.items():
                for current in range(max(0, previous - 1), min(len(lateral_values), previous + 2)):
                    if not feasible[s_index][current]:
                        continue
                    d_value = lateral_values[current]
                    lateral_delta = lateral_values[current] - lateral_values[previous]
                    acceleration = current - 2 * previous + previous_previous
                    next_cost = cost
                    next_cost += cfg.lattice_weight_deviation * d_value * d_value * s_step
                    next_cost += cfg.lattice_weight_slope * lateral_delta * lateral_delta / s_step
                    next_cost += cfg.lattice_weight_acceleration * (acceleration * lateral_step) ** 2
                    key = (previous, current)
                    if key not in next_states or next_cost < next_states[key][0]:
                        next_states[key] = (next_cost, path + [current])
            states = next_states
            if not states:
                return None

        goal_candidates = [
            value
            for (previous, current), value in states.items()
            if current == zero_index and previous == zero_index
        ]
        if not goal_candidates:
            return None
        _, best_cells = min(goal_candidates, key=lambda value: value[0])
        raw_lateral = [lateral_values[index] for index in best_cells]

        for smoothing_passes in range(cfg.lattice_smoothing_passes, -1, -1):
            lateral_profile = self._smooth_lateral_profile(raw_lateral, smoothing_passes)
            points: List[Point] = [(current_pose.x, current_pose.y)]
            for s_value, d_value in zip(s_values, lateral_profile):
                points.append(self.reference.xy_from_sd(s_value, d_value))
            points = remove_near_duplicates(points)
            valid, cost, _ = self._validate_and_score(points, obstacles, 0.0)
            if not valid:
                continue
            has_left = any(value > 0.15 for value in lateral_profile)
            has_right = any(value < -0.15 for value in lateral_profile)
            side = "mixed" if has_left and has_right else ("left" if has_left else "right")
            return PlanResult(
                status="detour",
                points=points,
                rejoin_s=s_values[-1],
                side=side,
                lateral_offset=max(lateral_profile, key=abs),
                cost=cost,
                reason="multi_obstacle_lattice",
            )
        return None

    def plan(self, current_pose: Pose2D, progress_s: float, obstacle_points: Sequence[Point]) -> PlanResult:
        search_min = max(0.0, progress_s - 0.8)
        search_max = min(self.reference.length, progress_s + 3.0)
        projection = self.reference.project(current_pose.x, current_pose.y, search_min, search_max)
        current_s = max(progress_s, projection.s)
        projection = self.reference.project(current_pose.x, current_pose.y, max(0.0, current_s - 0.5), search_max)
        obstacles = ObstacleIndex(obstacle_points)
        blocked = self._blocked_interval(current_s, obstacles)

        if blocked is None:
            end_s = min(self.reference.length, current_s + self.config.planning_horizon)
            return PlanResult(
                status="clear",
                points=self.reference.segment(current_s, end_s, self.config.path_sample_step, current_pose),
                rejoin_s=end_s,
                reason="reference_path_clear",
            )

        blocked_start_s, blocked_end_s = blocked
        candidates: List[PlanResult] = []
        rejection_reasons: Dict[str, int] = {}
        for offset_magnitude in self.config.lateral_offsets:
            if offset_magnitude > self.config.max_lateral_offset + 1e-6:
                continue
            for sign, side in ((1.0, "left"), (-1.0, "right")):
                offset = sign * offset_magnitude
                points, rejoin_s = self._detour_points(
                    current_pose, projection, blocked_start_s, blocked_end_s, offset
                )
                valid, cost, reason = self._validate_and_score(points, obstacles, offset)
                if not valid:
                    rejection_reasons[reason] = rejection_reasons.get(reason, 0) + 1
                    continue
                candidates.append(
                    PlanResult(
                        status="detour",
                        points=points,
                        rejoin_s=rejoin_s,
                        side=side,
                        lateral_offset=offset,
                        cost=cost,
                        reason="smooth_reference_detour",
                        blocked_start_s=blocked_start_s,
                        blocked_end_s=blocked_end_s,
                    )
                )

        if not candidates:
            lattice_candidate = self._multi_obstacle_lattice(
                current_pose,
                projection,
                obstacles,
                blocked_end_s,
            )
            if lattice_candidate is not None:
                lattice_candidate.blocked_start_s = blocked_start_s
                lattice_candidate.blocked_end_s = blocked_end_s
                return lattice_candidate
            reason_text = ",".join("{}={}".format(key, value) for key, value in sorted(rejection_reasons.items()))
            return PlanResult(
                status="blocked",
                points=[],
                rejoin_s=current_s,
                reason="no_safe_smooth_candidate:" + (reason_text or "unknown"),
                blocked_start_s=blocked_start_s,
                blocked_end_s=blocked_end_s,
            )

        candidates.sort(key=lambda candidate: candidate.cost)
        return candidates[0]


def base_pose_from_lidar_odometry(lidar_pose: Pose2D, vehicle: VehicleGeometry) -> Pose2D:
    """Convert an odometry pose located at the front lidar to the body centre."""
    cosine = math.cos(lidar_pose.yaw)
    sine = math.sin(lidar_pose.yaw)
    offset_x = cosine * vehicle.lidar_x - sine * vehicle.lidar_y
    offset_y = sine * vehicle.lidar_x + cosine * vehicle.lidar_y
    return Pose2D(lidar_pose.x - offset_x, lidar_pose.y - offset_y, lidar_pose.yaw)


def lidar_point_to_world(
    point_lidar: Tuple[float, float, float], base_pose: Pose2D, vehicle: VehicleGeometry
) -> Tuple[float, float, float]:
    """Transform a lidar-frame point to world coordinates through the body centre."""
    x_base = point_lidar[0] + vehicle.lidar_x
    y_base = point_lidar[1] + vehicle.lidar_y
    z_base = point_lidar[2] + vehicle.lidar_z
    cosine = math.cos(base_pose.yaw)
    sine = math.sin(base_pose.yaw)
    x_world = base_pose.x + cosine * x_base - sine * y_base
    y_world = base_pose.y + sine * x_base + cosine * y_base
    return x_world, y_world, z_base
