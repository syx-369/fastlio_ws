#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CSV 标记区间内的 Pure Pursuit + A* 避障测试节点。

CSV 的 ``task`` 列使用 ``avoid_start`` 和 ``avoid_end`` 标记静态随机
障碍测试区。节点复用 pure_pursuit_astar_follower.py 中已经实车化的：

* PointCloud2 / LaserScan 二维障碍提取；
* 障碍膨胀、8 邻域 A*、视线平滑；
* 分段执行和滚动重规划；
* Pure Pursuit 跟踪及车前紧急安全层。

与原测试节点的区别是：A* 和障碍安全层默认只在 CSV 标记区间内生效，
方便单独验证随机静态障碍区，不会改变区间外的正常路径跟踪行为。
"""

import csv
import math
from bisect import bisect_left, bisect_right
from dataclasses import dataclass

import rospy
from std_msgs.msg import Bool

from pure_pursuit_astar_follower import (
    PurePursuitAStarFollower,
    Waypoint,
    clamp,
    parse_bool,
    wrap_to_pi,
)


@dataclass
class AvoidanceWaypoint(Waypoint):
    task: str = "none"


class AvoidanceZoneAStarTest(PurePursuitAStarFollower):
    """只在 CSV avoid_start/avoid_end 区间启用 A* 的测试节点。"""

    def __init__(self):
        # 这些参数必须在 super() 前读取，因为父类初始化期间会回调
        # 本类的 load_waypoints()。
        self.zone_only = parse_bool(rospy.get_param("~zone_only", True))
        self.require_zone_markers = parse_bool(
            rospy.get_param("~require_zone_markers", True)
        )
        self.avoidance_ranges = []
        self.zone_active = False
        self.zone_state_pub = None
        self.localization_fault = False
        self.last_accepted_odom_xy = None
        self.odom_recovery_count = 0
        self.odom_jump_threshold = max(
            0.20, float(rospy.get_param("~odom_jump_threshold", 0.80))
        )
        self.odom_recovery_samples = max(
            1, int(rospy.get_param("~odom_recovery_samples", 5))
        )

        super().__init__()

        self.route_cumulative_s = [0.0]
        for previous, current in zip(self.global_waypoints, self.global_waypoints[1:]):
            self.route_cumulative_s.append(
                self.route_cumulative_s[-1]
                + math.hypot(current.x - previous.x, current.y - previous.y)
            )
        self.route_progress_s = 0.0
        self.progress_search_back = max(
            1, int(rospy.get_param("~progress_search_back", 10))
        )
        self.progress_search_ahead = max(
            10, int(rospy.get_param("~progress_search_ahead", 12))
        )
        self.progress_zone_search_ahead = max(
            self.progress_search_ahead,
            int(rospy.get_param("~progress_zone_search_ahead", 20)),
        )
        self.progress_max_advance = max(
            0.20, float(rospy.get_param("~progress_max_advance", 1.00))
        )
        self.progress_zone_max_advance = max(
            self.progress_max_advance,
            float(rospy.get_param("~progress_zone_max_advance", 3.60)),
        )
        self.progress_zone_confirm_samples = max(
            2, int(rospy.get_param("~progress_zone_confirm_samples", 3))
        )
        self.progress_zone_confirm_tolerance = max(
            0.05,
            float(rospy.get_param("~progress_zone_confirm_tolerance", 0.30)),
        )
        self.progress_heading_tolerance = max(
            0.10, float(rospy.get_param("~progress_heading_tolerance", 0.70))
        )
        self.progress_lateral_limit = max(
            0.5, float(rospy.get_param("~progress_lateral_limit", 3.0))
        )
        self.progress_pass_margin = max(
            0.0, float(rospy.get_param("~progress_pass_margin", 0.08))
        )
        self.zone_entry_margin = max(
            0.0, float(rospy.get_param("~zone_entry_margin", 0.05))
        )
        self.zone_exit_margin = max(
            0.0, float(rospy.get_param("~zone_exit_margin", 0.0))
        )
        self.zone_handoff_distance = max(
            0.20, float(rospy.get_param("~zone_handoff_distance", 0.60))
        )
        self.zone_handoff_early_release = min(
            self.zone_handoff_distance,
            max(0.0, float(rospy.get_param("~zone_handoff_early_release", 0.25))),
        )
        self.zone_handoff_lateral_tolerance = max(
            0.05,
            float(rospy.get_param("~zone_handoff_lateral_tolerance", 0.30)),
        )
        self.last_route_lateral = float("inf")
        self.progress_fault = False
        self.zone_progress_candidate_s = None
        self.zone_progress_candidate_segment = None
        self.zone_progress_candidate_count = 0
        # 区外在任务点后重新接回航迹时，投影可能有轻度前跳。该情况先
        # 限速并连续确认，而不是把比赛流程直接锁死。
        self.progress_recovery_max_advance = max(
            self.progress_max_advance,
            float(rospy.get_param("~progress_recovery_max_advance", 1.80)),
        )
        self.progress_recovery_samples = max(
            2, int(rospy.get_param("~progress_recovery_samples", 3))
        )
        self.progress_recovery_tolerance = max(
            0.05,
            float(rospy.get_param("~progress_recovery_tolerance", 0.30)),
        )
        self.progress_recovery_speed = max(
            0.0, float(rospy.get_param("~progress_recovery_speed", 0.25))
        )
        self.progress_recovery_active = False
        self.progress_recovery_candidate_s = None
        self.progress_recovery_candidate_segment = None
        self.progress_recovery_candidate_count = 0

        self.planner_requested = self.planner_enabled
        if not self.planner_requested:
            rospy.logwarn("~planner_enabled=false: A* is disabled even inside avoid zone.")
        self.min_valid_plan_dist = max(
            0.10, float(rospy.get_param("~zone_min_valid_plan_dist", 0.25))
        )

        # 区域外连续 Pure Pursuit 参考路径。CSV 约每 0.16m 一个点，不能把
        # 每个点都当成需要停车/减速的独立目标。
        self.reference_horizon = max(
            1.0, float(rospy.get_param("~reference_horizon", 4.0))
        )
        self.reference_spacing = max(
            0.05, float(rospy.get_param("~reference_spacing", 0.15))
        )
        smoothing_window = int(rospy.get_param("~reference_smoothing_window", 5))
        self.reference_smoothing_window = max(1, smoothing_window | 1)
        self.reference_end_index = 0

        # 父类的 final_approach_dist 原本会对“当前的每一个 CSV 点”减速。
        # 这里保存其数值，并关闭父类逐点减速，只在整条路线最终点附近使用。
        self.endpoint_slow_dist = max(0.05, self.final_approach_dist)
        self.final_approach_dist = 0.0
        self.nominal_target_speed = self.target_speed
        self.min_tracking_speed = min(
            self.nominal_target_speed,
            max(0.0, float(rospy.get_param("~min_tracking_speed", 0.08))),
        )

        # 速度斜率限制，避免每个控制周期的速度突变传到底盘。
        self.max_linear_accel = max(
            0.01, float(rospy.get_param("~max_linear_accel", 0.25))
        )
        self.max_linear_decel = max(
            self.max_linear_accel,
            float(rospy.get_param("~max_linear_decel", 0.60)),
        )
        self.max_angular_accel = max(
            0.05, float(rospy.get_param("~max_angular_accel", 1.50))
        )
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_cmd_time = rospy.Time.now()
        self.force_stop_cmd = False

        self.zone_state_pub = rospy.Publisher(
            "~avoidance_zone_active", Bool, queue_size=1, latch=True
        )
        self.update_zone_state(force=True)

        ranges_text = ", ".join(
            "seq %d..%d"
            % (self.global_waypoints[start].seq, self.global_waypoints[end].seq)
            for start, end in self.avoidance_ranges
        )
        rospy.loginfo("Avoidance ranges: %s", ranges_text or "none (whole route mode)")
        rospy.loginfo(
            "Zone test options: start_seq=%d zone_only=%s; run to CSV endpoint",
            self.global_waypoints[0].seq,
            self.zone_only,
        )
        rospy.loginfo(
            "Continuous reference: horizon=%.1fm spacing=%.2fm smooth_window=%d",
            self.reference_horizon,
            self.reference_spacing,
            self.reference_smoothing_window,
        )
        rospy.loginfo(
            "Speed profile: cruise=%.2f min=%.2f accel=%.2f decel=%.2f angular_accel=%.2f",
            self.nominal_target_speed,
            self.min_tracking_speed,
            self.max_linear_accel,
            self.max_linear_decel,
            self.max_angular_accel,
        )
        rospy.loginfo(
            "Route progress: length=%.2fm search=%d/%d zone_ahead=%d max_advance=%.2fm "
            "zone_max=%.2fm zone_confirm=%d@%.2fm heading=%.2frad "
            "lateral_limit=%.2fm pass_margin=%.2fm",
            self.route_cumulative_s[-1],
            self.progress_search_back,
            self.progress_search_ahead,
            self.progress_zone_search_ahead,
            self.progress_max_advance,
            self.progress_zone_max_advance,
            self.progress_zone_confirm_samples,
            self.progress_zone_confirm_tolerance,
            self.progress_heading_tolerance,
            self.progress_lateral_limit,
            self.progress_pass_margin,
        )
        rospy.loginfo(
            "Localization guard: jump=%.2fm recovery_samples=%d",
            self.odom_jump_threshold,
            self.odom_recovery_samples,
        )
        rospy.loginfo(
            "Zone handoff: distance=%.2fm early_release=%.2fm lateral_tol=%.2fm",
            self.zone_handoff_distance,
            self.zone_handoff_early_release,
            self.zone_handoff_lateral_tolerance,
        )

    def load_waypoints(self, csv_path):
        """读取航点及 task，并校验 avoid_start/avoid_end 是否成对。"""
        waypoints = []
        ranges = []
        open_start = None
        required = {"seq", "x", "y", "yaw", "task"}

        with open(csv_path, "r", encoding="utf-8-sig", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            missing = required.difference(reader.fieldnames or [])
            if missing:
                raise ValueError(
                    "CSV missing required columns: %s" % ", ".join(sorted(missing))
                )

            for row_number, row in enumerate(reader, start=2):
                try:
                    task = (row.get("task") or "none").strip().lower()
                    waypoint = AvoidanceWaypoint(
                        seq=int(row["seq"]),
                        x=float(row["x"]),
                        y=float(row["y"]),
                        yaw=float(row["yaw"]),
                        tol=(
                            float(row["tol"])
                            if row.get("tol", "").strip()
                            else self.goal_tolerance_default
                        ),
                        task=task,
                    )
                except (TypeError, ValueError) as exc:
                    raise ValueError("Invalid CSV row %d: %s" % (row_number, exc))

                index = len(waypoints)
                if task == "avoid_start":
                    if open_start is not None:
                        raise ValueError(
                            "Nested avoid_start at CSV row %d is not supported" % row_number
                        )
                    open_start = index
                elif task == "avoid_end":
                    if open_start is None:
                        raise ValueError(
                            "avoid_end at CSV row %d has no preceding avoid_start"
                            % row_number
                        )
                    ranges.append((open_start, index))
                    open_start = None

                waypoints.append(waypoint)

        if open_start is not None:
            raise ValueError("CSV avoid_start has no matching avoid_end")
        if self.require_zone_markers and not ranges:
            raise ValueError(
                "CSV has no avoid_start/avoid_end pair; set "
                "~require_zone_markers:=false and ~zone_only:=false "
                "for whole-route A* testing"
            )

        self.avoidance_ranges = ranges
        return waypoints

    def index_inside_zone(self, index):
        if index < 0 or index >= len(self.global_waypoints):
            return False
        if not self.zone_only:
            return True
        return any(start <= index <= end for start, end in self.avoidance_ranges)

    def zone_handoff_target(self, zone_end):
        """返回不超过交接距离的实际 CSV 目标索引和弧长。"""
        desired_s = min(
            self.route_cumulative_s[-1],
            self.route_cumulative_s[zone_end]
            + self.zone_exit_margin
            + self.zone_handoff_distance,
        )
        target_index = min(
            len(self.global_waypoints) - 1,
            max(zone_end, bisect_right(self.route_cumulative_s, desired_s) - 1),
        )
        return target_index, self.route_cumulative_s[target_index]

    def active_zone_bounds(self):
        """按路线弧长判断避障区，并在出口后保留有限 A* 交接走廊。"""
        if not self.zone_only:
            return (0, len(self.global_waypoints) - 1)
        for start, end in self.avoidance_ranges:
            start_s = max(0.0, self.route_cumulative_s[start] - self.zone_entry_margin)
            end_s = self.route_cumulative_s[end] + self.zone_exit_margin
            _, handoff_goal_s = self.zone_handoff_target(end)
            # 切换必须发生在 A* 剩余距离仍足够形成有效路径时，避免出口处
            # 反复产生 points=1 / dist<min_valid_plan_dist 后短暂停车。
            release_buffer = max(
                self.zone_handoff_early_release,
                self.min_valid_plan_dist,
            )
            handoff_release_s = max(
                end_s,
                handoff_goal_s - release_buffer,
            )
            if start_s <= self.route_progress_s < end_s:
                return start, end
            # 只有已经从这个区域进入 A* 后，才允许延长至出口交接走廊；
            # 避免节点从区域外启动时误判成仍在避障区。
            if self.zone_active and end_s <= self.route_progress_s <= handoff_goal_s:
                if (
                    self.route_progress_s < handoff_release_s
                    or self.last_route_lateral > self.zone_handoff_lateral_tolerance
                ):
                    return start, end
        return None

    def update_zone_state(self, force=False):
        was_active = self.zone_active
        zone_bounds = self.active_zone_bounds()
        active = zone_bounds is not None
        if not force and active == self.zone_active:
            return

        self.zone_active = active
        self.planner_enabled = self.planner_requested and active
        self.local_path = []
        self.local_index = 0
        self.last_full_plan = []
        self.blocked_since = None
        self.detour_side_lock = 0
        self.detour_lock_until = rospy.Time(0)
        self.clear_zone_progress_candidate()

        if self.zone_state_pub is not None:
            self.zone_state_pub.publish(Bool(data=active))
        if hasattr(self, "full_path_pub"):
            self.publish_path(self.full_path_pub, [])
            self.publish_path(self.exec_path_pub, [])

        if active:
            start, end = zone_bounds
            rospy.logwarn(
                "ENTER avoidance zone seq=%d..%d at progress %.2fm: "
                "obstacle input and rolling A* enabled.",
                self.global_waypoints[start].seq,
                self.global_waypoints[end].seq,
                self.route_progress_s,
            )
        else:
            if was_active:
                rospy.logwarn(
                    "EXIT avoidance zone at progress %.2fm lateral=%.2fm: "
                    "switch smoothly to continuous CSV.",
                    self.route_progress_s,
                    self.last_route_lateral,
                )
            else:
                rospy.loginfo(
                    "Outside avoidance zone at progress %.2fm: follow continuous CSV.",
                    self.route_progress_s,
                )

    def odom_callback(self, msg):
        """拒绝单帧大幅位姿跳变，防止规划器突然回头追旧路线。"""
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_accepted_odom_xy is not None:
            jump = math.hypot(
                x - self.last_accepted_odom_xy[0],
                y - self.last_accepted_odom_xy[1],
            )
            if jump > self.odom_jump_threshold:
                self.localization_fault = True
                self.odom_recovery_count = 0
                rospy.logerr_throttle(
                    1.0,
                    "Localization jump rejected: %.2fm > %.2fm; robot locked. "
                    "Check /Odometry publishers or FAST-LIO relocalization.",
                    jump,
                    self.odom_jump_threshold,
                )
                return

        if self.localization_fault:
            self.odom_recovery_count += 1
            if self.odom_recovery_count < self.odom_recovery_samples:
                return
            self.localization_fault = False
            self.odom_recovery_count = 0
            rospy.logwarn("Localization recovered on the last accepted trajectory.")

        super().odom_callback(msg)
        self.last_accepted_odom_xy = (x, y)

    def get_obstacles_snapshot(self):
        # 点云/scan 回调仍持续缓存数据，进入区域后能够立即使用最新障碍；
        # 区域外向规划器和安全层返回空集，避免误触发避障。
        if self.zone_only and not self.zone_active:
            return []
        return super().get_obstacles_snapshot()

    def smooth_reference_points(self, points):
        """对 CSV 折线做小窗口移动平均，并保留窗口首尾点。"""
        if len(points) < 3 or self.reference_smoothing_window <= 1:
            return list(points)

        half = self.reference_smoothing_window // 2
        smoothed = []
        for index, point in enumerate(points):
            if index == 0 or index == len(points) - 1:
                smoothed.append(point)
                continue
            begin = max(0, index - half)
            end = min(len(points), index + half + 1)
            count = end - begin
            x = sum(points[i][0] for i in range(begin, end)) / count
            y = sum(points[i][1] for i in range(begin, end)) / count
            smoothed.append((x, y))
        return smoothed

    def resample_reference_points(self, points):
        """按固定弧长重采样，使 Pure Pursuit 前视点分布连续。"""
        if len(points) < 2:
            return list(points)

        cumulative = [0.0]
        for previous, current in zip(points, points[1:]):
            cumulative.append(
                cumulative[-1]
                + math.hypot(current[0] - previous[0], current[1] - previous[1])
            )
        total = cumulative[-1]
        if total < 1e-6:
            return [points[-1]]

        result = [points[0]]
        segment = 1
        sample_s = self.reference_spacing
        while sample_s < total:
            while segment < len(cumulative) - 1 and cumulative[segment] < sample_s:
                segment += 1
            s0 = cumulative[segment - 1]
            s1 = cumulative[segment]
            ratio = (sample_s - s0) / max(s1 - s0, 1e-9)
            x0, y0 = points[segment - 1]
            x1, y1 = points[segment]
            result.append((x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)))
            sample_s += self.reference_spacing
        if math.hypot(result[-1][0] - points[-1][0], result[-1][1] - points[-1][1]) > 0.02:
            result.append(points[-1])
        return result

    def build_continuous_reference(self):
        """从当前进度构造数米长的连续 CSV 参考路径。"""
        if self.global_index >= len(self.global_waypoints):
            return []

        start_index = max(0, self.global_index - 1)
        hard_end = len(self.global_waypoints) - 1
        # 区域外的参考路径不跨过下一个 avoid_start；到边界后立即切 A*。
        for zone_start, _ in self.avoidance_ranges:
            if zone_start > self.global_index:
                hard_end = min(hard_end, zone_start)
                break

        raw = []
        travelled = 0.0
        previous = None
        end_index = start_index
        for index in range(start_index, hard_end + 1):
            waypoint = self.global_waypoints[index]
            point = (waypoint.x, waypoint.y)
            if previous is not None:
                travelled += math.hypot(point[0] - previous[0], point[1] - previous[1])
            raw.append(point)
            previous = point
            end_index = index
            if travelled >= self.reference_horizon and index > self.global_index:
                break

        self.reference_end_index = end_index
        return self.resample_reference_points(self.smooth_reference_points(raw))

    def replan_to_global_waypoint(self, wp):
        if self.zone_active:
            return super().replan_to_global_waypoint(wp)

        reference = self.build_continuous_reference()
        if not reference:
            return False
        self.last_full_plan = reference
        self.local_path = list(reference)
        self.local_index = 0
        self.last_plan_stamp = rospy.Time.now()
        self.publish_path(self.full_path_pub, self.last_full_plan)
        self.publish_path(self.exec_path_pub, self.local_path)
        rospy.loginfo(
            "Continuous CSV reference: seq=%d..%d points=%d",
            self.global_waypoints[self.global_index].seq,
            self.global_waypoints[self.reference_end_index].seq,
            len(reference),
        )
        return True

    def local_segment_done(self):
        if self.zone_active:
            return super().local_segment_done()
        if not self.local_path:
            return True
        if self.global_index >= self.reference_end_index:
            return True
        end_x, end_y = self.local_path[-1]
        end_x_body, _ = self.point_in_body(end_x, end_y)
        if end_x_body < -0.20:
            return True
        return math.hypot(end_x - self.current_x, end_y - self.current_y) <= self.local_goal_tolerance

    def progress_projection_end_segment(self):
        """返回本轮允许用于进度投影的最后一个折线段。

        普通路径允许看到文件末尾；任务跟踪器会覆盖此方法，把搜索硬限制在
        下一个未完成任务点之前，防止重复航迹把进度吸到下一圈。
        """
        return len(self.global_waypoints) - 2

    def project_pose_to_route(self):
        """在当前进度附近把车辆投影到 CSV 折线，返回弧长和横向距离。"""
        if len(self.global_waypoints) < 2:
            return 0.0, float("inf"), 0

        center = min(self.global_index, len(self.global_waypoints) - 1)
        begin = max(0, center - self.progress_search_back - 1)
        search_ahead = (
            self.progress_zone_search_ahead
            if self.zone_active
            else self.progress_search_ahead
        )
        end = min(
            len(self.global_waypoints) - 2,
            center + search_ahead,
            self.progress_projection_end_segment(),
        )
        if end < begin:
            return self.route_progress_s, float("inf"), center
        best = None
        for segment_index in range(begin, end + 1):
            first = self.global_waypoints[segment_index]
            second = self.global_waypoints[segment_index + 1]
            vx = second.x - first.x
            vy = second.y - first.y
            length_sq = vx * vx + vy * vy
            if length_sq < 1e-9:
                continue
            ratio = clamp(
                ((self.current_x - first.x) * vx + (self.current_y - first.y) * vy)
                / length_sq,
                0.0,
                1.0,
            )
            projected_x = first.x + ratio * vx
            projected_y = first.y + ratio * vy
            lateral = math.hypot(
                self.current_x - projected_x,
                self.current_y - projected_y,
            )
            segment_length = math.sqrt(length_sq)
            projected_s = self.route_cumulative_s[segment_index] + ratio * segment_length
            # 不让一个路线交叉点把进度匹配到已经走过很远的后方。
            if projected_s < self.route_progress_s - 0.50:
                continue
            # 用录制航向插值，而不是折线切向。任务点附近常有几毫米的停顿
            # 小段，其几何切向会被定位噪声放大，但录制航向仍然可靠。
            route_yaw = wrap_to_pi(
                first.yaw + ratio * wrap_to_pi(second.yaw - first.yaw)
            )
            heading_error = abs(wrap_to_pi(route_yaw - self.current_yaw))
            if heading_error > self.progress_heading_tolerance:
                continue
            candidate = (lateral, projected_s, segment_index)
            if best is None or candidate < best:
                best = candidate

        if best is None:
            return self.route_progress_s, float("inf"), center
        return best[1], best[0], best[2]

    def select_planning_waypoint(self):
        """沿路线弧长选目标；区内目标绝不越过 avoid_end。"""
        if self.global_index >= len(self.global_waypoints):
            return None
        if not self.zone_active:
            target_index = min(
                len(self.global_waypoints) - 1,
                max(self.global_index, self.reference_end_index),
            )
            return self.global_waypoints[target_index]

        zone_bounds = self.active_zone_bounds()
        if zone_bounds is None:
            return self.global_waypoints[self.global_index]
        _, zone_end = zone_bounds
        handoff_limit_index, handoff_limit_s = self.zone_handoff_target(zone_end)
        target_s = min(
            handoff_limit_s,
            self.route_progress_s + max(
                self.planning_goal_min_dist,
                self.lookahead_distance * 2.0,
            ),
        )
        target_index = bisect_left(self.route_cumulative_s, target_s)
        target_index = min(
            handoff_limit_index,
            max(self.global_index, target_index),
        )
        return self.global_waypoints[target_index]

    def clear_zone_progress_candidate(self):
        """清除避障区路线重投影的连续确认状态。"""
        self.zone_progress_candidate_s = None
        self.zone_progress_candidate_segment = None
        self.zone_progress_candidate_count = 0

    def clear_progress_recovery_candidate(self):
        """清除区外轻度进度跳变的恢复候选状态。"""
        self.progress_recovery_active = False
        self.progress_recovery_candidate_s = None
        self.progress_recovery_candidate_segment = None
        self.progress_recovery_candidate_count = 0

    def confirm_progress_recovery(self, projected_s, segment_index):
        """对任务点后等可预期的轻度前跳做连续确认后再接回航迹。"""
        consistent = (
            self.progress_recovery_candidate_s is not None
            and abs(projected_s - self.progress_recovery_candidate_s)
            <= self.progress_recovery_tolerance
        )
        if consistent:
            self.progress_recovery_candidate_count += 1
        else:
            self.progress_recovery_candidate_count = 1

        self.progress_recovery_active = True
        self.progress_recovery_candidate_s = projected_s
        self.progress_recovery_candidate_segment = segment_index
        if self.progress_recovery_candidate_count >= self.progress_recovery_samples:
            rospy.logwarn(
                "Route progress recovery accepted after %d consistent samples: "
                "%.2fm -> %.2fm (advance %.2fm, segment=%d).",
                self.progress_recovery_candidate_count,
                self.route_progress_s,
                projected_s,
                projected_s - self.route_progress_s,
                segment_index,
            )
            self.clear_progress_recovery_candidate()
            return True

        rospy.logwarn_throttle(
            1.0,
            "Route progress recovery pending: %.2fm -> %.2fm "
            "(advance %.2fm, sample %d/%d, segment=%d); limit speed to %.2fm/s.",
            self.route_progress_s,
            projected_s,
            projected_s - self.route_progress_s,
            self.progress_recovery_candidate_count,
            self.progress_recovery_samples,
            segment_index,
            self.progress_recovery_speed,
        )
        return False

    def confirm_zone_progress_rejoin(self, projected_s, segment_index):
        """连续确认 A* 绕行后重新落回 CSV 的中等进度跳变。

        A* 会让车辆暂时偏离原始 CSV，路线投影可能先停滞、再一次落到前方
        1m 以上的位置。这类跳变只在避障区内放宽，并要求多帧候选进度相互
        一致；区外仍使用严格的单帧防跳锁止。
        """
        consistent = (
            self.zone_progress_candidate_s is not None
            and abs(projected_s - self.zone_progress_candidate_s)
            <= self.progress_zone_confirm_tolerance
        )
        if consistent:
            self.zone_progress_candidate_count += 1
        else:
            self.zone_progress_candidate_count = 1

        self.zone_progress_candidate_s = projected_s
        self.zone_progress_candidate_segment = segment_index
        confirmed = (
            self.zone_progress_candidate_count >= self.progress_zone_confirm_samples
        )
        if confirmed:
            rospy.logwarn(
                "Avoidance-zone route rejoin accepted after %d consistent samples: "
                "%.2fm -> %.2fm (advance %.2fm, segment=%d).",
                self.zone_progress_candidate_count,
                self.route_progress_s,
                projected_s,
                projected_s - self.route_progress_s,
                segment_index,
            )
            self.clear_zone_progress_candidate()
            return True

        rospy.logwarn_throttle(
            1.0,
            "Avoidance-zone route rejoin pending: %.2fm -> %.2fm "
            "(advance %.2fm, sample %d/%d, segment=%d).",
            self.route_progress_s,
            projected_s,
            projected_s - self.route_progress_s,
            self.zone_progress_candidate_count,
            self.progress_zone_confirm_samples,
            segment_index,
        )
        return False

    def advance_global_waypoint_if_needed(self):
        old_index = self.global_index
        projected_s, lateral, segment_index = self.project_pose_to_route()
        self.last_route_lateral = lateral
        if lateral > self.progress_lateral_limit:
            self.clear_zone_progress_candidate()
            self.clear_progress_recovery_candidate()
            rospy.logwarn_throttle(
                1.0,
                "Route projection is %.2fm away (limit %.2fm); keep monotonic progress %.2fm.",
                lateral,
                self.progress_lateral_limit,
                self.route_progress_s,
            )
            return

        progress_advance = projected_s - self.route_progress_s
        if progress_advance > self.progress_max_advance:
            if (
                self.zone_active
                and progress_advance <= self.progress_zone_max_advance
            ):
                if not self.confirm_zone_progress_rejoin(projected_s, segment_index):
                    return
            elif (
                not self.zone_active
                and progress_advance <= self.progress_recovery_max_advance
            ):
                if not self.confirm_progress_recovery(projected_s, segment_index):
                    return
            else:
                self.clear_zone_progress_candidate()
                self.clear_progress_recovery_candidate()
                active_limit = (
                    self.progress_zone_max_advance
                    if self.zone_active
                    else self.progress_max_advance
                )
                self.progress_fault = True
                rospy.logfatal(
                    "Route progress jump rejected: %.2fm -> %.2fm (advance %.2fm, "
                    "limit %.2fm, avoidance_zone=%s, segment=%d). Robot is latched "
                    "stopped; restart after checking route/localization.",
                    self.route_progress_s,
                    projected_s,
                    progress_advance,
                    active_limit,
                    self.zone_active,
                    segment_index,
                )
                self.stop_robot()
                return
        else:
            self.clear_zone_progress_candidate()
            self.clear_progress_recovery_candidate()

        self.route_progress_s = max(self.route_progress_s, projected_s)

        passed_s = self.route_progress_s + self.progress_pass_margin
        self.global_index = min(
            len(self.global_waypoints),
            bisect_right(self.route_cumulative_s, passed_s),
        )

        if self.global_index != old_index:
            first_seq = self.global_waypoints[min(old_index, len(self.global_waypoints) - 1)].seq
            last_seq = self.global_waypoints[
                min(self.global_index - 1, len(self.global_waypoints) - 1)
            ].seq
            rospy.loginfo(
                "Route progress %.2fm lateral=%.2fm segment=%d passed seq=%d..%d next=%s",
                self.route_progress_s,
                lateral,
                segment_index,
                first_seq,
                last_seq,
                (
                    str(self.global_waypoints[self.global_index].seq)
                    if self.global_index < len(self.global_waypoints)
                    else "FINISH"
                ),
            )
        self.update_zone_state()

    def publish_cmd(self, linear_x, angular_z):
        """限制相邻控制周期速度变化；零速安全命令仍立即生效。"""
        now = rospy.Time.now()
        dt = (now - self.last_cmd_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(self.control_rate, 1.0)

        if self.force_stop_cmd:
            limited_linear = 0.0
            limited_angular = 0.0
        else:
            desired_linear = float(linear_x)
            if desired_linear <= 0.0:
                # 原地转向或安全停车时，线速度立即归零。
                limited_linear = 0.0
            else:
                delta = desired_linear - self.last_cmd_linear
                rate = self.max_linear_accel if delta >= 0.0 else self.max_linear_decel
                limited_linear = self.last_cmd_linear + clamp(delta, -rate * dt, rate * dt)

            angular_delta = float(angular_z) - self.last_cmd_angular
            limited_angular = self.last_cmd_angular + clamp(
                angular_delta,
                -self.max_angular_accel * dt,
                self.max_angular_accel * dt,
            )

        self.last_cmd_linear = limited_linear
        self.last_cmd_angular = limited_angular
        self.last_cmd_time = now
        super().publish_cmd(limited_linear, limited_angular)

    def stop_robot(self):
        self.force_stop_cmd = True
        try:
            super().stop_robot()
        finally:
            self.force_stop_cmd = False

    def control_step(self):
        if self.progress_fault:
            rospy.logerr_throttle(
                1.0,
                "Route progress fault is latched; publish zero speed until node restart.",
            )
            self.stop_robot()
            return
        if self.localization_fault:
            rospy.logerr_throttle(
                1.0,
                "Localization fault is latched; publish zero speed until odometry recovers.",
            )
            self.stop_robot()
            return
        # 保持中间航点恒定巡航；只在整条 CSV 的最终点附近连续减速。
        self.target_speed = self.nominal_target_speed
        if self.progress_recovery_active:
            self.target_speed = min(
                self.target_speed, self.progress_recovery_speed
            )
        if self.has_odom and self.global_index == len(self.global_waypoints) - 1:
            final_waypoint = self.global_waypoints[-1]
            final_distance = math.hypot(
                final_waypoint.x - self.current_x,
                final_waypoint.y - self.current_y,
            )
            ratio = clamp(final_distance / self.endpoint_slow_dist, 0.0, 1.0)
            self.target_speed = max(
                self.min_tracking_speed,
                self.nominal_target_speed * ratio,
            )
        try:
            super().control_step()
        finally:
            self.target_speed = self.nominal_target_speed

    def on_shutdown(self):
        super().on_shutdown()
        rospy.loginfo("Avoidance zone A* test node shutdown.")


if __name__ == "__main__":
    rospy.init_node("avoidance_zone_astar_test", anonymous=False)
    try:
        node = AvoidanceZoneAStarTest()
        node.spin()
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        rospy.logfatal("Avoidance zone test initialization failed: %s", exc)
        raise
