#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""决赛统一跟踪器。

继承 waypoint_tools/scripts/avoidance_zone_astar_test.py 的 AvoidanceZoneAStarTest，
完整保留其已实车验证的能力：

* CSV avoid_start/avoid_end 区间内的滚动 A* 避障；
* 区间外的连续参考线 Pure Pursuit（解决逐点 P 控制的速度锯齿）；
* 定位跳变拒绝（odom_jump_threshold）；
* 线速度/角速度斜率限幅。

在此之上新增比赛必需的两件事：

1. 任务点三段式精确停靠：SLOWDOWN -> APPROACH(P控制) -> ALIGN_YAW(原地自转)。
   父类只做巡航，不做点到点精确停靠；而机械臂能否抓到取决于车辆停靠位姿。
2. CSV task 列的外部任务握手：ext:<name> 停车 -> 发 start 事件 -> 阻塞等 done。
   父类把 task 读进了 AvoidanceWaypoint.task 但从未使用。

刻意不做的事（由其他节点负责）：
* 不等红旗、不识别红绿灯（final_manager + final_vision）；
* 不直接驱动机械臂（piper_task 自己订阅 /waypoint_task_event）。
"""

import math
import os
import sys

import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

# 父类在 waypoint_tools/scripts 下，不是标准 python 包，需要手动加入路径。
_WAYPOINT_TOOLS_SCRIPTS = os.path.join(
    rospkg.RosPack().get_path("waypoint_tools"), "scripts"
)
if _WAYPOINT_TOOLS_SCRIPTS not in sys.path:
    sys.path.insert(0, _WAYPOINT_TOOLS_SCRIPTS)

from avoidance_zone_astar_test import AvoidanceZoneAStarTest  # noqa: E402
from pure_pursuit_astar_follower import clamp, wrap_to_pi  # noqa: E402


# 区间标记不是任务，必须排除在任务分派之外，否则会被当成未知任务跳过
# （这正是原 race_mission 路线里避障失效的原因）。
ZONE_MARKERS = ("avoid_start", "avoid_end")

# 停靠阶段。父类用 self.state（WAIT_ODOM/TRACK/FINISH），这里用独立的
# task_phase，避免与父类状态机互相干扰。
PHASE_NONE = "NONE"
PHASE_APPROACH = "APPROACH"
PHASE_ALIGN = "ALIGN_YAW"
PHASE_TASK = "TASK"
PHASE_BACKTRACK = "BACKTRACK_RECOVERY"

BACKTRACK_PICK3_FAILURE = "failed:pick3:object_not_found_at_all_pick_points"
BACKTRACK_PICK3_EDGE_FAILURE_PREFIX = (
    "failed:pick3:target_at_edge_at_final_pick_point="
)
BACKTRACK_RECOVERY_MID = "piper_recovery_mid"
BACKTRACK_TARGETS = ("piper_stop_3", "piper_stop_2")
BACKTRACK_EDGE_TARGETS = (
    BACKTRACK_RECOVERY_MID,
    "piper_stop_3",
    "piper_stop_2",
)
BACKTRACK_PLACE3_FAILURE = "failed:place3:target_not_found_at_all_place_points"
PLACE_BACKTRACK_TARGETS = ("piper_stop_6", "piper_stop_5")


class FinalTracker(AvoidanceZoneAStarTest):
    def __init__(self):
        super().__init__()

        # ---------------- 停靠参数 ----------------
        self.task_trigger_margin = float(rospy.get_param("~task_trigger_margin", 0.80))
        self.approach_switch_dist = float(rospy.get_param("~approach_switch_dist", 0.55))
        self.approach_k_linear = float(rospy.get_param("~approach_k_linear", 0.5))
        self.approach_max_speed = float(rospy.get_param("~approach_max_speed", 0.10))
        self.approach_min_speed = float(rospy.get_param("~approach_min_speed", 0.04))
        self.goal_reached_dist = float(rospy.get_param("~goal_reached_dist", 0.05))
        self.approach_max_angular = max(
            0.01, float(rospy.get_param("~approach_max_angular", 0.30))
        )
        self.approach_near_goal_dist = max(
            self.goal_reached_dist,
            float(rospy.get_param("~approach_near_goal_dist", 0.10)),
        )

        self.task_yaw_tolerance = float(rospy.get_param("~task_yaw_tolerance", 0.05))
        self.task_yaw_max_angular = max(
            0.01, float(rospy.get_param("~task_yaw_max_angular", 0.30))
        )
        self.task_yaw_k = float(rospy.get_param("~task_yaw_k", 1.2))
        self.task_yaw_min_angular = float(
            rospy.get_param("~task_yaw_min_angular", 0.08)
        )

        self.approach_timeout = float(rospy.get_param("~approach_timeout", 15.0))
        self.align_timeout = float(rospy.get_param("~align_timeout", 8.0))

        # ---------------- 握手参数 ----------------
        self.task_event_topic = rospy.get_param(
            "~task_event_topic", "/waypoint_task_event"
        )
        self.task_done_topic = rospy.get_param(
            "~task_done_topic", "/waypoint_task_done"
        )
        self.skip_topic = rospy.get_param(
            "~skip_topic", "/piper_task/navigation_skip"
        )
        self.task_bypass_topic = rospy.get_param(
            "~task_bypass_topic", "/final_mission/task_bypass_next"
        )
        self.external_task_timeout = float(
            rospy.get_param("~external_task_timeout", 150.0)
        )
        self.arm_bridge_status_topic = rospy.get_param(
            "~arm_bridge_status_topic", "/arm_bridge/status"
        )

        # ---------------- 第三个取货点失败后的专用倒车 ----------------
        self.backtrack_command_topic = rospy.get_param(
            "~backtrack_command_topic", "/final_mission/backtrack_command"
        )
        self.backtrack_event_topic = rospy.get_param(
            "~backtrack_event_topic", "/final_mission/backtrack_event"
        )
        self.arm_result_topic = rospy.get_param(
            "~arm_result_topic", "/piper_task/result"
        )
        self.reverse_k_linear = max(
            0.01, float(rospy.get_param("~reverse_k_linear", 0.35))
        )
        self.reverse_min_speed = max(
            0.01, float(rospy.get_param("~reverse_min_speed", 0.035))
        )
        self.reverse_max_speed = max(
            self.reverse_min_speed,
            float(rospy.get_param("~reverse_max_speed", 0.06)),
        )
        self.reverse_max_angular = max(
            0.01, float(rospy.get_param("~reverse_max_angular", 0.12))
        )
        self.reverse_yaw_k = max(
            0.0, float(rospy.get_param("~reverse_yaw_k", 0.8))
        )
        self.reverse_max_yaw_error = max(
            0.01, float(rospy.get_param("~reverse_max_yaw_error", 0.35))
        )
        self.reverse_yaw_slow_threshold = min(
            self.reverse_max_yaw_error,
            max(
                0.0,
                float(rospy.get_param("~reverse_yaw_slow_threshold", 0.10)),
            ),
        )
        self.reverse_yaw_resume_threshold = min(
            self.reverse_yaw_slow_threshold,
            max(
                0.0,
                float(rospy.get_param("~reverse_yaw_resume_threshold", 0.05)),
            ),
        )
        self.reverse_heading_correction_timeout = max(
            0.5,
            float(rospy.get_param("~reverse_heading_correction_timeout", 4.0)),
        )
        self.reverse_goal_dist = max(
            0.01, float(rospy.get_param("~reverse_goal_dist", 0.04))
        )
        self.reverse_cross_track_max = max(
            0.02, float(rospy.get_param("~reverse_cross_track_max", 0.10))
        )
        self.reverse_arrival_cross_track_max = min(
            self.reverse_cross_track_max,
            max(
                self.reverse_goal_dist,
                float(
                    rospy.get_param(
                        "~reverse_arrival_cross_track_max", 0.10
                    )
                ),
            ),
        )
        self.reverse_arrival_overshoot_max = max(
            self.reverse_goal_dist,
            float(rospy.get_param("~reverse_arrival_overshoot_max", 0.08)),
        )
        self.reverse_timeout = max(
            1.0, float(rospy.get_param("~reverse_timeout", 12.0))
        )
        self.reverse_max_segment_dist = max(
            0.10, float(rospy.get_param("~reverse_max_segment_dist", 0.80))
        )
        self.reverse_recovery_mid_dist = max(
            self.reverse_goal_dist * 2.0,
            float(rospy.get_param("~reverse_recovery_mid_dist", 0.14)),
        )
        self.reverse_recovery_mid_max_speed = min(
            self.reverse_max_speed,
            max(
                self.reverse_min_speed,
                float(
                    rospy.get_param(
                        "~reverse_recovery_mid_max_speed", 0.04
                    )
                ),
            ),
        )
        self.reverse_accel = max(
            0.01, float(rospy.get_param("~reverse_accel", 0.08))
        )
        self.reverse_rear_stop_dist = max(
            0.05, float(rospy.get_param("~reverse_rear_stop_dist", 0.20))
        )
        self.reverse_safety_half_width = max(
            self.footprint_half_width,
            float(
                rospy.get_param(
                    "~reverse_safety_half_width", self.footprint_half_width + 0.05
                )
            ),
        )
        self.reverse_require_obstacles = bool(
            rospy.get_param("~reverse_require_obstacles", True)
        )

        # 起步闸门：总控在红旗确认前压住跟踪器。默认 false，避免上电即动。
        self.enabled = bool(rospy.get_param("~enabled", False))
        self.enable_topic = rospy.get_param(
            "~enable_topic", "/final_mission/tracker_enable"
        )
        self.unknown_task_policy = str(
            rospy.get_param("~unknown_task_policy", "skip")
        ).strip().lower()
        self.detect_pause_time = float(rospy.get_param("~detect_pause_time", 2.0))

        # ---------------- 停靠状态 ----------------
        self.task_phase = PHASE_NONE
        self.task_cursor = 0          # 已消费到第几个任务点（不可回退）
        self.active_task = None       # (index, task_string)
        self.pending_task_name = None
        self.pending_is_external = False
        self.task_deadline = None
        self.phase_started_at = None
        self.approach_snapshot = None  # 进 TASK 时的实际误差，用于日志

        self.task_points = self.build_task_points()
        self.original_external_names = {
            index: self.external_name(self.global_waypoints[index].task)
            for index in self.task_points
        }
        self.backtrack_authorized = False
        self.backtrack_mode = None
        self.backtrack_owner_task = None
        self.backtrack_expected_targets = ()
        self.backtrack_target_cursor = 0
        self.backtrack_target_index = None
        self.backtrack_target_pose = None
        self.backtrack_target_name = None
        self.backtrack_moving = False
        self.backtrack_started_at = None
        self.backtrack_segment_start = None
        self.backtrack_heading_correcting = False
        self.backtrack_heading_correction_started_at = None
        self.reverse_speed_abs = 0.0
        self.reverse_last_time = rospy.Time.now()
        self.warn_task_inside_zone()

        # ---------------- ROS ----------------
        self.task_event_pub = rospy.Publisher(
            self.task_event_topic, String, queue_size=10
        )
        self.tracker_state_pub = rospy.Publisher(
            "~phase", String, queue_size=1, latch=True
        )
        self.backtrack_event_pub = rospy.Publisher(
            self.backtrack_event_topic, String, queue_size=10
        )
        rospy.Subscriber(
            self.task_done_topic, String, self.task_done_callback, queue_size=10
        )
        rospy.Subscriber(
            self.arm_bridge_status_topic,
            String,
            self.arm_bridge_status_callback,
            queue_size=10,
        )
        rospy.Subscriber(self.skip_topic, String, self.skip_callback, queue_size=10)
        rospy.Subscriber(
            self.task_bypass_topic,
            String,
            self.task_bypass_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self.backtrack_command_topic,
            String,
            self.backtrack_command_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            self.arm_result_topic,
            String,
            self.arm_result_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            self.enable_topic, Bool, self.enable_callback, queue_size=5
        )

        self.log_startup()

    def enable_callback(self, msg):
        if bool(msg.data) == self.enabled:
            return
        self.enabled = bool(msg.data)
        rospy.logwarn("跟踪器 %s", "使能" if self.enabled else "禁用（发零速）")
        if not self.enabled:
            self.stop_robot()

    # ==================================================================
    # 初始化辅助
    # ==================================================================
    @staticmethod
    def external_name(task):
        task = (task or "").strip()
        if not task.lower().startswith("ext:"):
            return None
        name = task[4:].strip()
        return name or None

    def build_task_points(self):
        """预扫描 CSV，按弧长升序列出所有需要停车的任务点。

        父类已把 task 读进 AvoidanceWaypoint.task 并统一转成小写。
        avoid_start/avoid_end 是区间标记，不算任务点。
        """
        points = []
        for index, waypoint in enumerate(self.global_waypoints):
            task = (getattr(waypoint, "task", "none") or "none").strip()
            if task in ("", "none") or task.lower() in ZONE_MARKERS:
                continue
            points.append(index)
        return points

    def warn_task_inside_zone(self):
        """任务点落在避障区内会让 A* 重规划与精确停靠互相打架。"""
        bad = []
        for index in self.task_points:
            for start, end in self.avoidance_ranges:
                if start <= index <= end:
                    bad.append(self.global_waypoints[index].seq)
        if bad:
            rospy.logerr(
                "任务点落在避障区内，A* 重规划会与精确停靠冲突。"
                "请重录航迹使两者分离。seq=%s",
                ", ".join(str(s) for s in bad),
            )

    def log_startup(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("FinalTracker started (分区A*避障 + 任务点精确停靠 + 外部握手)")
        rospy.loginfo("任务点数量     : %d", len(self.task_points))
        for index in self.task_points:
            waypoint = self.global_waypoints[index]
            rospy.loginfo(
                "  seq=%-5d s=%6.2fm task=%s",
                waypoint.seq,
                self.route_cumulative_s[index],
                waypoint.task,
            )
        rospy.loginfo("触发余量/逼近  : %.2fm / %.2fm",
                      self.task_trigger_margin, self.approach_switch_dist)
        rospy.loginfo("到位判据       : %.3fm / %.3frad",
                      self.goal_reached_dist, self.task_yaw_tolerance)
        rospy.loginfo("逼近限速       : %.2f~%.2f m/s",
                      self.approach_min_speed, self.approach_max_speed)
        rospy.loginfo("逼近转向/近点  : %.2f rad/s / %.2fm",
                      self.approach_max_angular, self.approach_near_goal_dist)
        rospy.loginfo("=" * 60)

    def publish_phase(self, detail=""):
        self.tracker_state_pub.publish(
            String(data="%s:%s" % (self.task_phase, detail))
        )

    # ==================================================================
    # 任务点查找
    # ==================================================================
    def next_task_index(self):
        """返回下一个未消费任务点的航点索引；没有则 None。

        用游标而不是删除列表项：两轮 CSV 里 piper_stop_1 会出现两次，
        按值删除会让第二轮失效。
        """
        while self.task_cursor < len(self.task_points):
            index = self.task_points[self.task_cursor]
            task = (getattr(self.global_waypoints[index], "task", "none") or "none").strip()
            # navigation_skip 可能把 task 改成 none，此时跳过该点。
            if task in ("", "none"):
                self.task_cursor += 1
                continue
            return index
        return None

    def progress_projection_end_segment(self):
        """进度投影不得跨过下一个尚未完成的任务点。

        indoor.csv 的两轮航迹在空间上高度重合。父类仅按最近距离投影时，
        可能把第一轮当前位置匹配到第二轮或出场段。这里用任务游标形成硬边界；
        navigation_skip 已改成 none 的候选点不构成边界。
        """
        default_end = super().progress_projection_end_segment()
        if not hasattr(self, "task_points") or not hasattr(self, "task_cursor"):
            return default_end

        cursor = self.task_cursor
        while cursor < len(self.task_points):
            task_index = self.task_points[cursor]
            task = (
                getattr(self.global_waypoints[task_index], "task", "none") or "none"
            ).strip()
            if task not in ("", "none"):
                return min(default_end, max(0, task_index - 1))
            cursor += 1
        return default_end

    def distance_to(self, index):
        waypoint = self.global_waypoints[index]
        return math.hypot(
            waypoint.x - self.current_x, waypoint.y - self.current_y
        )

    def sync_route_progress_to_completed_task(self):
        """任务点已实际到达，消费任务前同步父类的单调路线进度。

        APPROACH/ALIGN/TASK 期间由本类直接控制车辆，父类的路线投影不会运行。
        连续候选点之间虽然车辆真实前进了，route_progress_s 仍可能停在第一个
        候选点之前。这里使用当前 CSV 自动计算的累计弧长同步，不写死任何距离。
        """
        if self.active_task is None:
            return
        index = self.active_task[0]
        if index < 0 or index >= len(self.global_waypoints):
            return

        previous_s = self.route_progress_s
        previous_index = self.global_index
        task_s = self.route_cumulative_s[index]
        self.route_progress_s = max(self.route_progress_s, task_s)
        self.global_index = max(
            self.global_index,
            min(len(self.global_waypoints), index + 1),
        )
        rospy.loginfo(
            "任务点进度同步：seq=%d s=%.2fm（原 %.2fm），global_index %d -> %d",
            self.global_waypoints[index].seq,
            self.route_progress_s,
            previous_s,
            previous_index,
            self.global_index,
        )

    def consume_current_task(self):
        self.sync_route_progress_to_completed_task()
        self.task_cursor += 1
        self.active_task = None
        self.pending_task_name = None
        self.pending_is_external = False
        self.task_deadline = None
        self.approach_snapshot = None
        self.task_phase = PHASE_NONE
        # 清掉局部路径，强制父类重新规划，避免沿用停车前的旧路径。
        self.local_path = []
        self.local_index = 0
        self.last_full_plan = []
        self.blocked_since = None
        self.clear_backtrack_state()

    def clear_backtrack_state(self):
        self.backtrack_authorized = False
        self.backtrack_mode = None
        self.backtrack_owner_task = None
        self.backtrack_expected_targets = ()
        self.backtrack_target_cursor = 0
        self.backtrack_target_index = None
        self.backtrack_target_pose = None
        self.backtrack_target_name = None
        self.backtrack_moving = False
        self.backtrack_started_at = None
        self.backtrack_segment_start = None
        self.backtrack_heading_correcting = False
        self.backtrack_heading_correction_started_at = None
        self.reverse_speed_abs = 0.0
        self.reverse_last_time = rospy.Time.now()

    # ==================================================================
    # 停靠：APPROACH（P 控制点到点）
    # ==================================================================
    def handle_approach(self):
        index = self.active_task[0]
        waypoint = self.global_waypoints[index]
        dx = waypoint.x - self.current_x
        dy = waypoint.y - self.current_y
        distance = math.hypot(dx, dy)

        elapsed = (rospy.Time.now() - self.phase_started_at).to_sec()

        if distance <= self.goal_reached_dist:
            self.enter_align(distance, elapsed)
            return

        if elapsed >= self.approach_timeout:
            rospy.logwarn(
                "逼近超时 %.1fs：seq=%d 残余距离 %.3fm，带误差进入对齐。",
                elapsed, waypoint.seq, distance,
            )
            self.enter_align(distance, elapsed)
            return

        # P 控制直奔目标坐标。相比 Pure Pursuit 的前视点追踪，直线逼近
        # 横向外摆更小 —— 候选点之间机械臂侧伸，这一点很关键。
        target_heading = math.atan2(dy, dx)
        point_heading_error = wrap_to_pi(target_heading - self.current_yaw)

        # 距离目标很近时，几毫米横向残差会把 atan2 方向角突然放大，
        # 造成履带左右来回修正。进入近点区后，平滑地从“朝向目标坐标”
        # 过渡到“目标点记录航向”；到停车阈值附近基本只保持最终航向。
        heading_error = point_heading_error
        if distance < self.approach_near_goal_dist:
            near_span = max(
                self.approach_near_goal_dist - self.goal_reached_dist, 1e-6
            )
            point_weight = clamp(
                (distance - self.goal_reached_dist) / near_span, 0.0, 1.0
            )
            final_yaw_error = wrap_to_pi(waypoint.yaw - self.current_yaw)
            heading_error = wrap_to_pi(
                final_yaw_error
                + point_weight
                * wrap_to_pi(point_heading_error - final_yaw_error)
            )

        linear = clamp(
            self.approach_k_linear * distance,
            self.approach_min_speed,
            self.approach_max_speed,
        )
        angular = clamp(
            1.2 * heading_error,
            -self.approach_max_angular,
            self.approach_max_angular,
        )

        # 航向偏差过大时先转正再前进，避免画弧线撞到桌子。
        if abs(heading_error) > 0.8:
            linear = 0.0
            angular = clamp(
                self.task_yaw_k * heading_error,
                -self.approach_max_angular,
                self.approach_max_angular,
            )
            if abs(angular) < self.task_yaw_min_angular:
                angular = math.copysign(self.task_yaw_min_angular, heading_error)

        self.publish_cmd(linear, angular)
        self.publish_phase("seq%d d=%.3f" % (waypoint.seq, distance))

    # ==================================================================
    # 停靠：ALIGN_YAW（原地自转对齐录制朝向）
    # ==================================================================
    def enter_align(self, distance, approach_elapsed):
        self.stop_robot()
        index = self.active_task[0]
        waypoint = self.global_waypoints[index]
        self.approach_snapshot = {
            "distance": distance,
            "dx": waypoint.x - self.current_x,
            "dy": waypoint.y - self.current_y,
            "approach_time": approach_elapsed,
        }
        self.task_phase = PHASE_ALIGN
        self.phase_started_at = rospy.Time.now()

    def handle_align(self):
        index = self.active_task[0]
        waypoint = self.global_waypoints[index]
        yaw_error = wrap_to_pi(waypoint.yaw - self.current_yaw)
        elapsed = (rospy.Time.now() - self.phase_started_at).to_sec()

        if abs(yaw_error) <= self.task_yaw_tolerance:
            self.enter_task(yaw_error)
            return

        if elapsed >= self.align_timeout:
            rospy.logwarn(
                "对齐超时 %.1fs：seq=%d 残余朝向误差 %.3frad(%.1f°)，直接进入任务。",
                elapsed, waypoint.seq, yaw_error, math.degrees(yaw_error),
            )
            self.enter_task(yaw_error)
            return

        angular = clamp(
            self.task_yaw_k * yaw_error,
            -self.task_yaw_max_angular,
            self.task_yaw_max_angular,
        )
        # 角速度死区：太小履带不转，误差永远收不敛。
        if abs(angular) < self.task_yaw_min_angular:
            angular = math.copysign(self.task_yaw_min_angular, yaw_error)

        self.publish_cmd(0.0, angular)
        self.publish_phase("seq%d yaw=%.3f" % (waypoint.seq, yaw_error))

    # ==================================================================
    # 停靠：TASK（发事件 + 阻塞等 done）
    # ==================================================================
    def enter_task(self, yaw_error):
        self.stop_robot()
        index, task = self.active_task
        waypoint = self.global_waypoints[index]

        snap = self.approach_snapshot or {}
        # 这条日志是误差数据的唯一来源：每次试跑自动积累，
        # 抓取成功率上不去时据此分解原因，不需要单独的测量环节。
        rospy.loginfo(
            "[TASK] %s @seq%d: 位置误差 %.3fm (dx=%+.3f dy=%+.3f), "
            "朝向误差 %.3frad(%.1f°), 逼近耗时 %.1fs",
            task, waypoint.seq,
            snap.get("distance", float("nan")),
            snap.get("dx", float("nan")),
            snap.get("dy", float("nan")),
            yaw_error, math.degrees(yaw_error),
            snap.get("approach_time", float("nan")),
        )

        self.task_phase = PHASE_TASK
        self.phase_started_at = rospy.Time.now()
        self.dispatch_task(task)

    def dispatch_task(self, task):
        """按 task 字符串分派。avoid_start/avoid_end 已在 build_task_points 排除。"""
        stop_seconds = self.parse_stop_seconds(task)
        if stop_seconds is not None:
            rospy.loginfo("定时停车 %s：%.1fs", task, stop_seconds)
            self.pending_task_name = task
            self.pending_is_external = False
            self.task_deadline = rospy.Time.now() + rospy.Duration(stop_seconds)
            self.publish_task_event("start", task)
            return

        if task.lower() == "detect":
            self.pending_task_name = task
            self.pending_is_external = False
            self.task_deadline = rospy.Time.now() + rospy.Duration(
                self.detect_pause_time
            )
            self.publish_task_event("start", task)
            return

        name = self.external_name(task)
        if name:
            rospy.loginfo("外部任务 %s：等待 %s 上的完成信号。",
                          name, self.task_done_topic)
            self.pending_task_name = name
            self.pending_is_external = True
            self.task_deadline = (
                rospy.Time.now() + rospy.Duration(self.external_task_timeout)
                if self.external_task_timeout > 0.0 else None
            )
            self.publish_task_event("start", name)
            return

        if self.unknown_task_policy == "hold":
            rospy.logwarn("未知任务 %s：按 hold 策略停 %.1fs。",
                          task, self.detect_pause_time)
            self.pending_task_name = task
            self.pending_is_external = False
            self.task_deadline = rospy.Time.now() + rospy.Duration(
                self.detect_pause_time
            )
            self.publish_task_event("start", task)
            return

        rospy.logwarn("未知任务 %s：按 skip 策略跳过。", task)
        self.publish_task_event("skip", task)
        self.consume_current_task()

    @staticmethod
    def parse_stop_seconds(task):
        import re
        match = re.match(
            r"(?:stop|hold|pause)_(\d+(?:\.\d+)?)([sm])$", (task or "").strip().lower()
        )
        if not match:
            return None
        value = float(match.group(1))
        return value * 60.0 if match.group(2) == "m" else value

    def publish_task_event(self, phase, name):
        index = self.active_task[0] if self.active_task else -1
        self.task_event_pub.publish(
            String(data="%s:%s:idx%d" % (phase, name, index))
        )

    def handle_task(self):
        # 停车必须走 stop_robot()：它设 force_stop_cmd 绕过父类的减速斜率限幅，
        # 保证立即到零。
        self.stop_robot()

        if self.pending_is_external and self.task_deadline is None:
            return  # 无限等待外部完成信号

        if self.task_deadline is None:
            self.finish_task("no_deadline")
            return

        if rospy.Time.now() >= self.task_deadline:
            if self.pending_is_external:
                rospy.logwarn(
                    "外部任务 %s 超时 %.1fs，放行继续。",
                    self.pending_task_name, self.external_task_timeout,
                )
            self.finish_task("timeout_or_done")

    def finish_task(self, reason):
        name = self.pending_task_name or "none"
        rospy.loginfo("任务 %s 结束（%s），恢复跟踪。", name, reason)
        self.publish_task_event("done", name)
        self.consume_current_task()

    def task_done_callback(self, msg):
        if self.task_phase != PHASE_TASK or not self.pending_is_external:
            return
        if not self.pending_task_name:
            return
        done = msg.data.strip()
        if not done:
            return
        expected = "done:%s" % self.pending_task_name
        if done in ("done", "all_done", self.pending_task_name, expected):
            rospy.loginfo("收到外部完成信号：%s", done)
            self.task_deadline = rospy.Time.now()

    def arm_bridge_status_callback(self, msg):
        """丢弃/收臂期间禁止150秒兜底越过机械臂回零确认。"""
        status = (msg.data or "").strip().lower()
        if not status.startswith(("place_backtrack:", "place_discard:")):
            return
        if (
            self.task_phase != PHASE_TASK
            or not self.pending_is_external
            or self.pending_task_name != "piper_stop_7"
        ):
            return
        if self.task_deadline is not None:
            rospy.logwarn(
                "放置失败进入倒车/降级/回零安全恢复："
                "关闭本任务自动超时，等待完整恢复确认。"
            )
        self.task_deadline = None

    # ==================================================================
    # BACKTRACK_RECOVERY：仅第三个抓取/放置点指定失败后允许的低速倒车
    # ==================================================================
    def arm_result_callback(self, msg):
        """由跟踪器亲自核验失败结果；没有该授权，任何命令都不能产生负速度。"""
        result = (msg.data or "").strip()
        if result == BACKTRACK_PICK3_FAILURE:
            expected_targets = BACKTRACK_TARGETS
            mode = "pick"
            owner_task = "piper_stop_4"
            failure_text = "三个候选点均未找到目标"
        elif result.startswith(BACKTRACK_PICK3_EDGE_FAILURE_PREFIX):
            expected_targets = BACKTRACK_EDGE_TARGETS
            mode = "pick"
            owner_task = "piper_stop_4"
            edge_direction = result[len(BACKTRACK_PICK3_EDGE_FAILURE_PREFIX):]
            failure_text = "第三点目标位于画面边缘(%s)" % (
                edge_direction or "unknown"
            )
        elif result == BACKTRACK_PLACE3_FAILURE:
            expected_targets = PLACE_BACKTRACK_TARGETS
            mode = "place"
            owner_task = "piper_stop_7"
            failure_text = "三个放置候选点均未找到正确目标"
        else:
            return
        if (
            self.task_phase != PHASE_TASK
            or not self.pending_is_external
            or self.pending_task_name != owner_task
        ):
            rospy.logwarn(
                "收到 %s 恢复失败结果，但当前并非 %s，拒绝倒车授权。",
                mode,
                owner_task,
            )
            return

        self.backtrack_authorized = True
        self.backtrack_mode = mode
        self.backtrack_owner_task = owner_task
        self.backtrack_expected_targets = expected_targets
        self.backtrack_target_cursor = 0
        rospy.logwarn(
            "已核验 %s 的 %s 失败（%s）："
            "仅本次任务开放 BACKTRACK_RECOVERY。",
            owner_task,
            mode,
            failure_text,
        )

    def backtrack_command_callback(self, msg):
        fields = (msg.data or "").strip().split(":", 1)
        if len(fields) != 2:
            return
        command, value = fields[0].strip().lower(), fields[1].strip()
        if command == "reverse":
            self.begin_backtrack_segment(value)
        elif command == "complete":
            self.complete_backtrack_recovery(value)

    def backtrack_index(self, target_name):
        """在当前失败点之前、本轮 piper_stop_1 之后查找回退目标点。"""
        if self.active_task is None:
            return None
        active_index = self.active_task[0]
        for index in reversed(self.task_points):
            if index >= active_index:
                continue
            name = self.original_external_names.get(index)
            if name == target_name:
                return index
            if name == "piper_stop_1":
                break
        return None

    def publish_backtrack_event(self, status, target, reason=""):
        payload = "%s:%s" % (status, target)
        if reason:
            payload += ":%s" % reason
        self.backtrack_event_pub.publish(String(data=payload))

    def reject_backtrack(self, target, reason):
        self.stop_robot()
        self.reverse_speed_abs = 0.0
        self.backtrack_moving = False
        self.backtrack_heading_correcting = False
        self.backtrack_heading_correction_started_at = None
        rospy.logerr("拒绝/终止倒车至 %s：%s", target or "unknown", reason)
        self.publish_backtrack_event("failed", target or "unknown", reason)

    def begin_backtrack_segment(self, target_name):
        """开始一段失败恢复倒车；正常任务状态永远不能进入这里。"""
        expected = (
            self.backtrack_expected_targets[self.backtrack_target_cursor]
            if self.backtrack_target_cursor < len(self.backtrack_expected_targets)
            else None
        )
        if not self.backtrack_authorized:
            self.reject_backtrack(target_name, "not_authorized_by_arm_failure")
            return
        if (
            self.pending_task_name != self.backtrack_owner_task
            or self.task_phase not in (PHASE_TASK, PHASE_BACKTRACK)
        ):
            self.reject_backtrack(
                target_name,
                "not_waiting_at_%s" % (self.backtrack_owner_task or "owner_task"),
            )
            return
        if self.backtrack_moving:
            self.reject_backtrack(target_name, "already_reversing")
            return
        if target_name != expected:
            self.reject_backtrack(
                target_name, "unexpected_target_expected_%s" % (expected or "none")
            )
            return
        if not self.has_odom or self.localization_fault:
            self.reject_backtrack(target_name, "localization_not_ready")
            return

        target_index = None
        if target_name == BACKTRACK_RECOVERY_MID:
            # 恢复点不写入正常航迹，也不会在前进流程触发。它只在已经核验的
            # pick3 边缘失败后，沿当前点到 pick2 的方向生成一次性短倒目标。
            anchor_index = self.backtrack_index("piper_stop_3")
            if anchor_index is None:
                self.reject_backtrack(
                    target_name, "recovery_anchor_not_found_in_current_round"
                )
                return
            anchor = self.global_waypoints[anchor_index]
            anchor_dx = anchor.x - self.current_x
            anchor_dy = anchor.y - self.current_y
            anchor_distance = math.hypot(anchor_dx, anchor_dy)
            # 控制器在距离目标 reverse_goal_dist 时判定到达，因此临时目标
            # 额外向后放一个 goal_dist，保证车辆实际短倒量接近配置的 0.14m。
            target_distance = min(
                self.reverse_recovery_mid_dist + self.reverse_goal_dist,
                max(0.0, anchor_distance - self.reverse_goal_dist),
            )
            if target_distance <= self.reverse_goal_dist:
                self.reject_backtrack(
                    target_name,
                    "insufficient_room_%.3fm_for_mid_recovery" % anchor_distance,
                )
                return
            scale = target_distance / anchor_distance
            target_pose = (
                self.current_x + anchor_dx * scale,
                self.current_y + anchor_dy * scale,
                self.current_yaw,
            )
            # 只保留 anchor index 用于诊断；实际控制始终使用上面的临时位姿。
            target_index = anchor_index
            distance = target_distance
        else:
            target_index = self.backtrack_index(target_name)
            if target_index is None:
                self.reject_backtrack(target_name, "target_not_found_in_current_round")
                return
            waypoint = self.global_waypoints[target_index]
            target_pose = (waypoint.x, waypoint.y, waypoint.yaw)
            distance = math.hypot(
                target_pose[0] - self.current_x,
                target_pose[1] - self.current_y,
            )

        # 倒车时车头应朝向“目标方向的反方向”。使用本段起点到目标点的
        # 几何方向作为航向基准，不直接使用录制点的 yaw，避免两个点之间
        # 很小的录制航向差在短距离倒车中被放大。
        segment_dx = target_pose[0] - self.current_x
        segment_dy = target_pose[1] - self.current_y
        reverse_heading = wrap_to_pi(
            math.atan2(segment_dy, segment_dx) + math.pi
        )
        target_pose = (target_pose[0], target_pose[1], reverse_heading)

        if distance > self.reverse_max_segment_dist:
            self.reject_backtrack(
                target_name,
                "segment_too_long_%.3fm_limit_%.3fm"
                % (distance, self.reverse_max_segment_dist),
            )
            return

        xb, _ = self.point_in_body(target_pose[0], target_pose[1])
        if xb >= -self.reverse_goal_dist:
            self.reject_backtrack(target_name, "target_not_behind_vehicle")
            return

        self.stop_robot()
        self.task_phase = PHASE_BACKTRACK
        self.backtrack_target_index = target_index
        self.backtrack_target_pose = target_pose
        self.backtrack_target_name = target_name
        self.backtrack_moving = True
        self.backtrack_started_at = rospy.Time.now()
        self.backtrack_segment_start = (self.current_x, self.current_y)
        self.backtrack_heading_correcting = False
        self.backtrack_heading_correction_started_at = None
        self.reverse_speed_abs = 0.0
        self.reverse_last_time = rospy.Time.now()
        if target_name == BACKTRACK_RECOVERY_MID:
            rospy.logwarn(
                "BACKTRACK_RECOVERY 开始：%s -> %s，临时目标距离 %.3fm，"
                "预计实际短倒 %.3fm；该负速度授权只对本段有效。",
                self.pending_task_name,
                target_name,
                distance,
                max(0.0, distance - self.reverse_goal_dist),
            )
        else:
            rospy.logwarn(
                "BACKTRACK_RECOVERY 开始：%s -> %s，距离 %.3fm；"
                "该负速度授权只对本段有效。",
                self.pending_task_name,
                target_name,
                distance,
            )
        rospy.loginfo(
            "倒车段采用几何航向 %.3frad；偏差超过 %.3frad 时停车纠偏，"
            "回到 %.3frad 内再继续倒退。",
            reverse_heading,
            self.reverse_yaw_slow_threshold,
            self.reverse_yaw_resume_threshold,
        )

    def complete_backtrack_recovery(self, outcome):
        """核验恢复结果，消费原失败任务点并恢复只前进跟踪。"""
        allowed_outcomes = {
            "pick": ("payload", "no_payload"),
            "place": ("placed", "discarded"),
        }
        if outcome not in allowed_outcomes.get(self.backtrack_mode, ()):
            rospy.logerr("忽略未知的恢复结果：%s", outcome)
            return
        if (
            self.pending_task_name != self.backtrack_owner_task
            or self.task_phase not in (PHASE_TASK, PHASE_BACKTRACK)
        ):
            rospy.logerr("忽略无对应失败任务的恢复完成命令：%s", outcome)
            return
        if not self.backtrack_authorized:
            rospy.logerr("忽略未由指定机械臂失败授权的恢复完成命令。")
            return

        self.stop_robot()
        rospy.loginfo("BACKTRACK_RECOVERY 完成（%s），恢复正常前向路线。", outcome)
        self.publish_task_event("done", self.pending_task_name)
        self.consume_current_task()

    def backtrack_path_metrics(self):
        """返回段长、沿段已倒距离、沿段剩余距离和横向偏差。"""
        if self.backtrack_segment_start is None or self.backtrack_target_pose is None:
            return (float("inf"),) * 4
        sx, sy = self.backtrack_segment_start
        target_x, target_y, _ = self.backtrack_target_pose
        vx = target_x - sx
        vy = target_y - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return 0.0, 0.0, 0.0, 0.0
        current_dx = self.current_x - sx
        current_dy = self.current_y - sy
        progress = (current_dx * vx + current_dy * vy) / length
        remaining = length - progress
        cross_track = abs(
            vx * (self.current_y - sy) - vy * (self.current_x - sx)
        ) / length
        return length, progress, remaining, cross_track

    def backtrack_cross_track_error(self):
        return self.backtrack_path_metrics()[3]

    def reverse_obstacle_data_ready(self):
        if not self.reverse_require_obstacles:
            return True
        if self.obstacle_source == "none" or self.obstacle_stamp == rospy.Time(0):
            return False
        return (
            rospy.Time.now() - self.obstacle_stamp
        ).to_sec() <= self.obstacle_timeout

    def rear_clearance(self):
        """返回车尾保险杠到后方走廊内最近障碍物的净距离。"""
        obstacles = self.get_obstacles_snapshot()
        if not obstacles:
            return float("inf")
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        rear_edge = -self.footprint_rear
        minimum = float("inf")
        for ox, oy in obstacles:
            rx = ox - self.current_x
            ry = oy - self.current_y
            xb = cy * rx + sy * ry
            yb = -sy * rx + cy * ry
            clearance = rear_edge - xb
            if clearance > 0.0 and abs(yb) <= self.reverse_safety_half_width:
                minimum = min(minimum, clearance)
        return minimum

    def publish_failure_only_reverse(self, linear_x, angular_z):
        """唯一允许发布负线速度的出口，带双重状态授权检查。"""
        if (
            not self.backtrack_authorized
            or self.task_phase != PHASE_BACKTRACK
            or not self.backtrack_moving
            or linear_x >= 0.0
        ):
            self.stop_robot()
            return
        msg = Twist()
        msg.linear.x = max(-self.reverse_max_speed, float(linear_x))
        msg.angular.z = clamp(
            float(angular_z), -self.reverse_max_angular, self.reverse_max_angular
        )
        self.cmd_pub.publish(msg)
        # 正常跟踪恢复时从零速重新起步，不继承负速度历史。
        self.last_cmd_linear = msg.linear.x
        self.last_cmd_angular = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    def publish_failure_only_heading_correction(self, angular_z):
        """失败恢复倒车段内原地纠偏；线速度严格保持为零。"""
        if (
            not self.backtrack_authorized
            or self.task_phase != PHASE_BACKTRACK
            or not self.backtrack_moving
        ):
            self.stop_robot()
            return
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = clamp(
            float(angular_z), -self.reverse_max_angular, self.reverse_max_angular
        )
        self.cmd_pub.publish(msg)
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    def handle_backtrack(self):
        if not self.backtrack_moving:
            self.stop_robot()
            self.publish_phase("waiting_arm")
            return
        if self.backtrack_target_pose is None or not self.backtrack_target_name:
            self.reject_backtrack("unknown", "missing_target")
            return

        target_x, target_y, target_yaw = self.backtrack_target_pose
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)
        elapsed = (rospy.Time.now() - self.backtrack_started_at).to_sec()
        segment_length, progress, along_remaining, cross_track = (
            self.backtrack_path_metrics()
        )

        # 对短距离倒车，二维圆形到达条件会因几厘米横向误差而漏判：车辆已经
        # 倒够距离后目标转到车侧方，旧逻辑反而报 target_left_rear_half_plane。
        # 因此同时使用沿本段方向的进度；达到目标前 goal_dist 且横向仍在安全
        # 范围内就立即停车。对异常定位跳变造成的大幅越界仍保持失败保护。
        arrived_by_progress = (
            math.isfinite(along_remaining)
            and along_remaining <= self.reverse_goal_dist
            and along_remaining >= -self.reverse_arrival_overshoot_max
            and cross_track <= self.reverse_arrival_cross_track_max
        )

        if distance <= self.reverse_goal_dist or arrived_by_progress:
            arrived_name = self.backtrack_target_name
            arrival_method = (
                "euclidean"
                if distance <= self.reverse_goal_dist
                else "along_track"
            )
            self.stop_robot()
            self.backtrack_moving = False
            self.backtrack_heading_correcting = False
            self.backtrack_heading_correction_started_at = None
            self.reverse_speed_abs = 0.0
            self.backtrack_target_cursor += 1
            rospy.loginfo(
                "倒车到达 %s：判定=%s，二维误差 %.3fm，沿段进度 "
                "%.3f/%.3fm，横向误差 %.3fm，耗时 %.1fs。",
                arrived_name,
                arrival_method,
                distance,
                progress,
                segment_length,
                cross_track,
                elapsed,
            )
            self.publish_phase("arrived:%s" % arrived_name)
            self.publish_backtrack_event("arrived", arrived_name)
            return
        if elapsed >= self.reverse_timeout:
            self.reject_backtrack(
                self.backtrack_target_name, "timeout_%.1fs" % elapsed
            )
            return

        xb, yb = self.point_in_body(target_x, target_y)
        if xb >= -self.reverse_goal_dist:
            self.reject_backtrack(
                self.backtrack_target_name,
                "target_left_rear_half_plane_progress_%.3f_remaining_%.3f_cross_%.3f"
                % (progress, along_remaining, cross_track),
            )
            return

        yaw_error = wrap_to_pi(target_yaw - self.current_yaw)
        if abs(yaw_error) > self.reverse_max_yaw_error:
            self.reject_backtrack(
                self.backtrack_target_name,
                "yaw_error_%.3frad_limit_%.3frad"
                % (yaw_error, self.reverse_max_yaw_error),
            )
            return

        if cross_track > self.reverse_cross_track_max:
            self.reject_backtrack(
                self.backtrack_target_name,
                "cross_track_%.3fm_limit_%.3fm"
                % (cross_track, self.reverse_cross_track_max),
            )
            return

        if not self.reverse_obstacle_data_ready():
            self.reject_backtrack(
                self.backtrack_target_name, "rear_obstacle_data_missing_or_stale"
            )
            return
        rear_clear = self.rear_clearance()
        if rear_clear <= self.reverse_rear_stop_dist:
            self.reject_backtrack(
                self.backtrack_target_name,
                "rear_obstacle_%.3fm_limit_%.3fm"
                % (rear_clear, self.reverse_rear_stop_dist),
            )
            return

        now = rospy.Time.now()
        needs_heading_correction = (
            abs(yaw_error) > self.reverse_yaw_slow_threshold
            or (
                self.backtrack_heading_correcting
                and abs(yaw_error) > self.reverse_yaw_resume_threshold
            )
        )
        if needs_heading_correction:
            if not self.backtrack_heading_correcting:
                self.backtrack_heading_correcting = True
                self.backtrack_heading_correction_started_at = now
                rospy.logwarn(
                    "倒车航向误差 %.3frad：先停车原地纠偏，禁止带偏继续倒退。",
                    yaw_error,
                )
            correction_elapsed = (
                now - self.backtrack_heading_correction_started_at
            ).to_sec()
            if correction_elapsed >= self.reverse_heading_correction_timeout:
                self.reject_backtrack(
                    self.backtrack_target_name,
                    "heading_correction_timeout_%.1fs_error_%.3frad"
                    % (correction_elapsed, yaw_error),
                )
                return
            self.reverse_speed_abs = 0.0
            self.reverse_last_time = now
            correction_angular = clamp(
                self.reverse_yaw_k * yaw_error,
                -self.reverse_max_angular,
                self.reverse_max_angular,
            )
            self.publish_failure_only_heading_correction(correction_angular)
            self.publish_phase(
                "%s heading_correction yaw=%.3f rear=%.3f"
                % (self.backtrack_target_name, yaw_error, rear_clear)
            )
            return

        if self.backtrack_heading_correcting:
            rospy.loginfo(
                "倒车航向已恢复至 %.3frad，继续倒退至 %s。",
                yaw_error,
                self.backtrack_target_name,
            )
            self.backtrack_heading_correcting = False
            self.backtrack_heading_correction_started_at = None
            self.reverse_speed_abs = 0.0
            self.reverse_last_time = now

        desired_abs = clamp(
            self.reverse_k_linear * distance,
            self.reverse_min_speed,
            self.reverse_max_speed,
        )
        if self.backtrack_target_name == BACKTRACK_RECOVERY_MID:
            desired_abs = min(desired_abs, self.reverse_recovery_mid_max_speed)
        dt = (now - self.reverse_last_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(self.control_rate, 1.0)
        self.reverse_speed_abs = min(
            desired_abs, self.reverse_speed_abs + self.reverse_accel * dt
        )
        self.reverse_last_time = now
        linear = -self.reverse_speed_abs

        # 后视 Pure Pursuit：目标在车体后方，负线速度乘曲率后会产生正确转向。
        curvature = 2.0 * yb / max(distance * distance, 0.01)
        angular = linear * curvature + self.reverse_yaw_k * yaw_error
        angular = clamp(
            angular, -self.reverse_max_angular, self.reverse_max_angular
        )
        self.publish_failure_only_reverse(linear, angular)
        self.publish_phase(
            "%s d=%.3f cross=%.3f rear=%.3f"
            % (self.backtrack_target_name, distance, cross_track, rear_clear)
        )

    # ==================================================================
    # 动态旁路：把尚未到达的任务点恢复成普通巡航点
    # ==================================================================
    def task_bypass_callback(self, msg):
        """旁路下一个指定外部任务；当前正在执行的任务绝不会被旁路。"""
        requested_name = (msg.data or "").strip().lower()
        if not requested_name:
            return

        start = self.task_cursor
        if (
            self.active_task is not None
            and start < len(self.task_points)
            and self.task_points[start] == self.active_task[0]
        ):
            start += 1

        for cursor in range(start, len(self.task_points)):
            index = self.task_points[cursor]
            waypoint = self.global_waypoints[index]
            name = self.external_name(getattr(waypoint, "task", "none"))
            if name and name.lower() == requested_name:
                waypoint.task = "none"
                rospy.logwarn(
                    "动态旁路：%s@seq%d 已并入普通巡航，"
                    "不会触发减速、停车、对齐或外部握手。",
                    name,
                    waypoint.seq,
                )
                return

        rospy.logwarn("动态旁路未找到后续任务：%s", requested_name)

    # ==================================================================
    # navigation_skip：抓/放成功后，后续候选点无需停车
    # ==================================================================
    def skip_callback(self, msg):
        skip_names = {
            item.strip() for item in (msg.data or "").split(",") if item.strip()
        }
        if not skip_names:
            return  # piper_task 每轮开始会发空集清状态

        changed = []
        start = self.task_cursor
        for cursor in range(start, len(self.task_points)):
            index = self.task_points[cursor]
            waypoint = self.global_waypoints[index]
            name = self.external_name(getattr(waypoint, "task", "none"))
            # 只改本轮：撞到下一个 piper_stop_1 就停，避免影响第二轮。
            if cursor > start and name == "piper_stop_1":
                break
            if name in skip_names:
                waypoint.task = "none"
                changed.append("%s@seq%d" % (name, waypoint.seq))

        if changed:
            rospy.loginfo("后续候选点并入巡航，不停车：%s", ", ".join(changed))

    # ==================================================================
    # 主控制循环
    # ==================================================================
    def control_step(self):
        # 起步闸门：未使能时持续发零速，保持在起点不动。
        if not self.enabled:
            self.stop_robot()
            return

        # 定位故障时父类会锁死，这里也不进入停靠流程。
        if self.localization_fault:
            super().control_step()
            return

        if not self.has_odom:
            super().control_step()
            return

        # ---- 停靠流程优先，接管速度权 ----
        if self.task_phase == PHASE_APPROACH:
            self.handle_approach()
            return
        if self.task_phase == PHASE_ALIGN:
            self.handle_align()
            return
        if self.task_phase == PHASE_TASK:
            self.handle_task()
            return
        if self.task_phase == PHASE_BACKTRACK:
            self.handle_backtrack()
            return

        # ---- 检查是否该进入停靠 ----
        index = self.next_task_index()
        if index is not None:
            distance = self.distance_to(index)
            if distance <= self.approach_switch_dist:
                waypoint = self.global_waypoints[index]
                rospy.loginfo(
                    "进入逼近：seq=%d task=%s 距离 %.3fm",
                    waypoint.seq, waypoint.task, distance,
                )
                self.active_task = (index, waypoint.task)
                self.task_phase = PHASE_APPROACH
                self.phase_started_at = rospy.Time.now()
                self.handle_approach()
                return

            # ---- SLOWDOWN：巡航中按距离线性降速 ----
            # 父类 control_step 会在开头把 target_speed 重置为 nominal_target_speed，
            # 所以这里改 nominal 而不是 target。
            if distance <= self.task_trigger_margin:
                span = max(self.task_trigger_margin - self.approach_switch_dist, 1e-6)
                ratio = clamp(
                    (distance - self.approach_switch_dist) / span, 0.0, 1.0
                )
                original = self.nominal_target_speed
                self.nominal_target_speed = max(
                    self.approach_max_speed,
                    self.approach_max_speed
                    + (original - self.approach_max_speed) * ratio,
                )
                try:
                    super().control_step()
                finally:
                    self.nominal_target_speed = original
                return

        super().control_step()

    def on_shutdown(self):
        super().on_shutdown()
        rospy.loginfo("FinalTracker shutdown.")


if __name__ == "__main__":
    rospy.init_node("final_tracker", anonymous=False)
    try:
        FinalTracker().spin()
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        rospy.logfatal("FinalTracker 启动失败: %s", exc)
        raise
