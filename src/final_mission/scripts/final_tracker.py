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


class FinalTracker(AvoidanceZoneAStarTest):
    def __init__(self):
        super().__init__()

        # ---------------- 停靠参数 ----------------
        self.task_trigger_margin = float(rospy.get_param("~task_trigger_margin", 0.80))
        self.approach_switch_dist = float(rospy.get_param("~approach_switch_dist", 0.45))
        self.approach_k_linear = float(rospy.get_param("~approach_k_linear", 0.5))
        self.approach_max_speed = float(rospy.get_param("~approach_max_speed", 0.10))
        self.approach_min_speed = float(rospy.get_param("~approach_min_speed", 0.04))
        self.goal_reached_dist = float(rospy.get_param("~goal_reached_dist", 0.05))

        self.task_yaw_tolerance = float(rospy.get_param("~task_yaw_tolerance", 0.05))
        self.task_yaw_max_correction = float(
            rospy.get_param("~task_yaw_max_correction", 0.10)
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
        self.external_task_timeout = float(
            rospy.get_param("~external_task_timeout", 150.0)
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
        self.warn_task_inside_zone()

        # ---------------- ROS ----------------
        self.task_event_pub = rospy.Publisher(
            self.task_event_topic, String, queue_size=10
        )
        self.tracker_state_pub = rospy.Publisher(
            "~phase", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            self.task_done_topic, String, self.task_done_callback, queue_size=10
        )
        rospy.Subscriber(self.skip_topic, String, self.skip_callback, queue_size=10)
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

    def distance_to(self, index):
        waypoint = self.global_waypoints[index]
        return math.hypot(
            waypoint.x - self.current_x, waypoint.y - self.current_y
        )

    def consume_current_task(self):
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
        heading_error = wrap_to_pi(target_heading - self.current_yaw)

        linear = clamp(
            self.approach_k_linear * distance,
            self.approach_min_speed,
            self.approach_max_speed,
        )
        angular = clamp(1.2 * heading_error, -self.max_angular, self.max_angular)

        # 航向偏差过大时先转正再前进，避免画弧线撞到桌子。
        if abs(heading_error) > 0.8:
            linear = 0.0
            angular = clamp(
                self.task_yaw_k * heading_error,
                -self.max_angular,
                self.max_angular,
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

        # 机械臂在候选点之间保持 piper_task 的 PICK_SCAN 姿态
        # （piper_task_node.py:32，joint1=-1.530rad=-87.7°，侧伸）。
        # 大幅自转会让臂尖横扫，有撞桌子/扫落物品的风险
        # （规则：货物遗失该轮卸货也 0 分）。
        # 宁可带朝向误差让视觉补偿，也不要伸着臂大转。
        # 注意：本上限 0.10rad 是按臂尖伸出约 0.6m 估算的（估值，未实测），
        # 实测臂尖伸出距离后可据此调整。
        if abs(yaw_error) > self.task_yaw_max_correction:
            rospy.logerr(
                "朝向误差 %.3frad(%.1f°) 超过纠正上限 %.3frad：seq=%d 跳过对齐。"
                "机械臂可能侧伸，大幅自转有碰撞风险。请检查航迹录制朝向。",
                yaw_error, math.degrees(yaw_error),
                self.task_yaw_max_correction, waypoint.seq,
            )
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
            self.task_yaw_k * yaw_error, -self.max_angular, self.max_angular
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
