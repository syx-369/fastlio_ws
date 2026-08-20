#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂任务桥：原地重试、失败放行与取货回退恢复。

piper_task 自己订阅 /waypoint_task_event，车一停就直接执行 card/pick/place，
本节点不抢这个职责，只补充 piper_task 缺失的三件事：

1. 普通动作原地重试。card/place 的可恢复视觉失败仍按配置在原地重试。

2. 取货回退恢复。第三点仍没有安全抓取位姿时，不在第三点原地重复：
   若正确目标位于画面边缘，保持抓取观察位并短倒约 0.14m 复查；仍失败再
   依次倒回第二、第一候选点。完全没有检测到目标时直接倒回前两个候选点。
   每一段倒车都先确认机械臂保持抓取观察位；跟踪器也只在亲自收到指定的
   pick3 失败结果后才接受负速度。

3. 失败放行。piper_task 只在 ok=True 时发 done，
   失败路径不发，车会一直卡到跟踪器的 external_task_timeout 才走。
   普通动作重试用尽后代发 done；取货回退也失败则执行 abort_round，
   收臂并标记 no_payload，再让车辆继续经过卸货区。

4. 放置回退恢复。三个放置候选点均未找到正确目标时，保持持物观察位，
   倒回第二、第一放置点复查；仍失败则尝试放到任意安全识别物体处，最后才
   张爪丢弃。任何结束路径都必须确认回运输零位后才放行。

关键前提（已核实）：piper_task 的 command_callback 入队时 waypoint_task_name
为 None（piper_task_node.py:178），因此经 /piper_task/command 重发的命令
成功也不会自动发 done —— 放行时机完全由本节点掌握，不会与 piper_task 重复发。
"""

import re

import rospy
from std_msgs.msg import String


# piper_task 的结果格式：<success|failed|rejected>:<command>:<reason>
RESULT_PATTERN = re.compile(r"^(success|failed|rejected):([^:]+):(.*)$")

# 任务名 -> 重试计数归类
TASK_KIND = {
    "piper_stop_1": "card",
    "piper_stop_2": "pick",
    "piper_stop_3": "pick",
    "piper_stop_4": "pick",
    "piper_stop_5": "place",
    "piper_stop_6": "place",
    "piper_stop_7": "place",
}

BACKTRACK_PICK3_FAILURE = "object_not_found_at_all_pick_points"
BACKTRACK_PICK3_EDGE_FAILURE = "target_at_edge_at_final_pick_point"
BACKTRACK_RECOVERY_MID = "piper_recovery_mid"
BACKTRACK_TARGETS = ("piper_stop_3", "piper_stop_2")
BACKTRACK_EDGE_TARGETS = (
    BACKTRACK_RECOVERY_MID,
    "piper_stop_3",
    "piper_stop_2",
)
BACKTRACK_PICK_COMMANDS = {
    # 恢复中间点位于 pick2 与 pick3 之间，用 pick2 命令复查；未找到时
    # 它会返回可继续恢复的 success:not_here_continue_to_pick3。
    BACKTRACK_RECOVERY_MID: "pick2",
    "piper_stop_3": "pick2",
    "piper_stop_2": "pick1",
}
PLACE_BACKTRACK_TARGETS = ("piper_stop_6", "piper_stop_5")
PLACE_BACKTRACK_COMMANDS = {
    "piper_stop_6": "place2",
    "piper_stop_5": "place1",
}


class ArmBridge:
    def __init__(self):
        rospy.init_node("arm_bridge", anonymous=False)

        self.event_topic = rospy.get_param(
            "~task_event_topic", "/waypoint_task_event"
        )
        self.done_topic = rospy.get_param(
            "~task_done_topic", "/waypoint_task_done"
        )
        self.arm_command_topic = rospy.get_param(
            "~arm_command_topic", "/piper_task/command"
        )
        self.arm_result_topic = rospy.get_param(
            "~arm_result_topic", "/piper_task/result"
        )
        self.arm_state_topic = rospy.get_param(
            "~arm_state_topic", "/piper_task/state"
        )
        self.backtrack_command_topic = rospy.get_param(
            "~backtrack_command_topic", "/final_mission/backtrack_command"
        )
        self.backtrack_event_topic = rospy.get_param(
            "~backtrack_event_topic", "/final_mission/backtrack_event"
        )
        self.stop_actions = rospy.get_param("~vehicle_stop_actions", {})
        self.retry_counts = rospy.get_param(
            "~retry_counts", {"card": 2, "pick": 2, "place": 2}
        )
        self.retryable = set(rospy.get_param("~retryable_reasons", []))
        self.release_after_exhausted = bool(
            rospy.get_param("~release_after_retry_exhausted", True)
        )
        self.action_timeout = float(rospy.get_param("~arm_action_timeout", 60.0))
        self.resend_delay = float(rospy.get_param("~resend_delay", 1.0))
        self.place_discard_max_attempts = max(
            1, int(rospy.get_param("~place_discard_max_attempts", 3))
        )

        # 当前正在盯的停靠点
        self.active_task = None      # piper_stop_N
        self.active_command = None   # card / pick1 / place2 ...
        self.attempts = 0            # 已尝试次数（含首次）
        self.max_attempts = 1
        self.last_activity = None
        self.released = False
        self.resend_at = None
        self.scheduled_command = None

        # 第三个取货点失败后的专用恢复状态。
        self.recovery_active = False
        self.recovery_kind = None
        self.recovery_state = None
        self.recovery_target_cursor = 0
        self.recovery_targets = ()
        self.expected_recovery_command = None

        # 第三个放置点仍未找到目标后的“丢弃并收臂”安全收尾。
        self.place_discard_active = False
        self.place_discard_attempts = 0
        self.place_discard_hold = False

        self.done_pub = rospy.Publisher(self.done_topic, String, queue_size=10)
        self.command_pub = rospy.Publisher(
            self.arm_command_topic, String, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        self.backtrack_command_pub = rospy.Publisher(
            self.backtrack_command_topic, String, queue_size=10
        )

        rospy.Subscriber(
            self.event_topic, String, self.event_callback, queue_size=20
        )
        rospy.Subscriber(
            self.arm_result_topic, String, self.result_callback, queue_size=20
        )
        rospy.Subscriber(
            self.backtrack_event_topic,
            String,
            self.backtrack_event_callback,
            queue_size=20,
        )

        rospy.loginfo("=" * 60)
        rospy.loginfo("ArmBridge started (原地重试 + 取货回退 + 失败放行)")
        rospy.loginfo("重试次数       : %s", self.retry_counts)
        rospy.loginfo("可重试 reason  : %s", sorted(self.retryable) or "(未配置)")
        rospy.loginfo("重试用尽后放行 : %s", self.release_after_exhausted)
        rospy.loginfo("单次动作超时   : %.1fs", self.action_timeout)
        rospy.loginfo("=" * 60)

    # ------------------------------------------------------------------
    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def event_callback(self, msg):
        """跟踪器到点后发 start:<task>:idx<N>；piper_task 也收到同一条并开始执行。"""
        fields = (msg.data or "").strip().split(":", 2)
        if len(fields) < 2 or fields[0] != "start":
            return
        task = fields[1].strip()
        if task not in TASK_KIND:
            return

        kind = TASK_KIND[task]
        self.active_task = task
        self.active_command = str(self.stop_actions.get(task, "")).strip().lower()
        self.attempts = 1
        self.max_attempts = 1 + int(self.retry_counts.get(kind, 0))
        self.last_activity = rospy.Time.now()
        self.released = False
        self.resend_at = None
        self.scheduled_command = None
        self.recovery_active = False
        self.recovery_kind = None
        self.recovery_state = None
        self.recovery_target_cursor = 0
        self.recovery_targets = ()
        self.expected_recovery_command = None
        self.place_discard_active = False
        self.place_discard_attempts = 0
        self.place_discard_hold = False

        rospy.loginfo(
            "盯住 %s（动作 %s），最多 %d 次尝试。",
            task, self.active_command or "?", self.max_attempts,
        )
        self.publish_status("watching:%s:1/%d" % (task, self.max_attempts))

    def result_callback(self, msg):
        if self.active_task is None or self.released:
            return

        match = RESULT_PATTERN.match((msg.data or "").strip())
        if not match:
            return
        status, command, reason = match.groups()
        command = command.strip().lower()
        reason = reason.strip()

        if self.recovery_active:
            self.handle_recovery_result(status, command, reason)
            return

        if self.place_discard_active:
            self.handle_place_discard_result(status, command, reason)
            return

        # piper_task 的停靠事件预处理结果是另一种格式：
        #   <status>:waypoint:<task_name>:<reason>
        # 经 RESULT_PATTERN 解析后 command 恒为 "waypoint"，必须单独处理，
        # 否则会被下面的动作名比较过滤掉，盯守一直不撤。
        if command == "waypoint":
            self.handle_waypoint_result(status, reason)
            return

        # 重发命令时机械臂可能还没退出 busy（piper_task 先发结果再清 busy），
        # 此时会回 rejected:busy:<command>，reason 才是我们发的动作名。
        if status == "rejected" and reason == self.active_command:
            self.schedule_command(self.active_command, "rejected_%s" % command)
            return

        # 只处理当前盯住的动作。piper_task 也会为 status 查询等发结果。
        if self.active_command and command != self.active_command:
            return

        self.last_activity = rospy.Time.now()

        if status == "success":
            # piper_task 走 waypoint 触发路径时自己会发 done，这里不重复发。
            # 但重试成功是经 /piper_task/command 触发的，必须由本节点放行。
            if self.attempts > 1:
                rospy.loginfo(
                    "%s 第 %d 次尝试成功（%s），代发 done 放行。",
                    self.active_task, self.attempts, reason,
                )
                self.release("retry_success")
            else:
                rospy.loginfo(
                    "%s 首次成功（%s），piper_task 自行放行。",
                    self.active_task, reason,
                )
                self.clear()
            return

        if status == "rejected":
            # 上面已按 reason==active_command 处理过重发被拒；走到这里说明是
            # 别的动作被拒（例如人工在另一个终端发了命令），与本次盯守无关。
            rospy.logwarn("收到与当前盯守无关的拒绝：%s:%s", command, reason)
            return

        # status == failed
        base_reason = reason.split("=")[0]

        # 三个放置候选点都没有找到正确目标后，进入独立的放置回退恢复；
        # 只有正确目标复查和任意目标放置都失败后才执行最终丢弃。
        if (
            self.active_task == "piper_stop_7"
            and self.active_command == "place3"
            and base_reason == "target_not_found_at_all_place_points"
        ):
            self.begin_place_backtrack_recovery(reason)
            return

        # 只有“第三点没有安全抓取位姿”（完全未找到或目标仍在边缘）才进入
        # 倒车恢复。其他失败继续使用原有逻辑，不能触发任何负速度。
        if (
            self.active_task == "piper_stop_4"
            and self.active_command == "pick3"
            and base_reason in (
                BACKTRACK_PICK3_FAILURE,
                BACKTRACK_PICK3_EDGE_FAILURE,
            )
        ):
            self.begin_backtrack_recovery(base_reason, reason)
            return

        if base_reason not in self.retryable:
            rospy.logerr(
                "%s 失败且不可重试（%s）。%s",
                self.active_task, reason,
                "代发 done 放行。" if self.release_after_exhausted else "保持等待。",
            )
            if self.release_after_exhausted:
                self.release("non_retryable:%s" % base_reason)
            return

        if self.attempts >= self.max_attempts:
            rospy.logerr(
                "%s 已尝试 %d 次仍失败（%s）。%s",
                self.active_task, self.attempts, reason,
                "代发 done 放行，本环节 0 分但保住后续场景。"
                if self.release_after_exhausted else "保持等待。",
            )
            if self.release_after_exhausted:
                self.release("retry_exhausted:%s" % base_reason)
            return

        self.attempts += 1
        rospy.logwarn(
            "%s 第 %d 次失败（%s），原地重试第 %d/%d 次：%.1fs 后重发 %s",
            self.active_task, self.attempts - 1, reason,
            self.attempts, self.max_attempts, self.resend_delay,
            self.active_command,
        )
        self.publish_status(
            "retry:%s:%d/%d" % (self.active_task, self.attempts, self.max_attempts)
        )
        # 不立即重发：piper_task 在 run() 里先 publish_result 再把 busy 清掉
        # （piper_task_node.py:266 -> :237），马上重发有概率撞上 busy 被丢弃。
        self.schedule_command(self.active_command, "retry")

    def begin_place_backtrack_recovery(self, failure_reason):
        """第三放置点失败：倒回前两个点复查正确目标。"""
        self.recovery_active = True
        self.recovery_kind = "place"
        self.recovery_state = None
        self.recovery_target_cursor = 0
        self.recovery_targets = PLACE_BACKTRACK_TARGETS
        self.expected_recovery_command = None
        self.publish_status("place_backtrack:confirming_place_scan")
        rospy.logwarn(
            "进入 PLACE_BACKTRACK_RECOVERY。三个放置候选点均失败（%s）："
            "保持持物观察位，依次倒回第二、第一放置点复查。",
            failure_reason,
        )
        # 通过姿态确认结果与跟踪器完成授权握手，确认成功后才申请负速度。
        self.schedule_recovery_command(
            "prepare_place_scan",
            "confirm_place_scan_before_initial_backtrack",
            recovery_state="PLACE_WAIT_SCAN_FOR_REVERSE",
        )

    def begin_place_any_fallback(self, reason):
        """正确目标复查用尽或倒车失败：在当前位置尝试任意安全目标。"""
        self.publish_status("place_backtrack:fallback_any:%s" % reason)
        rospy.logwarn(
            "正确目标回退复查结束（%s）：在当前位置尝试任意安全识别物体。",
            reason,
        )
        self.schedule_recovery_command(
            "prepare_place_scan",
            "prepare_fallback_any",
            recovery_state="PLACE_WAIT_FALLBACK_SCAN",
        )

    def begin_place_recovery_discard(self, reason):
        """任意目标也不可用：执行最终丢弃并等待回零确认。"""
        self.place_discard_active = True
        self.place_discard_attempts = 1
        self.place_discard_hold = False
        self.publish_status(
            "place_discard:1/%d" % self.place_discard_max_attempts
        )
        rospy.logerr(
            "放置回退的视觉降级也失败（%s）：执行最终丢弃并确认回零。",
            reason,
        )
        self.schedule_recovery_command(
            "discard_place",
            "place_recovery_final_discard",
            recovery_state="PLACE_WAIT_DISCARD",
        )

    def begin_place_discard(self, failure_reason):
        self.place_discard_active = True
        self.place_discard_attempts = 1
        self.place_discard_hold = False
        self.publish_status("place_discard:1/%d" % self.place_discard_max_attempts)
        rospy.logerr(
            "%s 三个放置候选点均失败（%s）：张爪丢弃并回零，确认后放行。",
            self.active_task,
            failure_reason,
        )
        self.schedule_command("discard_place", "all_place_points_failed")

    def handle_place_discard_result(self, status, command, reason):
        """处理丢弃物品和回零结果；未确认回零时绝不主动放行。"""
        # piper_task 忙时结果格式为 rejected:busy:discard_place。
        if status == "rejected" and command == "busy" and reason == "discard_place":
            self.schedule_command("discard_place", "discard_rejected_busy")
            return

        if command != "discard_place":
            return

        self.last_activity = rospy.Time.now()
        if status == "success":
            rospy.loginfo("放置失败收尾完成（%s）：机械臂已回零，恢复跟踪。", reason)
            self.release("place_discard_complete")
            return

        if self.place_discard_attempts < self.place_discard_max_attempts:
            self.place_discard_attempts += 1
            rospy.logerr(
                "丢弃后回零失败（%s:%s），重试 %d/%d。",
                status,
                reason,
                self.place_discard_attempts,
                self.place_discard_max_attempts,
            )
            self.publish_status(
                "place_discard:%d/%d"
                % (self.place_discard_attempts, self.place_discard_max_attempts)
            )
            self.schedule_command("discard_place", "discard_retry")
            return

        self.place_discard_hold = True
        self.resend_at = None
        self.scheduled_command = None
        self.publish_status("place_discard:stow_failed_hold")
        rospy.logerr(
            "丢弃物品后仍无法确认机械臂回零（%s）；保持停车，不主动放行。",
            reason,
        )

    def begin_backtrack_recovery(self, failure_type, failure_detail):
        """第三点安全抓取失败：保持观察位，按失败类型选择倒车复查点。"""
        self.recovery_active = True
        self.recovery_kind = "pick"
        self.recovery_target_cursor = 0
        if failure_type == BACKTRACK_PICK3_EDGE_FAILURE:
            self.recovery_targets = BACKTRACK_EDGE_TARGETS
            edge_direction = failure_detail.partition("=")[2] or "unknown"
            recovery_text = (
                "第三点检测到正确目标但位于画面边缘（%s）：保持抓取观察位，"
                "短距离倒至恢复点复查。" % edge_direction
            )
        else:
            self.recovery_targets = BACKTRACK_TARGETS
            recovery_text = (
                "三个取货候选点均未找到目标：保持抓取观察位，"
                "倒回前两个候选点复查。"
            )
        self.publish_status("backtrack:confirming_pick_scan")
        rospy.logwarn("进入 BACKTRACK_RECOVERY。%s", recovery_text)
        # 不能在本回调里立刻发布 reverse：final_tracker 也在异步接收同一条
        # pick3 失败结果，过早申请倒车可能抢在其授权之前被拒。通过机械臂
        # 观察位确认结果完成握手，再申请倒车；机械臂全程不会回运输零位。
        self.schedule_recovery_command(
            "prepare_pick_scan",
            "confirm_scan_before_initial_backtrack",
            recovery_state="WAIT_SCAN_FOR_REVERSE",
        )

    def schedule_recovery_command(self, command, reason, recovery_state=None):
        self.expected_recovery_command = command
        if recovery_state is not None:
            self.recovery_state = recovery_state
        elif command == "stow":
            self.recovery_state = "WAIT_STOW"
        elif command == "prepare_pick_scan":
            self.recovery_state = "WAIT_SCAN"
        elif command in ("pick1", "pick2"):
            self.recovery_state = "WAIT_PICK"
        elif command == "abort_round":
            self.recovery_state = "WAIT_ABORT"
        self.schedule_command(command, reason)

    def current_recovery_target(self):
        if not 0 <= self.recovery_target_cursor < len(self.recovery_targets):
            return None
        return self.recovery_targets[self.recovery_target_cursor]

    @staticmethod
    def is_recovery_pick_miss(reason):
        """复查点没有安全抓取位姿：继续下一段倒车，而不是误判抓取成功。"""
        return (
            reason.startswith("not_here_continue_to_pick")
            or (
                reason.startswith("target_at_edge_")
                and "_continue_to_pick" in reason
            )
        )

    def request_current_backtrack(self):
        target = self.current_recovery_target()
        if target is None:
            if self.recovery_kind == "place":
                self.begin_place_any_fallback("no_recovery_target")
            else:
                self.begin_abort_round("no_recovery_target")
            return
        self.recovery_state = "WAIT_REVERSE"
        self.expected_recovery_command = None
        self.last_activity = rospy.Time.now()
        self.backtrack_command_pub.publish(String(data="reverse:%s" % target))
        self.publish_status("backtrack:reversing:%s" % target)
        posture = "放置持物观察位" if self.recovery_kind == "place" else "抓取观察位"
        rospy.loginfo("机械臂保持%s，申请车辆倒车至 %s。", posture, target)

    def handle_recovery_result(self, status, command, reason):
        """处理回退恢复期间由 /piper_task/command 触发的原子动作结果。"""
        if command == "waypoint":
            return

        expected = self.expected_recovery_command
        if status == "rejected" and reason == expected:
            self.schedule_command(expected, "recovery_busy")
            return
        if not expected or command != expected:
            return

        self.last_activity = rospy.Time.now()
        if self.recovery_kind == "place":
            self.handle_place_recovery_result(status, command, reason)
            return

        if status != "success":
            rospy.logerr(
                "回退恢复动作 %s 失败（%s:%s）。",
                command, status, reason,
            )
            if command == "abort_round":
                self.publish_status("backtrack:abort_failed:%s" % reason)
                # abort_round 不能确认收臂时保持停车，不能贸然恢复正常行驶。
                self.expected_recovery_command = None
                self.recovery_state = "ABORT_FAILED_HOLD"
            else:
                self.begin_abort_round("%s_failed:%s" % (command, reason))
            return

        if self.recovery_state == "WAIT_STOW":
            self.request_current_backtrack()
            return

        if self.recovery_state == "WAIT_SCAN_FOR_REVERSE":
            self.request_current_backtrack()
            return

        if self.recovery_state == "WAIT_SCAN":
            target = self.current_recovery_target()
            pick_command = BACKTRACK_PICK_COMMANDS.get(target)
            if not pick_command:
                self.begin_abort_round("missing_pick_command")
                return
            self.schedule_recovery_command(
                pick_command, "recheck_after_%s" % target
            )
            return

        if self.recovery_state == "WAIT_PICK":
            if self.is_recovery_pick_miss(reason):
                rospy.logwarn(
                    "%s 回退复查仍没有安全抓取位姿（%s）。",
                    self.current_recovery_target(),
                    reason,
                )
                self.recovery_target_cursor += 1
                if self.current_recovery_target() is None:
                    self.begin_abort_round("all_backtrack_points_missed")
                else:
                    # 复查未执行任何抓取运动，机械臂仍在观察位。下一段倒车前
                    # 再确认一次关节反馈，并借该结果与跟踪器完成状态握手。
                    self.schedule_recovery_command(
                        "prepare_pick_scan",
                        "confirm_scan_before_next_backtrack",
                        recovery_state="WAIT_SCAN_FOR_REVERSE",
                    )
                return

            rospy.loginfo(
                "%s 回退复查抓取成功（%s），结束恢复并继续前往卸货区。",
                self.current_recovery_target(), reason,
            )
            self.complete_backtrack("payload")
            return

        if self.recovery_state == "WAIT_ABORT":
            self.complete_backtrack("no_payload")

    @staticmethod
    def is_recovery_place_miss(reason):
        return reason.startswith("not_here_continue_to_place")

    def handle_place_recovery_result(self, status, command, reason):
        """处理放置倒车复查、任意目标放置和最终丢弃。"""
        state = self.recovery_state

        if state == "PLACE_WAIT_DISCARD":
            if status == "success":
                rospy.loginfo("最终丢弃及回零完成（%s），结束放置恢复。", reason)
                self.complete_backtrack("discarded")
                return
            if self.place_discard_attempts < self.place_discard_max_attempts:
                self.place_discard_attempts += 1
                self.publish_status(
                    "place_discard:%d/%d"
                    % (self.place_discard_attempts, self.place_discard_max_attempts)
                )
                rospy.logerr(
                    "最终丢弃/回零失败（%s:%s），重试 %d/%d。",
                    status,
                    reason,
                    self.place_discard_attempts,
                    self.place_discard_max_attempts,
                )
                self.schedule_recovery_command(
                    "discard_place",
                    "place_recovery_discard_retry",
                    recovery_state="PLACE_WAIT_DISCARD",
                )
            else:
                self.place_discard_hold = True
                self.recovery_state = "PLACE_DISCARD_FAILED_HOLD"
                self.expected_recovery_command = None
                self.publish_status("place_discard:stow_failed_hold")
                rospy.logerr("最终丢弃后仍无法确认回零；保持停车，不主动放行。")
            return

        if status != "success":
            self.begin_place_recovery_discard(
                "%s_failed:%s:%s" % (command, status, reason)
            )
            return

        if state == "PLACE_WAIT_SCAN_FOR_REVERSE":
            self.request_current_backtrack()
            return

        if state == "PLACE_WAIT_SCAN":
            target = self.current_recovery_target()
            place_command = PLACE_BACKTRACK_COMMANDS.get(target)
            if not place_command:
                self.begin_place_any_fallback("missing_place_command")
                return
            self.schedule_recovery_command(
                place_command,
                "recheck_exact_after_%s" % target,
                recovery_state="PLACE_WAIT_EXACT",
            )
            return

        if state == "PLACE_WAIT_EXACT":
            if self.is_recovery_place_miss(reason):
                rospy.logwarn(
                    "%s 回退复查仍未找到正确目标（%s）。",
                    self.current_recovery_target(),
                    reason,
                )
                self.recovery_target_cursor += 1
                if self.current_recovery_target() is None:
                    self.begin_place_any_fallback("all_exact_backtrack_points_missed")
                else:
                    self.schedule_recovery_command(
                        "prepare_place_scan",
                        "confirm_place_scan_before_next_backtrack",
                        recovery_state="PLACE_WAIT_SCAN_FOR_REVERSE",
                    )
                return

            rospy.loginfo(
                "%s 回退复查正确目标放置成功（%s）。",
                self.current_recovery_target(),
                reason,
            )
            self.complete_backtrack("placed")
            return

        if state == "PLACE_WAIT_FALLBACK_SCAN":
            self.schedule_recovery_command(
                "place_any",
                "try_any_safe_detected_object",
                recovery_state="PLACE_WAIT_ANY",
            )
            return

        if state == "PLACE_WAIT_ANY":
            rospy.logwarn("任意安全目标降级放置成功（%s）。", reason)
            self.complete_backtrack("placed")

    def backtrack_event_callback(self, msg):
        if not self.recovery_active or self.recovery_state != "WAIT_REVERSE":
            return
        fields = (msg.data or "").strip().split(":", 2)
        if len(fields) < 2:
            return
        status, target = fields[0], fields[1]
        reason = fields[2] if len(fields) > 2 else ""
        if target != self.current_recovery_target():
            return

        self.last_activity = rospy.Time.now()
        if status == "arrived":
            if self.recovery_kind == "place":
                rospy.loginfo(
                    "车辆已倒达 %s，机械臂保持持物观察姿态，停车后复查正确目标。",
                    target,
                )
                self.publish_status("place_backtrack:arrived:%s" % target)
                self.schedule_recovery_command(
                    "prepare_place_scan",
                    "prepare_place_scan_at_%s" % target,
                    recovery_state="PLACE_WAIT_SCAN",
                )
                return

            rospy.loginfo("车辆已倒达 %s，机械臂保持取货观察姿态，停车后复查。", target)
            self.publish_status("backtrack:arrived:%s" % target)
            self.schedule_recovery_command(
                "prepare_pick_scan", "prepare_scan_at_%s" % target
            )
            return

        if status == "failed":
            if self.recovery_kind == "place":
                rospy.logerr(
                    "倒车至 %s 失败（%s），在当前位置进入任意目标降级放置。",
                    target,
                    reason,
                )
                self.begin_place_any_fallback(
                    "reverse_failed:%s" % (reason or "unknown")
                )
                return
            rospy.logerr("倒车至 %s 失败（%s），中止本轮取货。", target, reason)
            self.begin_abort_round("reverse_failed:%s" % (reason or "unknown"))

    def begin_abort_round(self, reason):
        if not self.recovery_active:
            return
        rospy.logerr("本轮取货恢复终止（%s）：收臂并标记 no_payload。", reason)
        self.publish_status("backtrack:aborting:%s" % reason)
        self.schedule_recovery_command("abort_round", "abort_%s" % reason)

    def complete_backtrack(self, outcome):
        """让跟踪器退出专用倒车状态；不走普通 done，避免双重消费任务点。"""
        self.backtrack_command_pub.publish(String(data="complete:%s" % outcome))
        self.publish_status("backtrack:complete:%s" % outcome)
        self.clear()

    def handle_waypoint_result(self, status, payload):
        """处理 piper_task 对停靠事件的预处理结果。

        payload 形如 "<task_name>:<reason>"。

        success 只有 skip_without_arm 一种：候选点已被 navigation_skip 跳过，
        piper_task 自己发了 done。必须在这里撤销盯守，否则 arm_action_timeout
        到点后会补发一条过期的 done —— 虽然跟踪器按名字过滤不会误放行，
        但会刷出"机械臂可能异常"的假报警，掩盖真故障。

        failed 是 action_not_configured / invalid_action / arm_busy 三种预处理
        失败，piper_task 都不发 done，车会一直停着，需要在这里放行。
        """
        task, _, reason = payload.partition(":")
        task = task.strip()
        reason = reason.strip() or "unknown"
        if task != self.active_task:
            return

        if status == "success":
            rospy.loginfo("%s 由 piper_task 直接跳过（%s），撤销盯守。", task, reason)
            self.clear()
            return

        rospy.logerr(
            "%s 的停靠事件被 piper_task 拒绝（%s）。%s",
            task, reason,
            "代发 done 放行。" if self.release_after_exhausted else "保持等待。",
        )
        if self.release_after_exhausted:
            self.release("waypoint_rejected:%s" % reason)

    def schedule_command(self, command, reason):
        """延迟发送指定动作，避开 piper_task 发布结果后尚未清 busy 的窗口。"""
        if not command:
            return
        self.scheduled_command = command
        self.resend_at = rospy.Time.now() + rospy.Duration(self.resend_delay)
        self.last_activity = rospy.Time.now()
        rospy.logdebug(
            "%s 已排入命令队列：%s（%s）", self.active_task, command, reason
        )

    def release(self, reason):
        """代发 done，让跟踪器继续。"""
        task = self.active_task
        self.done_pub.publish(String(data="done:%s" % task))
        self.publish_status("released:%s:%s" % (task, reason))
        rospy.loginfo("已代发 done:%s（%s）", task, reason)
        self.released = True
        self.clear()

    def clear(self):
        self.active_task = None
        self.active_command = None
        self.attempts = 0
        self.last_activity = None
        self.resend_at = None
        self.scheduled_command = None
        self.recovery_active = False
        self.recovery_kind = None
        self.recovery_state = None
        self.recovery_target_cursor = 0
        self.recovery_targets = ()
        self.expected_recovery_command = None
        self.place_discard_active = False
        self.place_discard_attempts = 0
        self.place_discard_hold = False

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # 到时发送（避开 piper_task 的 busy 窗口）
            if (
                self.resend_at is not None
                and self.scheduled_command is not None
                and self.active_task is not None
                and not self.released
                and rospy.Time.now() >= self.resend_at
            ):
                command = self.scheduled_command
                self.resend_at = None
                self.scheduled_command = None
                rospy.loginfo(
                    "发送 %s（任务 %s，恢复状态 %s）",
                    command,
                    self.active_task,
                    self.recovery_state or "normal",
                )
                self.command_pub.publish(String(data=command))
                self.last_activity = rospy.Time.now()

            # 机械臂进程异常（崩溃/卡死）时不发任何结果，这里兜底放行，
            # 避免车一直停着直到跟踪器超时。
            if (
                self.active_task is not None
                and not self.released
                and self.last_activity is not None
                and self.action_timeout > 0.0
            ):
                idle = (rospy.Time.now() - self.last_activity).to_sec()
                if idle >= self.action_timeout:
                    rospy.logerr(
                        "%s 超过 %.1fs 无任何结果反馈，机械臂可能异常。",
                        self.active_task, self.action_timeout,
                    )
                    if self.place_discard_active and not self.recovery_active:
                        if self.place_discard_hold:
                            # 已明确进入安全保持状态，只刷新计时并持续停车。
                            self.last_activity = rospy.Time.now()
                        elif self.place_discard_attempts < self.place_discard_max_attempts:
                            self.place_discard_attempts += 1
                            self.publish_status(
                                "place_discard:%d/%d"
                                % (
                                    self.place_discard_attempts,
                                    self.place_discard_max_attempts,
                                )
                            )
                            self.schedule_command(
                                "discard_place", "discard_silent_retry"
                            )
                        else:
                            self.place_discard_hold = True
                            self.publish_status("place_discard:silent_hold")
                            self.last_activity = rospy.Time.now()
                            rospy.logerr(
                                "丢弃/回零命令连续无反馈；保持停车，不主动放行。"
                            )
                    elif self.recovery_active:
                        if self.recovery_kind == "place":
                            if self.recovery_state == "PLACE_WAIT_DISCARD":
                                if (
                                    self.place_discard_attempts
                                    < self.place_discard_max_attempts
                                ):
                                    self.place_discard_attempts += 1
                                    self.publish_status(
                                        "place_discard:%d/%d"
                                        % (
                                            self.place_discard_attempts,
                                            self.place_discard_max_attempts,
                                        )
                                    )
                                    self.schedule_recovery_command(
                                        "discard_place",
                                        "place_recovery_discard_silent_retry",
                                        recovery_state="PLACE_WAIT_DISCARD",
                                    )
                                else:
                                    self.place_discard_hold = True
                                    self.recovery_state = "PLACE_DISCARD_FAILED_HOLD"
                                    self.expected_recovery_command = None
                                    self.resend_at = None
                                    self.scheduled_command = None
                                    self.publish_status(
                                        "place_discard:silent_hold"
                                    )
                                    self.last_activity = rospy.Time.now()
                                    rospy.logerr(
                                        "放置最终丢弃/回零命令连续无反馈；"
                                        "保持停车，不主动放行。"
                                    )
                            elif self.recovery_state == "PLACE_DISCARD_FAILED_HOLD":
                                self.last_activity = rospy.Time.now()
                            else:
                                self.begin_place_recovery_discard(
                                    "place_recovery_timeout"
                                )
                        elif self.recovery_state == "WAIT_ABORT":
                            # 无法确认机械臂已经收回，车辆继续保持停车。
                            self.publish_status("backtrack:abort_timeout_hold")
                            self.recovery_state = "ABORT_FAILED_HOLD"
                            self.expected_recovery_command = None
                            self.resend_at = None
                            self.scheduled_command = None
                            self.last_activity = rospy.Time.now()
                        elif self.recovery_state == "ABORT_FAILED_HOLD":
                            self.last_activity = rospy.Time.now()
                        else:
                            self.begin_abort_round("recovery_timeout")
                    elif self.release_after_exhausted:
                        self.release("arm_silent_timeout")
                    else:
                        self.last_activity = rospy.Time.now()
            rate.sleep()


if __name__ == "__main__":
    try:
        ArmBridge().spin()
    except rospy.ROSInterruptException:
        pass
