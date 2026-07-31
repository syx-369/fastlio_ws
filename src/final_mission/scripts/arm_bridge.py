#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂任务桥：原地重试 + 失败放行。

piper_task 自己订阅 /waypoint_task_event，车一停就直接执行 card/pick/place，
本节点不抢这个职责，只做 piper_task 缺失的两件事：

1. 原地重试。piper_task 已有"换点重试"（stop_2 没找到 -> 发 done 让车开去
   stop_3 -> stop_4），但三个候选点都失败后直接返回 False，不再尝试。
   规则允许每轮最多 3 次装货机会（第 2 次成功 90 分、第 3 次 60 分），
   所以这里在最后一个候选点原地重发命令，把机会用足。

2. 失败放行。piper_task 只在 ok=True 时发 done（piper_task_node.py:229），
   失败路径不发，车会一直卡到跟踪器的 external_task_timeout 才走。
   七点位两轮共 14 个卡点，30 分钟总时长扛不住几次。这里在重试用尽后
   代发 done，本轮取货/卸货得 0 分，但保住避障、红绿灯、终点停车的分数。

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

        # 当前正在盯的停靠点
        self.active_task = None      # piper_stop_N
        self.active_command = None   # card / pick1 / place2 ...
        self.attempts = 0            # 已尝试次数（含首次）
        self.max_attempts = 1
        self.last_activity = None
        self.released = False
        self.resend_at = None        # 非 None 表示有一条待重发的命令

        self.done_pub = rospy.Publisher(self.done_topic, String, queue_size=10)
        self.command_pub = rospy.Publisher(
            self.arm_command_topic, String, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )

        rospy.Subscriber(
            self.event_topic, String, self.event_callback, queue_size=20
        )
        rospy.Subscriber(
            self.arm_result_topic, String, self.result_callback, queue_size=20
        )

        rospy.loginfo("=" * 60)
        rospy.loginfo("ArmBridge started (原地重试 + 失败放行)")
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
            self.schedule_resend("rejected_%s" % command)
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
        self.schedule_resend("retry")

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

    def schedule_resend(self, reason):
        """延迟重发当前动作，避开 piper_task 的 busy 窗口。"""
        self.resend_at = rospy.Time.now() + rospy.Duration(self.resend_delay)
        self.last_activity = rospy.Time.now()
        rospy.logdebug("%s 已排入重发队列（%s）", self.active_task, reason)

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

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # 到时重发（避开 piper_task 的 busy 窗口）
            if (
                self.resend_at is not None
                and self.active_task is not None
                and not self.released
                and rospy.Time.now() >= self.resend_at
            ):
                self.resend_at = None
                rospy.loginfo("重发 %s（%s 第 %d/%d 次）",
                              self.active_command, self.active_task,
                              self.attempts, self.max_attempts)
                self.command_pub.publish(String(data=self.active_command))
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
                    if self.release_after_exhausted:
                        self.release("arm_silent_timeout")
                    else:
                        self.last_activity = rospy.Time.now()
            rate.sleep()


if __name__ == "__main__":
    try:
        ArmBridge().spin()
    except rospy.ROSInterruptException:
        pass
