#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""决赛总控。

职责刻意收窄 —— 只管"什么时候允许开始"和"红绿灯放行"，不碰底层运动，
不碰机械臂动作：

  起步       等红旗（或手动/自动起步），起步前压住跟踪器
  红绿灯     ext:traffic_light 到点后切视觉到 light 模式，等绿灯再放行
  终点       ext:finish 到点后标记完成

不做的事：
  * 路径跟踪与避障      -> final_tracker
  * 机械臂动作          -> piper_task（自己订阅 /waypoint_task_event）
  * 机械臂重试与放行    -> arm_bridge

与原 race_mission_manager 的区别：不再用 subprocess 拉起跟踪器（那样进程
生命周期和参数传递都很脆），跟踪器由 launch 统一管理，这里只用一个
enable 话题控制它起停。
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class FinalManager:
    def __init__(self):
        rospy.init_node("final_manager", anonymous=False)

        self.wait_for_start = bool(rospy.get_param("~wait_for_start", True))
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.cmd_topic = rospy.get_param("~cmd_topic", "/smoother_cmd_vel")

        self.task_event_topic = rospy.get_param(
            "~task_event_topic", "/waypoint_task_event"
        )
        self.task_done_topic = rospy.get_param(
            "~task_done_topic", "/waypoint_task_done"
        )
        self.traffic_light_tasks = self.param_list(
            "~traffic_light_tasks", ["traffic_light"]
        )
        self.green_states = self.param_list("~green_states", ["green"])
        self.traffic_timeout = float(rospy.get_param("~traffic_timeout", 120.0))
        self.traffic_timeout_policy = str(
            rospy.get_param("~traffic_timeout_policy", "pass")
        ).strip().lower()
        self.finish_task_name = rospy.get_param("~finish_task_name", "finish")
        self.finish_done_delay = float(rospy.get_param("~finish_done_delay", 3.0))

        # ===== 改动 2026-07-30：图像通路确认 =====
        # 「图像根本没通」和「一直没看到红旗」在现象上完全一样 —— 车停着不动。
        # 赛场上没法快速区分，所以让视觉节点上报通路状态，这里定期检查：
        # 等红旗超过 image_warn_after 秒还没收到任何一帧，就大声报警并提示
        # 手动起步，把静默失败变成响亮失败。只告警，不阻止起步。
        self.image_warn_after = float(rospy.get_param("~image_warn_after", 10.0))
        self.vision_status = None
        self.wait_flag_since = None
        self.image_warned = False
        # ===== 改动结束 =====

        self.state = "BOOT"
        self.start_received = self.auto_start
        self.traffic_light_state = "none"
        self.pending_light_task = None
        self.light_wait_started = None
        self.finish_seen_at = None

        self.state_pub = rospy.Publisher(
            "/final_mission/state", String, queue_size=1, latch=True
        )
        self.tracker_enable_pub = rospy.Publisher(
            "/final_mission/tracker_enable", Bool, queue_size=1, latch=True
        )
        self.vision_control_pub = rospy.Publisher(
            "/final_mission/vision_control", String, queue_size=5, latch=True
        )
        self.done_pub = rospy.Publisher(
            self.task_done_topic, String, queue_size=10
        )
        self.stop_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)

        rospy.Subscriber(
            "/final_mission/start_signal", Bool,
            self.start_callback, queue_size=5,
        )
        rospy.Subscriber(
            "/final_mission/traffic_light", String,
            self.light_callback, queue_size=10,
        )
        rospy.Subscriber(
            "/final_mission/command", String,
            self.command_callback, queue_size=5,
        )
        # 改动 2026-07-30：订阅视觉通路状态（见 image_warn_after）
        rospy.Subscriber(
            "/final_mission/vision_status", String,
            self.vision_status_callback, queue_size=5,
        )
        rospy.Subscriber(
            self.task_event_topic, String, self.event_callback, queue_size=20
        )

        rospy.on_shutdown(self.on_shutdown)

        if self.wait_for_start and not self.auto_start:
            self.set_state("WAIT_FLAG")
            # 改动 2026-07-30：记录开始等红旗的时间，用于图像通路超时告警
            self.wait_flag_since = rospy.Time.now()
            self.tracker_enable_pub.publish(Bool(data=False))
            self.vision_control_pub.publish(String(data="flag"))
            rospy.loginfo("等待裁判挥红旗。手动起步：")
            rospy.loginfo("  rostopic pub -1 /final_mission/command "
                          "std_msgs/String \"data: 'start'\"")
        else:
            self.set_state("RUN")
            self.tracker_enable_pub.publish(Bool(data=True))
            self.vision_control_pub.publish(String(data="idle"))

    @staticmethod
    def param_list(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return list(value)

    def set_state(self, state):
        if self.state != state:
            rospy.loginfo("总控状态：%s -> %s", self.state, state)
        self.state = state
        self.state_pub.publish(String(data=state))

    def stop_robot(self):
        zero = Twist()
        for _ in range(3):
            self.stop_pub.publish(zero)
            rospy.sleep(0.02)

    # ------------------------------------------------------------------
    def start_callback(self, msg):
        if msg.data and not self.start_received:
            self.start_received = True
            rospy.loginfo("收到起步信号。")

    def light_callback(self, msg):
        self.traffic_light_state = (msg.data or "").strip().lower()

    # 改动 2026-07-30 新增
    def vision_status_callback(self, msg):
        """视觉通路状态：ok:frames=N:age=X / no_image:frames=0 / standalone:..."""
        self.vision_status = (msg.data or "").strip().lower()

    def image_path_ok(self):
        """图像通路是否确认可用。None 表示还没收到状态上报。"""
        if self.vision_status is None:
            return None
        return not self.vision_status.startswith("no_image")

    def check_image_path(self):
        """等红旗时长时间没图像就报警。只告警，不阻止起步。"""
        if self.image_warned or self.image_warn_after <= 0.0:
            return
        if self.wait_flag_since is None:
            return
        waited = (rospy.Time.now() - self.wait_flag_since).to_sec()
        if waited < self.image_warn_after:
            return
        if self.image_path_ok():
            return

        self.image_warned = True
        detail = self.vision_status or "（视觉节点未上报状态）"
        rospy.logerr("=" * 60)
        rospy.logerr(
            "已等待 %.0fs 但图像通路没通：%s", waited, detail
        )
        rospy.logerr(
            "红旗识别不可能成功。最常见原因：piper_task 未接入相机中继补丁，"
            "或启动时没有 enable_camera_relay:=true。"
        )
        rospy.logerr("请改用手动起步：")
        rospy.logerr("  rostopic pub -1 /final_mission/command "
                     "std_msgs/String \"data: 'start'\"")
        rospy.logerr("=" * 60)

    def command_callback(self, msg):
        command = (msg.data or "").strip().lower()
        if command == "start":
            self.start_received = True
            rospy.loginfo("手动起步指令。")
        elif command == "stop":
            rospy.logwarn("手动停止指令。")
            self.tracker_enable_pub.publish(Bool(data=False))
            self.stop_robot()
            self.set_state("STOPPED")
        elif command == "resume":
            rospy.loginfo("手动恢复指令。")
            self.tracker_enable_pub.publish(Bool(data=True))
            self.set_state("RUN")
        elif command == "finish":
            rospy.logwarn("手动结束指令。")
            self.finish_seen_at = rospy.Time.now()
            self.set_state("FINISHING")
        elif command:
            rospy.logwarn("未知总控指令：%s", command)

    def event_callback(self, msg):
        fields = (msg.data or "").strip().split(":", 2)
        if len(fields) < 2 or fields[0] != "start":
            return
        task = fields[1].strip()

        if task in self.traffic_light_tasks:
            self.begin_light_wait(task)
        elif task == self.finish_task_name:
            rospy.loginfo("到达终点任务点。")
            self.done_pub.publish(String(data="done:%s" % task))
            self.finish_seen_at = rospy.Time.now()
            self.set_state("FINISHING")

    # ------------------------------------------------------------------
    def begin_light_wait(self, task):
        # 机械臂零位（TRANSPORT_JOINTS 全 0）下相机前视，看灯无需动臂。
        if self.traffic_light_state in self.green_states:
            rospy.loginfo("红绿灯已是绿灯，直接放行。")
            self.release_light(task, "already_green")
            return
        self.pending_light_task = task
        self.light_wait_started = rospy.Time.now()
        self.vision_control_pub.publish(String(data="light"))
        self.set_state("WAIT_LIGHT")
        rospy.loginfo("等待绿灯，当前状态：%s", self.traffic_light_state)

    def release_light(self, task, reason):
        self.done_pub.publish(String(data="done:%s" % task))
        self.vision_control_pub.publish(String(data="idle"))
        self.pending_light_task = None
        self.light_wait_started = None
        rospy.loginfo("红绿灯放行（%s）。", reason)
        if self.state == "WAIT_LIGHT":
            self.set_state("RUN")

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == "WAIT_FLAG":
                # 改动 2026-07-30：图像通路没通就大声报警（不阻止起步）
                self.check_image_path()
                if self.start_received:
                    rospy.loginfo("起步：使能跟踪器。")
                    self.vision_control_pub.publish(String(data="idle"))
                    self.tracker_enable_pub.publish(Bool(data=True))
                    self.set_state("RUN")

            elif self.state == "WAIT_LIGHT":
                if self.traffic_light_state in self.green_states:
                    self.release_light(self.pending_light_task, "green")
                elif (
                    self.traffic_timeout > 0.0
                    and self.light_wait_started is not None
                ):
                    waited = (rospy.Time.now() - self.light_wait_started).to_sec()
                    if waited >= self.traffic_timeout:
                        if self.traffic_timeout_policy == "pass":
                            rospy.logwarn(
                                "红绿灯等待超时 %.0fs，按 pass 策略放行。",
                                self.traffic_timeout,
                            )
                            self.release_light(
                                self.pending_light_task, "timeout_pass"
                            )
                        else:
                            rospy.logerr("红绿灯等待超时，按 hold 策略停车。")
                            self.tracker_enable_pub.publish(Bool(data=False))
                            self.stop_robot()
                            self.set_state("ERROR")

            elif self.state == "FINISHING":
                if self.finish_seen_at is not None:
                    elapsed = (rospy.Time.now() - self.finish_seen_at).to_sec()
                    if elapsed >= self.finish_done_delay:
                        self.tracker_enable_pub.publish(Bool(data=False))
                        self.stop_robot()
                        self.set_state("DONE")

            elif self.state in ("DONE", "ERROR", "STOPPED"):
                self.stop_robot()

            rate.sleep()

    def on_shutdown(self):
        self.tracker_enable_pub.publish(Bool(data=False))
        self.stop_robot()


if __name__ == "__main__":
    try:
        FinalManager().spin()
    except rospy.ROSInterruptException:
        pass
