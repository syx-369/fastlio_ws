#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机帧转发：给 piper_task 用的一个 mixin，不是独立节点。

为什么需要它
------------
全车只有一台 D435i，装在机械臂上。pyrealsense2 的设备是独占的，而
piper_task 在构造时就打开相机并全程持有：

    vision_grasp_core.py:342    profile = self.pipeline.start(config)
    vision_grasp_core.py:950    self.pipeline.stop()      # 只在关节点关闭

所以 final_vision 无法再打开同一台相机 —— 若按原 race_mission 的做法自己
rs.pipeline().start()，第二个进程必然失败，红旗识别起不来，车不会起步。

解法：让 piper_task 成为唯一相机持有者，空闲时把帧发布成 ROS 话题，
final_vision 改为订阅。这样 vision_grasp_core.py 保持与已实测的
grab2016_7_21.py 逐字节一致，改动只落在 piper_task_node.py 的包装类里。

为什么安全
----------
机械臂只在两处取帧，且都在 execute_command 的 busy=True 区间内：

    vision_grasp_core.py:443    recognize_reference_target()
    vision_grasp_core.py:609    get_object_pose()

因此用 busy 标志让发布线程避让即可天然互斥，不会并发调用
wait_for_frames（pyrealsense2 不允许并发调用）。

为什么是「按需」而不是常开（改动 2026-07-30）
--------------------------------------------
原来是常开 10Hz。改成按需：只有 final_vision 明确请求时才发帧。

  * 相机所有权不变（永久归 piper_task）—— 分时移交方案要 pipeline.start()
    重开设备，1~3 秒且可能失败，一旦要不回来 14 个机械臂卡点全废，
    这个风险换不到任何好处。
  * 带宽：640x480x3 = 0.92MB/帧。红旗+红绿灯合计最多约 1 分钟，
    30 分钟赛程占空比约 3%，其余时间完全不取帧。
  * 与机械臂争用的机会也同比下降。

请求协议（话题 ~relay_request_topic，默认 /final_mission/camera/request）：

    "start"  开始发帧。需周期性重发当心跳。
    "stop"   停止发帧。

心跳超时（~relay_keepalive_timeout，默认 3s）内收不到新的 start 就自动停发。
这样 final_vision 崩了不会留下一个一直发帧的中继。

如何接入
--------
在 piper_task_node.py 里做三处改动（都用 `改动 2026-07-30` 注释标出）：

    1. 顶部导入：
       import os, sys, rospkg
       sys.path.insert(0, os.path.join(
           rospkg.RosPack().get_path("final_mission"), "scripts"))
       from camera_relay import CameraRelayMixin

    2. 让 CompetitionVisionController 混入本类，并在相机初始化之后调用
       self.start_frame_relay()：

       class CompetitionVisionController(CameraRelayMixin, PiperVisionController):
           def __init__(self):
               super().__init__(...)
               self.start_frame_relay()

    3. CompetitionTaskNode.run() 里把 busy 状态同步给 relay：
       在 self.busy = True 之后加 self.arm.set_relay_paused(True)
       在 finally 的 self.busy = False 之后加 self.arm.set_relay_paused(False)

默认 enable_camera_relay=false，即不接入时 piper_task 行为与原来完全一致。
"""

import threading

import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraRelayMixin:
    """把机械臂相机的彩色帧按需转发成 ROS 话题，供红旗/红绿灯识别使用。"""

    def start_frame_relay(self):
        self.relay_topic = rospy.get_param(
            "~relay_topic", "/final_mission/camera/color"
        )
        self.relay_rate = float(rospy.get_param("~relay_rate", 10.0))
        # 改动 2026-07-30：默认关闭。不显式打开时 piper_task 行为与原来一致，
        # 便于单独调机械臂，也避免误引入一个后台取帧线程。
        self.relay_enabled = bool(
            rospy.get_param("~enable_camera_relay", False)
        )
        # 改动 2026-07-30：按需发帧的请求话题与心跳超时
        self.relay_request_topic = rospy.get_param(
            "~relay_request_topic", "/final_mission/camera/request"
        )
        self.relay_keepalive_timeout = float(
            rospy.get_param("~relay_keepalive_timeout", 3.0)
        )

        self._relay_paused = False
        self._relay_wanted = False      # 改动 2026-07-30：是否有人请求发帧
        self._relay_last_request = None  # 改动 2026-07-30：最后一次心跳时间
        self._relay_lock = threading.Lock()
        self._relay_pub = rospy.Publisher(self.relay_topic, Image, queue_size=1)

        if not self.relay_enabled:
            rospy.loginfo(
                "相机帧转发未启用（enable_camera_relay=false），"
                "piper_task 行为与原来一致。"
            )
            return

        # 改动 2026-07-30：订阅请求话题
        rospy.Subscriber(
            self.relay_request_topic, String,
            self._relay_request_callback, queue_size=5,
        )

        self._relay_thread = threading.Thread(
            target=self._relay_loop, name="camera_relay", daemon=True
        )
        self._relay_thread.start()
        rospy.loginfo(
            "相机帧转发已就绪（按需）：%s @ %.0fHz，请求话题 %s，"
            "心跳超时 %.1fs（机械臂动作期间自动避让）",
            self.relay_topic, self.relay_rate,
            self.relay_request_topic, self.relay_keepalive_timeout,
        )

    # 改动 2026-07-30 新增
    def _relay_request_callback(self, msg):
        command = (msg.data or "").strip().lower()
        if command in ("start", "on", "1", "true"):
            with self._relay_lock:
                was = self._relay_wanted
                self._relay_wanted = True
                self._relay_last_request = rospy.Time.now()
            if not was:
                rospy.loginfo("收到发帧请求，开始转发相机帧。")
        elif command in ("stop", "off", "0", "false"):
            with self._relay_lock:
                was = self._relay_wanted
                self._relay_wanted = False
                self._relay_last_request = None
            if was:
                rospy.loginfo("收到停止请求，停止转发相机帧。")
        else:
            rospy.logwarn("未知发帧请求：%s", command)

    def set_relay_paused(self, paused):
        """机械臂开始/结束动作时调用，避免与 wait_for_frames 并发。"""
        with self._relay_lock:
            self._relay_paused = bool(paused)

    def _relay_paused_now(self):
        with self._relay_lock:
            return self._relay_paused

    # 改动 2026-07-30 新增：综合判断这一轮要不要取帧
    def _relay_should_publish(self):
        with self._relay_lock:
            if self._relay_paused or not self._relay_wanted:
                return False
            last = self._relay_last_request
        if self.relay_keepalive_timeout > 0.0 and last is not None:
            if (rospy.Time.now() - last).to_sec() > self.relay_keepalive_timeout:
                # 心跳断了（请求方可能已崩），自动停发
                with self._relay_lock:
                    if self._relay_wanted:
                        self._relay_wanted = False
                        self._relay_last_request = None
                rospy.logwarn(
                    "发帧请求心跳超过 %.1fs 未更新，自动停止转发。",
                    self.relay_keepalive_timeout,
                )
                return False
        return True

    def _relay_loop(self):
        rate = rospy.Rate(max(self.relay_rate, 1.0))
        while not rospy.is_shutdown():
            # 改动 2026-07-30：原来只看 paused + 订阅者数量，
            # 现在还要看是否有人明确请求（按需）。
            if not self._relay_should_publish():
                rate.sleep()
                continue
            if self._relay_pub.get_num_connections() == 0:
                # 没人订阅时不取帧，省 CPU 也少一层与机械臂争用的机会。
                rate.sleep()
                continue
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                color = frames.get_color_frame()
                if not color:
                    rate.sleep()
                    continue
                image = np.asanyarray(color.get_data())
                self._relay_pub.publish(self._to_image_msg(image))
            except RuntimeError:
                # 机械臂正在取帧或相机短暂无数据，跳过这一轮。
                pass
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "相机帧转发异常：%s", exc)
            rate.sleep()

    @staticmethod
    def _to_image_msg(image):
        message = Image()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "hand_cam"
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = image.shape[1] * 3
        message.data = image.tobytes()
        return message
