#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""起步红旗与红绿灯识别。

检测算法沿用 race_mission/scripts/start_light_vision_node.py（HSV 红旗 +
YOLO/HSV 红绿灯），只改图像来源。

图像来源两种模式（~vision_source）：

  arm_topic  —— 订阅 piper_task 发布的图像话题。全车只有一台 D435i 且装在
                机械臂上，pyrealsense2 的设备是独占的：piper_task 在构造时
                就 pipeline.start() 并全程持有（vision_grasp_core.py:342），
                本节点无法再打开同一台相机。因此比赛时必须走这个模式，
                由 piper_task 侧发布图像（见 camera_relay.py 与改动说明）。

  standalone —— 本节点自己开相机。仅用于不启动机械臂的路线联调（dry_run），
                此时没有进程占用相机。

模式切换由 /final_mission/vision_control 控制：flag / light / idle。
idle 时不做推理，standalone 模式下还会释放相机。
"""

import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# ===== 改动 2026-07-30（修订）：ultralytics 改为惰性导入 =====
# 演进过程，避免以后又改回去：
#   最初  try: from ultralytics import YOLO / except: YOLO = None
#         -> 同一名字先是类后是 None，Pylance 报 obscured declaration。
#   上一版 改成模块级 try + _HAS_YOLO 布尔量
#         -> 反而引入新问题：except 分支里 YOLO 这个名字根本没绑定，
#            下面 YOLO(self.weights) 被判 possiblyUnbound。
#   现在  完全不在模块级导入，改到 load_yolo() 里就地 import。
#         名字只在确实导入成功的作用域内存在，两种告警都没有。
#         附带好处：不用 YOLO（HSV 兜底）时省掉 ultralytics 1~2s 的导入耗时。
# ===== 改动结束 =====


class FinalVision:
    def __init__(self):
        rospy.init_node("final_vision", anonymous=False)

        self.source = str(
            rospy.get_param("~vision_source", "arm_topic")
        ).strip().lower()
        self.image_topic = rospy.get_param(
            "~camera_image_topic", "/final_mission/camera/color"
        )
        self.arm_detection_image_topic = rospy.get_param(
            "~arm_detection_image_topic", "/piper_task/detection_image"
        )
        self.arm_detection_status_topic = rospy.get_param(
            "~arm_detection_status_topic", "/piper_task/detection_status"
        )
        self.arm_detection_hold_time = max(
            1.0, float(rospy.get_param("~arm_detection_hold_time", 12.0))
        )
        self.weights = rospy.get_param(
            "~traffic_weights",
            "/home/user/fastlio_ws/src/waypoint_tools/config/traffic_light.pt",
        )
        self.conf = float(rospy.get_param("~conf", 0.4))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = str(rospy.get_param("~device", "0"))
        self.show_image = bool(rospy.get_param("~show_image", True))
        self.use_yolo = bool(rospy.get_param("~use_yolo", True))

        self.flag_confirm_threshold = int(rospy.get_param("~flag_confirm_frames", 5))
        self.flag_min_area = int(rospy.get_param("~flag_min_area", 3000))
        self.min_saturation = int(rospy.get_param("~min_saturation", 70))
        self.min_value = int(rospy.get_param("~min_value", 90))

        self.mode = str(rospy.get_param("~initial_mode", "flag")).strip().lower()
        self.flag_confirm_count = 0
        self.model = None
        self.names = {}
        self.pipeline = None
        self.latest_frame = None
        self.latest_arm_detection_frame = None
        self.last_arm_detection_at = None
        self.latest_arm_detection_status = ""

        # ===== 改动 2026-07-30：按需中继 =====
        # 相机永久归 piper_task（设备独占，交出去要不回来的风险太大），
        # 但只在本节点真正要识别时才让它发帧，不做全程 10Hz 常发。
        # 红旗最多几十秒、红绿灯最多几十秒，30 分钟赛程里占空比约 3%。
        self.relay_request_topic = rospy.get_param(
            "~relay_request_topic", "/final_mission/camera/request"
        )
        # 请求发帧的心跳周期。piper_task 侧收不到心跳会自动停发，
        # 这样本节点崩了也不会让它一直发。
        self.relay_keepalive_period = float(
            rospy.get_param("~relay_keepalive_period", 1.0)
        )
        self.relay_requested = False
        self.last_relay_request_at = None

        # 供总控确认「图像通路是否真的通了」。静默失败最难查，
        # 所以这里统计帧数，总控放行前会看这个话题。
        self.frames_seen = 0
        self.last_frame_at = None
        # ===== 改动结束 =====

        # ===== 改动 2026-07-30：show_image 无显示器时不崩 =====
        # 你要求保留 show_image=true。但没有 DISPLAY 时 cv2.imshow 会抛
        # cv2.error，原来会把 spin() 打断。这里探测一次，不可用就自动降级为
        # 不显示并告警，识别逻辑照常跑。
        self.display_ok = self.check_display()
        # ===== 改动结束 =====

        # 红色在 OpenCV HSV 中跨越 0/180，需要两个色相区间。
        self.lower_red1 = np.array([0, 120, 80])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 80])
        self.upper_red2 = np.array([180, 255, 255])

        self.start_pub = rospy.Publisher(
            "/final_mission/start_signal", Bool, queue_size=1, latch=True
        )
        self.light_pub = rospy.Publisher(
            "/final_mission/traffic_light", String, queue_size=1
        )
        # ===== 改动 2026-07-30：按需中继 + 通路状态 =====
        # relay_pub: 向 piper_task 请求/停止发帧（start / stop）
        # status_pub: 发布图像通路状态，供总控起步前确认（latch）
        self.relay_pub = rospy.Publisher(
            self.relay_request_topic, String, queue_size=5
        )
        self.status_pub = rospy.Publisher(
            "/final_mission/vision_status", String, queue_size=1, latch=True
        )
        # ===== 改动结束 =====

        rospy.Subscriber(
            "/final_mission/vision_control", String,
            self.control_callback, queue_size=5,
        )
        if self.source == "arm_topic":
            rospy.Subscriber(
                self.image_topic, Image, self.image_callback, queue_size=1
            )
            rospy.Subscriber(
                self.arm_detection_image_topic,
                Image,
                self.arm_detection_image_callback,
                queue_size=1,
            )
            rospy.Subscriber(
                self.arm_detection_status_topic,
                String,
                self.arm_detection_status_callback,
                queue_size=20,
            )

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("=" * 60)
        rospy.loginfo("FinalVision started")
        rospy.loginfo("图像来源       : %s", self.source)
        if self.source == "arm_topic":
            rospy.loginfo("订阅图像话题   : %s", self.image_topic)
            rospy.loginfo("机械臂检测图像 : %s", self.arm_detection_image_topic)
            rospy.loginfo("机械臂检测状态 : %s", self.arm_detection_status_topic)
        rospy.loginfo("初始模式       : %s", self.mode)
        rospy.loginfo("红绿灯权重     : %s", self.weights)
        rospy.loginfo("=" * 60)

    # ------------------------------------------------------------------
    # 图像来源
    # ------------------------------------------------------------------
    @staticmethod
    def decode_image(msg):
        """手工解码 sensor_msgs/Image，避免依赖 cv_bridge 的 ABI 兼容问题。"""
        if msg.encoding not in ("bgr8", "rgb8"):
            rospy.logwarn_throttle(
                5.0, "不支持的图像编码 %s，仅支持 bgr8/rgb8。", msg.encoding
            )
            return None
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            frame = frame.reshape(msg.height, msg.width, 3)
        except ValueError:
            rospy.logwarn_throttle(5.0, "图像尺寸与数据长度不匹配，丢弃该帧。")
            return None
        if msg.encoding == "rgb8":
            frame = frame[:, :, ::-1]
        return frame.copy()

    def image_callback(self, msg):
        frame = self.decode_image(msg)
        if frame is None:
            return
        self.latest_frame = frame
        # 改动 2026-07-30：记帧用于通路确认（见 publish_status）
        self.frames_seen += 1
        self.last_frame_at = rospy.Time.now()

    def arm_detection_image_callback(self, msg):
        frame = self.decode_image(msg)
        if frame is None:
            return
        self.latest_arm_detection_frame = frame
        self.last_arm_detection_at = rospy.Time.now()

    def arm_detection_status_callback(self, msg):
        status = (msg.data or "").strip()
        if not status:
            return
        self.latest_arm_detection_status = status
        rospy.loginfo("[机械臂视觉] %s", status)

    # ===== 改动 2026-07-30 新增：按需中继控制 =====
    def request_relay(self, want):
        """让 piper_task 开始/停止发帧。

        幂等：重复发同样的请求无副作用（piper_task 侧按状态去重）。
        start 需要周期性重发当心跳，piper_task 收不到心跳会自动停发——
        这样本节点异常退出时不会留下一个一直发帧的中继。
        """
        if self.source != "arm_topic":
            return  # standalone 模式自己开相机，不需要中继

        now = rospy.Time.now()
        if want:
            due = (
                self.last_relay_request_at is None
                or (now - self.last_relay_request_at).to_sec()
                >= self.relay_keepalive_period
            )
            if not due:
                return
            self.relay_pub.publish(String(data="start"))
            self.last_relay_request_at = now
            if not self.relay_requested:
                rospy.loginfo("请求 piper_task 开始发帧（按需中继）。")
            self.relay_requested = True
        else:
            if not self.relay_requested:
                return
            self.relay_pub.publish(String(data="stop"))
            self.relay_requested = False
            self.last_relay_request_at = None
            rospy.loginfo("已请求 piper_task 停止发帧。")

    def check_display(self):
        """探测 cv2.imshow 是否可用。不可用不算错误，只降级。"""
        if not self.show_image:
            return False
        if not os.environ.get("DISPLAY"):
            rospy.logwarn(
                "show_image=true 但环境变量 DISPLAY 为空（没接显示器或 ssh 未开 X11 转发），"
                "自动降级为不显示图像。识别逻辑不受影响。"
            )
            return False
        try:
            probe = np.zeros((16, 16, 3), dtype=np.uint8)
            cv2.imshow("Final Mission Vision", probe)
            cv2.waitKey(1)
            return True
        except cv2.error as exc:
            rospy.logwarn(
                "show_image=true 但 OpenCV 窗口不可用（%s），自动降级为不显示。", exc
            )
            return False

    def publish_status(self):
        """发布图像通路状态，供总控起步前确认。

        没有这个的话，「图像根本没通」和「一直没看到红旗」在现象上一模一样，
        都是车停着不动，赛场上没法快速判断。
        """
        if self.source != "arm_topic":
            self.status_pub.publish(String(data="standalone:frames=%d" % self.frames_seen))
            return
        if self.frames_seen == 0:
            self.status_pub.publish(String(data="no_image:frames=0"))
            return
        age = (
            (rospy.Time.now() - self.last_frame_at).to_sec()
            if self.last_frame_at is not None else -1.0
        )
        self.status_pub.publish(
            String(data="ok:frames=%d:age=%.1f" % (self.frames_seen, age))
        )
    # ===== 改动结束 =====

    def start_camera(self):
        if self.pipeline is not None:
            return True
        try:
            import pyrealsense2 as rs

            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            rospy.loginfo("standalone 模式：相机已打开。")
            return True
        except Exception as exc:
            self.pipeline = None
            rospy.logwarn_throttle(
                2.0,
                "打开 RealSense 失败：%s。若机械臂正在运行，它已独占相机，"
                "请改用 vision_source:=arm_topic。", exc,
            )
            return False

    def stop_camera(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
            rospy.loginfo("standalone 模式：相机已释放。")

    def grab_frame(self):
        if self.source == "arm_topic":
            return self.latest_frame
        if not self.start_camera():
            return None
        # 改动 2026-07-30：先取到局部变量。start_camera() 成功即保证
        # self.pipeline 非 None，但类型检查器推不出来（它初始化为 None），
        # 会报「wait_for_frames 不是 None 的已知属性」。局部变量 + 显式判空
        # 让检查器和读代码的人都能确认这里安全。
        pipeline = self.pipeline
        if pipeline is None:
            return None
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as exc:
            rospy.logwarn_throttle(2.0, "RealSense 取帧超时：%s", exc)
            return None
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data())

    # ------------------------------------------------------------------
    def control_callback(self, msg):
        command = (msg.data or "").strip().lower()
        if command in ("idle", "pause", "stop", "off"):
            self.mode = "idle"
            if self.source == "standalone":
                self.stop_camera()
            rospy.loginfo("视觉模式 -> idle")
        elif command == "flag":
            self.mode = "flag"
            self.flag_confirm_count = 0
            rospy.loginfo("视觉模式 -> flag")
        elif command in ("light", "traffic_light"):
            self.mode = "light"
            rospy.loginfo("视觉模式 -> light")
        else:
            rospy.logwarn("未知视觉控制指令：%s", command)

    # ------------------------------------------------------------------
    # 检测
    # ------------------------------------------------------------------
    def load_yolo(self):
        if not self.use_yolo or self.model is not None:
            return
        # 先查权重，省掉不必要的导入
        if not os.path.exists(self.weights):
            rospy.logwarn("红绿灯权重不存在：%s，改用 HSV 兜底。", self.weights)
            return
        # 改动 2026-07-30（修订）：ultralytics 就地导入。
        # 见文件顶部说明 —— 模块级导入会让 YOLO 这个名字在 except 分支未绑定。
        try:
            from ultralytics import YOLO
        except ImportError:
            rospy.logwarn("ultralytics 不可用，红绿灯改用 HSV 兜底。")
            return
        rospy.loginfo("加载红绿灯模型：%s", self.weights)
        self.model = YOLO(self.weights)
        self.names = self.model.names

    def detect_red_flag(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 改动 2026-07-30：原为两个掩膜用 | 相或。运行时是 numpy 按位或，
        # 没问题，但 cv2 的类型存根不认 MatLike 的 |。cv2.bitwise_or 是
        # OpenCV 的惯用写法，语义完全相同。
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.lower_red1, self.upper_red1),
            cv2.inRange(hsv, self.lower_red2, self.upper_red2),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        best_area = 0.0
        best_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > best_area:
                best_area = area
                best_contour = contour
        return best_area, best_contour

    def detect_light_hsv(self, roi):
        if roi.size == 0 or roi.shape[0] < 8 or roi.shape[1] < 8:
            return "unknown", 0.0
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        bright = (s >= self.min_saturation) & (v >= self.min_value)
        scores = {
            "red": float((((h <= 10) | (h >= 160)) & bright).sum()) / max(h.size, 1),
            "yellow": float((((h >= 15) & (h <= 40)) & bright).sum()) / max(h.size, 1),
            "green": float((((h >= 40) & (h <= 95)) & bright).sum()) / max(h.size, 1),
        }
        # 改动 2026-07-30：原为 key=scores.get。dict.get 的返回类型是
        # Optional[float]，类型检查器会报 max 的重载不匹配。运行时没问题
        # （键一定存在，永远不会返回 None），但索引写法更准确也更快。
        best = max(scores, key=lambda name: scores[name])
        if scores[best] < 0.01:
            return "unknown", scores[best]
        return best, scores[best]

    def process_light(self, frame, display):
        self.load_yolo()
        state = "none"
        detected_states = []
        colors = {
            "red": (0, 0, 255), "green": (0, 255, 0), "yellow": (0, 255, 255),
            "unknown": (128, 128, 128), "none": (255, 255, 255),
        }

        if self.model is not None:
            results = self.model.predict(
                frame, conf=self.conf, imgsz=self.imgsz,
                device=self.device, verbose=False,
            )
            if results and results[0].boxes is not None and len(results[0].boxes):
                boxes = results[0].boxes
                for i in range(len(boxes)):
                    x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                    score = float(boxes.conf[i].cpu().numpy())
                    cls_id = int(boxes.cls[i].cpu().numpy())
                    label = str(self.names.get(cls_id, cls_id)).lower()
                    roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]

                    if "red" in label:
                        box_state = "red"
                    elif "green" in label:
                        box_state = "green"
                    elif "yellow" in label:
                        box_state = "yellow"
                    else:
                        box_state, _ = self.detect_light_hsv(roi)

                    detected_states.append(box_state)

                    color = colors.get(box_state, (255, 255, 255))
                    cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        display, "%s %.2f" % (box_state, score),
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
                    )

                # 同一画面存在多个检测框时，红灯拥有最高优先级：只要看见
                # 任意红灯就发布 red，避免后处理框把状态覆盖成 green/none。
                if "red" in detected_states:
                    state = "red"
                elif "green" in detected_states:
                    state = "green"
                elif "yellow" in detected_states:
                    state = "yellow"
                elif detected_states:
                    state = "unknown"
        else:
            state, _ = self.detect_light_hsv(frame)

        rospy.loginfo_throttle(1.0, "红绿灯：%s", state)
        self.light_pub.publish(String(data=state))
        cv2.putText(
            display, "Light: %s" % state, (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors.get(state, (255, 255, 255)), 2,
        )

    def process_flag(self, frame, display):
        area, contour = self.detect_red_flag(frame)
        if contour is not None and area >= self.flag_min_area:
            self.flag_confirm_count += 1
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                display, "RED area=%d" % area, (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
        else:
            self.flag_confirm_count = 0

        cv2.putText(
            display,
            "FLAG %d/%d" % (self.flag_confirm_count, self.flag_confirm_threshold),
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2,
        )

        if self.flag_confirm_count >= self.flag_confirm_threshold:
            rospy.loginfo("红旗确认，发布起步信号。")
            self.start_pub.publish(Bool(data=True))
            self.mode = "idle"
            if self.source == "standalone":
                self.stop_camera()

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(30)
        # 改动 2026-07-30：状态话题按 1Hz 发，不跟着 30Hz 刷
        last_status_at = rospy.Time.now()
        while not rospy.is_shutdown():
            # ===== 改动 2026-07-30：按需中继 =====
            # idle 时停发帧；flag/light 时周期性请求（兼作心跳）。
            if self.mode == "idle":
                self.request_relay(False)
            else:
                self.request_relay(True)
            # ===== 改动结束 =====

            now = rospy.Time.now()
            if (now - last_status_at).to_sec() >= 1.0:
                self.publish_status()
                last_status_at = now

            if self.mode == "idle":
                if self.display_ok:
                    fresh_debug = (
                        self.latest_arm_detection_frame is not None
                        and self.last_arm_detection_at is not None
                        and (now - self.last_arm_detection_at).to_sec()
                        <= self.arm_detection_hold_time
                    )
                    try:
                        if fresh_debug:
                            cv2.imshow(
                                "Final Mission Vision",
                                self.latest_arm_detection_frame,
                            )
                        # 即使暂时没有新图，也持续处理 GUI 事件，避免卡片识别
                        # 成功后窗口因为 waitKey 停止调用而变成“未响应”。
                        cv2.waitKey(1)
                    except cv2.error as exc:
                        rospy.logwarn("机械臂检测图像显示失败：%s", exc)
                        self.display_ok = False
                rate.sleep()
                continue

            frame = self.grab_frame()
            if frame is None:
                if self.source == "arm_topic":
                    rospy.logwarn_throttle(
                        5.0,
                        "还没收到图像（%s）。请确认 piper_task 已接入中继补丁"
                        "（camera_relay.py）且 enable_camera_relay:=true。",
                        self.image_topic,
                    )
                rate.sleep()
                continue

            display = frame.copy()
            if self.mode == "flag":
                self.process_flag(frame, display)
            elif self.mode == "light":
                self.process_light(frame, display)

            # 改动 2026-07-30：原为 if self.show_image，改用探测过的 display_ok，
            # 没有 DISPLAY 时不会抛 cv2.error 打断循环。
            if self.display_ok:
                try:
                    cv2.imshow("Final Mission Vision", display)
                    cv2.waitKey(1)
                except cv2.error as exc:
                    # 运行中窗口被关掉/X 断开也不该让识别停摆
                    rospy.logwarn("图像显示失败，后续不再显示：%s", exc)
                    self.display_ok = False
            rate.sleep()

    def on_shutdown(self):
        # 改动 2026-07-30：退出前务必让 piper_task 停发帧
        self.request_relay(False)
        self.stop_camera()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


if __name__ == "__main__":
    try:
        FinalVision().spin()
    except rospy.ROSInterruptException:
        pass
