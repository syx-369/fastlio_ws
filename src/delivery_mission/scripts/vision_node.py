#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一视觉节点：D435i 相机 + 红旗检测 + 红绿灯检测
- 启动后先做红旗检测（HSV），检测到后发布 /flag_detected
- 之后持续做红绿灯检测（YOLO + HSV），发布 /traffic_light_state
- 全程显示图像和检测结果
"""

import time
import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from std_msgs.msg import Bool, String
from ultralytics import YOLO


class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')

        # 参数
        self.weights = rospy.get_param("~weights", "/home/user/fastlio_ws/src/waypoint_tools/config/traffic_light.pt")
        self.conf = float(rospy.get_param("~conf", 0.4))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "0")
        self.show_image = rospy.get_param("~show_image", True)

        # 红旗参数
        self.flag_confirm_threshold = int(rospy.get_param("~flag_confirm_frames", 5))
        self.flag_min_area = int(rospy.get_param("~flag_min_area", 3000))

        # 红绿灯 HSV 参数
        self.min_saturation = int(rospy.get_param("~min_saturation", 70))
        self.min_value = int(rospy.get_param("~min_value", 90))

        # 状态
        self.flag_detected = False
        self.flag_confirm_count = 0

        # HSV 红色范围
        self.lower_red1 = np.array([0, 120, 80])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 80])
        self.upper_red2 = np.array([180, 255, 255])

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # YOLO（延迟加载，红旗检测阶段不需要）
        self.model = None

        # Publishers
        self.flag_pub = rospy.Publisher('/flag_detected', Bool, queue_size=1)
        self.state_pub = rospy.Publisher('/traffic_light_state', String, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("VisionNode started. Phase: RED FLAG detection.")

    def load_yolo(self):
        if self.model is None:
            rospy.loginfo("Loading YOLO model: %s", self.weights)
            self.model = YOLO(self.weights)
            self.names = self.model.names
            rospy.loginfo("YOLO model loaded.")

    # ================================================================
    # 红旗检测
    # ================================================================
    def detect_red_flag(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = mask1 | mask2

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        max_contour = None
        for c in contours:
            area = cv2.contourArea(c)
            if area > max_area:
                max_area = area
                max_contour = c

        return max_area, max_contour

    # ================================================================
    # 红绿灯检测
    # ================================================================
    def detect_light_state(self, roi_bgr):
        if roi_bgr.size == 0 or roi_bgr.shape[0] < 8 or roi_bgr.shape[1] < 8:
            return "unknown", 0.0

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        bright_mask = (s >= self.min_saturation) & (v >= self.min_value)

        red_mask = ((h <= 10) | (h >= 160)) & bright_mask
        yellow_mask = ((h >= 15) & (h <= 40)) & bright_mask
        green_mask = ((h >= 40) & (h <= 95)) & bright_mask

        roi_h, roi_w = roi_bgr.shape[:2]
        yy = np.indices((roi_h, roi_w))[0]
        one_third = roi_h / 3.0

        regions = {
            "red": yy < one_third,
            "yellow": (yy >= one_third) & (yy < 2 * one_third),
            "green": yy >= 2 * one_third,
        }
        color_masks = {"red": red_mask, "yellow": yellow_mask, "green": green_mask}

        scores = {}
        for color_name, c_mask in color_masks.items():
            region = regions[color_name]
            region_area = max(int(region.sum()), 1)
            masked = c_mask & region
            if masked.sum() == 0:
                scores[color_name] = 0.0
            else:
                pixel_frac = float(masked.sum()) / float(region_area)
                mean_bright = float(v[masked].mean()) / 255.0
                scores[color_name] = pixel_frac * mean_bright

        best = max(scores, key=scores.get)
        best_score = scores[best]

        if best_score < 0.01:
            return "unknown", best_score

        sorted_scores = sorted(scores.values(), reverse=True)
        if len(sorted_scores) > 1 and sorted_scores[1] > 0:
            if sorted_scores[0] / max(sorted_scores[1], 1e-6) < 1.15:
                return "unknown", best_score

        return best, best_score

    def process_traffic_light(self, frame, display):
        self.load_yolo()

        results = self.model.predict(
            frame, conf=self.conf, imgsz=self.imgsz,
            device=self.device, verbose=False
        )

        current_state = "none"
        color_map = {"red": (0, 0, 255), "green": (0, 255, 0),
                     "yellow": (0, 255, 255), "unknown": (128, 128, 128), "none": (255, 255, 255)}

        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                box = boxes.xyxy[i].cpu().numpy().astype(int)
                score = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                class_name = self.names.get(cls_id, str(cls_id))

                x1, y1, x2, y2 = box
                roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]

                lower_name = class_name.lower()
                if "red" in lower_name:
                    state, state_score = "red", 1.0
                elif "green" in lower_name:
                    state, state_score = "green", 1.0
                elif "yellow" in lower_name:
                    state, state_score = "yellow", 1.0
                else:
                    state, state_score = self.detect_light_state(roi)

                current_state = state

                box_color = color_map.get(state, (255, 255, 255))
                cv2.rectangle(display, (x1, y1), (x2, y2), box_color, 2)
                label = f"{state} ({score:.2f})"
                cv2.putText(display, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                rospy.loginfo_throttle(1, f"Traffic light: {state} (score={state_score:.3f}, conf={score:.2f})")

        self.state_pub.publish(String(data=current_state))

        # 状态文字
        state_color = color_map.get(current_state, (255, 255, 255))
        cv2.putText(display, f"Light: {current_state}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)

    # ================================================================
    # 主循环
    # ================================================================
    def run(self):
        rate = rospy.Rate(30)
        prev_time = time.time()

        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                rate.sleep()
                continue

            frame = np.asanyarray(color_frame.get_data())
            display = frame.copy()

            if not self.flag_detected:
                # ---- 红旗检测阶段 ----
                max_area, max_contour = self.detect_red_flag(frame)

                if max_contour is not None and max_area >= self.flag_min_area:
                    self.flag_confirm_count += 1
                    x, y, w, h = cv2.boundingRect(max_contour)
                    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(display, f"RED area={max_area}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    self.flag_confirm_count = 0

                status = f"FLAG: confirming {self.flag_confirm_count}/{self.flag_confirm_threshold}"
                cv2.putText(display, status, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                if self.flag_confirm_count >= self.flag_confirm_threshold:
                    self.flag_detected = True
                    self.flag_pub.publish(Bool(data=True))
                    rospy.loginfo("RED FLAG CONFIRMED!")

            else:
                # ---- 红绿灯检测阶段 ----
                self.process_traffic_light(frame, display)

                curr_time = time.time()
                fps_val = 1.0 / max(curr_time - prev_time, 1e-6)
                prev_time = curr_time
                cv2.putText(display, f"FPS: {fps_val:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            if self.show_image:
                cv2.imshow("Vision", display)
                cv2.waitKey(1)

            rate.sleep()

    def on_shutdown(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = VisionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
