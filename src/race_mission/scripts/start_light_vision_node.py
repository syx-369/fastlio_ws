#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from std_msgs.msg import Bool, String

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class StartLightVisionNode:
    """Command-driven start flag and traffic-light vision node.

    /race/vision_control accepts:
      flag  -> open camera and detect the start red flag
      light -> open camera and publish traffic light state
      idle  -> release camera and do nothing
    """

    def __init__(self):
        rospy.init_node("start_light_vision_node", anonymous=False)

        self.weights = rospy.get_param("~weights", "/home/user/fastlio_ws/src/waypoint_tools/config/traffic_light.pt")
        self.conf = float(rospy.get_param("~conf", 0.4))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "0")
        self.show_image = bool(rospy.get_param("~show_image", True))
        self.use_yolo = bool(rospy.get_param("~use_yolo", True))

        self.flag_confirm_threshold = int(rospy.get_param("~flag_confirm_frames", 5))
        self.flag_min_area = int(rospy.get_param("~flag_min_area", 3000))
        self.min_saturation = int(rospy.get_param("~min_saturation", 70))
        self.min_value = int(rospy.get_param("~min_value", 90))

        self.start_signal_topic = rospy.get_param("~start_signal_topic", "/race/start_signal")
        self.traffic_light_topic = rospy.get_param("~traffic_light_topic", "/race/traffic_light")
        self.vision_control_topic = rospy.get_param("~vision_control_topic", "/race/vision_control")
        self.mode = rospy.get_param("~initial_mode", "flag").strip().lower()

        self.flag_detected = False
        self.flag_confirm_count = 0
        self.pipeline = None
        self.model = None
        self.names = {}

        self.lower_red1 = np.array([0, 120, 80])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 80])
        self.upper_red2 = np.array([180, 255, 255])

        self.start_pub = rospy.Publisher(self.start_signal_topic, Bool, queue_size=1, latch=True)
        self.flag_pub = rospy.Publisher("/flag_detected", Bool, queue_size=1, latch=True)
        self.light_pub = rospy.Publisher(self.traffic_light_topic, String, queue_size=1)
        self.compat_light_pub = rospy.Publisher("/traffic_light_state", String, queue_size=1)

        rospy.Subscriber(self.vision_control_topic, String, self.control_callback, queue_size=5)
        rospy.on_shutdown(self.on_shutdown)

    # ------------------------------------------------------------------
    # Camera lifecycle
    # ------------------------------------------------------------------
    def start_camera(self):
        if self.pipeline is not None:
            return True

        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            rospy.loginfo("Start/light vision camera opened.")
            return True
        except Exception as exc:
            self.pipeline = None
            rospy.logwarn_throttle(2.0, "Failed to open RealSense for start/light vision: %s", exc)
            return False

    def stop_camera(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
            rospy.loginfo("Start/light vision camera released.")
        if self.show_image:
            try:
                cv2.destroyWindow("Race Start/Light Vision")
            except cv2.error:
                pass

    def control_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ("idle", "pause", "stop", "off"):
            self.mode = "idle"
            self.stop_camera()
            rospy.loginfo("Start/light vision mode: idle.")
        elif command == "flag":
            self.mode = "flag"
            self.flag_detected = False
            self.flag_confirm_count = 0
            rospy.loginfo("Start/light vision mode: flag.")
        elif command in ("light", "traffic_light"):
            self.mode = "light"
            rospy.loginfo("Start/light vision mode: light.")
        else:
            rospy.logwarn("Unknown vision control command: %s", command)

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------
    def load_yolo(self):
        if not self.use_yolo or self.model is not None:
            return
        if YOLO is None:
            rospy.logwarn("ultralytics is not available; traffic light uses HSV fallback.")
            return
        if not os.path.exists(self.weights):
            rospy.logwarn("Traffic-light weights not found: %s; using HSV fallback.", self.weights)
            return

        rospy.loginfo("Loading traffic-light model: %s", self.weights)
        self.model = YOLO(self.weights)
        self.names = self.model.names
        rospy.loginfo("Traffic-light model loaded.")

    def detect_red_flag(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = mask1 | mask2

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0.0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
        return max_area, max_contour

    def detect_light_state_hsv(self, roi_bgr):
        if roi_bgr.size == 0 or roi_bgr.shape[0] < 8 or roi_bgr.shape[1] < 8:
            return "unknown", 0.0

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        bright_mask = (s >= self.min_saturation) & (v >= self.min_value)

        red_mask = ((h <= 10) | (h >= 160)) & bright_mask
        yellow_mask = ((h >= 15) & (h <= 40)) & bright_mask
        green_mask = ((h >= 40) & (h <= 95)) & bright_mask

        scores = {
            "red": float(red_mask.sum()) / max(red_mask.size, 1),
            "yellow": float(yellow_mask.sum()) / max(yellow_mask.size, 1),
            "green": float(green_mask.sum()) / max(green_mask.size, 1),
        }
        best = max(scores, key=scores.get)
        if scores[best] < 0.01:
            return "unknown", scores[best]
        return best, scores[best]

    def process_traffic_light(self, frame, display):
        self.load_yolo()
        current_state = "none"

        color_map = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "yellow": (0, 255, 255),
            "unknown": (128, 128, 128),
            "none": (255, 255, 255),
        }

        if self.model is not None:
            results = self.model.predict(
                frame, conf=self.conf, imgsz=self.imgsz,
                device=self.device, verbose=False
            )
            if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                for i in range(len(boxes)):
                    box = boxes.xyxy[i].cpu().numpy().astype(int)
                    score = float(boxes.conf[i].cpu().numpy())
                    cls_id = int(boxes.cls[i].cpu().numpy())
                    class_name = str(self.names.get(cls_id, cls_id)).lower()
                    x1, y1, x2, y2 = box
                    roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]

                    if "red" in class_name:
                        state, state_score = "red", 1.0
                    elif "green" in class_name:
                        state, state_score = "green", 1.0
                    elif "yellow" in class_name:
                        state, state_score = "yellow", 1.0
                    else:
                        state, state_score = self.detect_light_state_hsv(roi)

                    current_state = state
                    box_color = color_map.get(state, (255, 255, 255))
                    cv2.rectangle(display, (x1, y1), (x2, y2), box_color, 2)
                    cv2.putText(display, "%s %.2f" % (state, score), (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    rospy.loginfo_throttle(
                        1.0, "Traffic light: %s score=%.3f conf=%.2f",
                        state, state_score, score
                    )
        else:
            current_state, state_score = self.detect_light_state_hsv(frame)
            rospy.loginfo_throttle(1.0, "Traffic light HSV fallback: %s score=%.3f", current_state, state_score)

        self.light_pub.publish(String(data=current_state))
        self.compat_light_pub.publish(String(data=current_state))
        cv2.putText(display, "Light: %s" % current_state, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_map.get(current_state, (255, 255, 255)), 2)

    # ------------------------------------------------------------------
    # Main
    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(30)
        prev_time = time.time()

        while not rospy.is_shutdown():
            if self.mode == "idle":
                self.stop_camera()
                rate.sleep()
                continue

            if not self.start_camera():
                rate.sleep()
                continue

            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError as exc:
                rospy.logwarn_throttle(2.0, "RealSense frame timeout: %s", exc)
                rate.sleep()
                continue

            color_frame = frames.get_color_frame()
            if not color_frame:
                rate.sleep()
                continue

            frame = np.asanyarray(color_frame.get_data())
            display = frame.copy()

            if self.mode == "flag":
                max_area, max_contour = self.detect_red_flag(frame)
                if max_contour is not None and max_area >= self.flag_min_area:
                    self.flag_confirm_count += 1
                    x, y, w, h = cv2.boundingRect(max_contour)
                    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(display, "RED area=%d" % max_area, (x, max(20, y - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    self.flag_confirm_count = 0

                cv2.putText(display, "FLAG %d/%d" % (self.flag_confirm_count, self.flag_confirm_threshold),
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

                if self.flag_confirm_count >= self.flag_confirm_threshold:
                    self.flag_detected = True
                    self.start_pub.publish(Bool(data=True))
                    self.flag_pub.publish(Bool(data=True))
                    rospy.loginfo("Red flag confirmed. Start signal published.")
                    self.mode = "idle"
                    self.stop_camera()
                    rate.sleep()
                    continue
            elif self.mode == "light":
                self.process_traffic_light(frame, display)
                curr_time = time.time()
                fps_val = 1.0 / max(curr_time - prev_time, 1e-6)
                prev_time = curr_time
                cv2.putText(display, "FPS %.1f" % fps_val, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)
            else:
                rospy.logwarn_throttle(2.0, "Unsupported vision mode '%s'; switching to idle.", self.mode)
                self.mode = "idle"
                continue

            if self.show_image:
                cv2.imshow("Race Start/Light Vision", display)
                cv2.waitKey(1)

            rate.sleep()

    def on_shutdown(self):
        self.stop_camera()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


if __name__ == "__main__":
    try:
        StartLightVisionNode().spin()
    except rospy.ROSInterruptException:
        pass
