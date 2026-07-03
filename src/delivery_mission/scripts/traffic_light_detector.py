#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from std_msgs.msg import String
from ultralytics import YOLO


class TrafficLightDetector:
    def __init__(self):
        rospy.init_node('traffic_light_detector')

        weights = rospy.get_param("~weights", "/home/user/fastlio_ws/src/waypoint_tools/config/traffic_light.pt")
        self.conf = float(rospy.get_param("~conf", 0.4))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "0")
        self.show_image = rospy.get_param("~show_image", True)
        self.min_saturation = int(rospy.get_param("~min_saturation", 70))
        self.min_value = int(rospy.get_param("~min_value", 90))

        self.model = YOLO(weights)
        self.names = self.model.names

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 发布灯态: "red" / "green" / "yellow" / "unknown" / "none"
        self.state_pub = rospy.Publisher('/traffic_light_state', String, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("TrafficLightDetector started. weights=%s", weights)

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
        one_third = roi_h / 3.0

        # 竖直排布：上红中黄下绿
        regions = {
            "red": np.indices((roi_h, roi_w))[0] < one_third,
            "yellow": (np.indices((roi_h, roi_w))[0] >= one_third) & (np.indices((roi_h, roi_w))[0] < 2 * one_third),
            "green": np.indices((roi_h, roi_w))[0] >= 2 * one_third,
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

    def run(self):
        rate = rospy.Rate(15)
        prev_time = time.time()

        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                rate.sleep()
                continue

            frame = np.asanyarray(color_frame.get_data())
            display = frame.copy()

            results = self.model.predict(
                frame, conf=self.conf, imgsz=self.imgsz,
                device=self.device, verbose=False
            )

            current_state = "none"

            if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                for i in range(len(boxes)):
                    box = boxes.xyxy[i].cpu().numpy().astype(int)
                    score = float(boxes.conf[i].cpu().numpy())
                    cls_id = int(boxes.cls[i].cpu().numpy())
                    class_name = self.names.get(cls_id, str(cls_id))

                    x1, y1, x2, y2 = box
                    roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]

                    # 如果模型类别名直接是 red/green/yellow
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

                    # 画框
                    color_map = {"red": (0, 0, 255), "green": (0, 255, 0),
                                 "yellow": (0, 255, 255), "unknown": (128, 128, 128)}
                    box_color = color_map.get(state, (255, 255, 255))

                    cv2.rectangle(display, (x1, y1), (x2, y2), box_color, 2)
                    label = f"{state} ({score:.2f})"
                    cv2.putText(display, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                    rospy.loginfo_throttle(1, f"Traffic light: {state} (score={state_score:.3f}, conf={score:.2f})")

            # 发布状态
            self.state_pub.publish(String(data=current_state))

            if self.show_image:
                curr_time = time.time()
                fps_val = 1.0 / max(curr_time - prev_time, 1e-6)
                prev_time = curr_time
                cv2.putText(display, f"FPS: {fps_val:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                cv2.putText(display, f"State: {current_state}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            color_map.get(current_state, (255, 255, 255)), 2)
                cv2.imshow("Traffic Light Detector", display)
                cv2.waitKey(1)

            rate.sleep()

    def on_shutdown(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        detector = TrafficLightDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
