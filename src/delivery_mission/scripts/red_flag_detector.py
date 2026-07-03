#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import Bool


class RedFlagDetector:
    def __init__(self):
        rospy.init_node('red_flag_detector')

        self.detected = False
        self.confirm_count = 0
        self.confirm_threshold = int(rospy.get_param("~confirm_frames", 5))
        self.min_area = int(rospy.get_param("~min_area", 3000))
        self.show_image = rospy.get_param("~show_image", True)

        # HSV 红色范围（红色跨越0度，需要两段）
        self.lower_red1 = np.array([0, 120, 80])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 80])
        self.upper_red2 = np.array([180, 255, 255])

        self.flag_pub = rospy.Publisher('/flag_detected', Bool, queue_size=1)

        # 初始化 RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("RedFlagDetector started (pyrealsense2). Waiting for red flag...")

    def detect(self, frame):
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

        return max_area, max_contour, mask

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                rate.sleep()
                continue

            frame = np.asanyarray(color_frame.get_data())
            max_area, max_contour, mask = self.detect(frame)

            # 显示画面
            if self.show_image:
                display = frame.copy()
                if max_contour is not None and max_area >= self.min_area:
                    x, y, w, h = cv2.boundingRect(max_contour)
                    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(display, f"RED area={max_area}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                status = "DETECTED!" if self.detected else f"confirming {self.confirm_count}/{self.confirm_threshold}"
                cv2.putText(display, status, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("Red Flag Detector", display)
                cv2.waitKey(1)

            if self.detected:
                rate.sleep()
                continue

            # 确认逻辑
            if max_area >= self.min_area:
                self.confirm_count += 1
                rospy.loginfo_throttle(1, f"Red detected, confirming... ({self.confirm_count}/{self.confirm_threshold})")
            else:
                self.confirm_count = 0

            if self.confirm_count >= self.confirm_threshold:
                self.detected = True
                self.flag_pub.publish(Bool(data=True))
                rospy.loginfo("RED FLAG CONFIRMED! Published flag_detected=True")

            rate.sleep()

    def on_shutdown(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        detector = RedFlagDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
