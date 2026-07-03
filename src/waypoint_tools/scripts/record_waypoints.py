#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointRecorder:
    def __init__(self):
        # ---------- 参数 ----------
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.task_topic = rospy.get_param("~task_topic", "/waypoint_task")
        self.stop_seconds_topic = rospy.get_param("~stop_seconds_topic", "/waypoint_stop_seconds")

        self.min_distance = float(rospy.get_param("~min_distance", 0.30))
        self.min_yaw_change = float(rospy.get_param("~min_yaw_change", 0.35))
        self.default_task = rospy.get_param("~default_task", "none")
        self.default_tol = float(rospy.get_param("~default_tol", 0.30))
        self.default_stop_seconds = float(rospy.get_param("~default_stop_seconds", 10.0))
        self.record_z = bool(rospy.get_param("~record_z", False))
        self.flush_every = int(rospy.get_param("~flush_every", 1))
        self.frame_override = rospy.get_param("~frame_id", "")
        self.file_name = rospy.get_param("~file_name", "")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_output_dir = os.path.normpath(os.path.join(script_dir, "..", "data"))
        self.output_dir = rospy.get_param("~output_dir", default_output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        if self.file_name.strip() == "":
            time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.file_name = f"waypoints_{time_str}.csv"

        self.csv_path = os.path.join(self.output_dir, self.file_name)

        # ---------- 运行状态 ----------
        self.seq = 0
        self.last_saved_x = None
        self.last_saved_y = None
        self.last_saved_yaw = None
        self.first_msg_received = False

        self.current_msg = None
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_qx = None
        self.current_qy = None
        self.current_qz = None
        self.current_qw = None
        self.current_yaw = None
        self.current_frame_id = ""

        # ---------- 打开 CSV ----------
        self.csv_file = open(self.csv_path, mode="w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "seq",
            "stamp",
            "frame_id",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
            "yaw",
            "task",
            "tol"
        ])
        self.csv_file.flush()

        # ---------- ROS ----------
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        self.task_sub = rospy.Subscriber(self.task_topic, String, self.task_callback, queue_size=10)
        self.stop_sub = rospy.Subscriber(self.stop_seconds_topic, Float32, self.stop_callback, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("==============================================")
        rospy.loginfo("Waypoint recorder started.")
        rospy.loginfo("Subscribed odom topic : %s", self.odom_topic)
        rospy.loginfo("Subscribed task topic : %s", self.task_topic)
        rospy.loginfo("Subscribed stop topic : %s", self.stop_seconds_topic)
        rospy.loginfo("Output csv file       : %s", self.csv_path)
        rospy.loginfo("Min distance          : %.3f m", self.min_distance)
        rospy.loginfo("Min yaw change        : %.3f rad", self.min_yaw_change)
        rospy.loginfo("==============================================")

    def odom_callback(self, msg: Odometry):
        self.current_msg = msg

        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])

        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.current_qx = qx
        self.current_qy = qy
        self.current_qz = qz
        self.current_qw = qw
        self.current_yaw = yaw
        self.current_frame_id = msg.header.frame_id

        if not self.first_msg_received:
            self.first_msg_received = True
            rospy.loginfo("First odometry message received.")

        should_save = False
        reason = ""

        if self.last_saved_x is None:
            should_save = True
            reason = "first_point"
        else:
            dx = x - self.last_saved_x
            dy = y - self.last_saved_y
            dist = math.hypot(dx, dy)
            dyaw = abs(wrap_to_pi(yaw - self.last_saved_yaw))

            if dist >= self.min_distance:
                should_save = True
                reason = f"distance={dist:.3f}m"
            elif dyaw >= self.min_yaw_change:
                should_save = True
                reason = f"yaw_change={dyaw:.3f}rad"

        if should_save:
            self.save_waypoint(task=self.default_task, tol=self.default_tol, reason=reason, force=False)

    def task_callback(self, msg: String):
        task_name = msg.data.strip()
        if task_name == "":
            rospy.logwarn("Received empty task mark, ignored.")
            return

        if self.current_msg is None:
            rospy.logwarn("No odometry received yet, cannot mark task waypoint.")
            return

        # 任务点强制写入 CSV
        self.save_waypoint(task=task_name, tol=self.default_tol, reason=f"manual_task={task_name}", force=True)

    def stop_callback(self, msg: Float32):
        if self.current_msg is None:
            rospy.logwarn("No odometry received yet, cannot mark stop waypoint.")
            return

        seconds = float(msg.data)
        if seconds <= 0.0:
            seconds = self.default_stop_seconds

        # Keep task string compatible with follower parser.
        if abs(seconds - round(seconds)) < 1e-4:
            task_name = f"stop_{int(round(seconds))}s"
        else:
            task_name = f"stop_{seconds:.1f}s"

        self.save_waypoint(task=task_name, tol=self.default_tol, reason=f"manual_stop={seconds:.1f}s", force=True)

    def save_waypoint(self, task, tol, reason="", force=False):
        if self.current_msg is None:
            return

        x = self.current_x
        y = self.current_y
        z = self.current_z if self.record_z else 0.0
        qx = self.current_qx
        qy = self.current_qy
        qz = self.current_qz
        qw = self.current_qw
        yaw = self.current_yaw

        if not force and self.last_saved_x is not None:
            dx = x - self.last_saved_x
            dy = y - self.last_saved_y
            dist = math.hypot(dx, dy)
            dyaw = abs(wrap_to_pi(yaw - self.last_saved_yaw))
            if dist < 1e-6 and dyaw < 1e-6:
                return

        frame_id = self.frame_override.strip() if self.frame_override.strip() else self.current_frame_id
        stamp_sec = self.current_msg.header.stamp.to_sec()

        self.writer.writerow([
            self.seq,
            f"{stamp_sec:.6f}",
            frame_id,
            f"{x:.6f}",
            f"{y:.6f}",
            f"{z:.6f}",
            f"{qx:.6f}",
            f"{qy:.6f}",
            f"{qz:.6f}",
            f"{qw:.6f}",
            f"{yaw:.6f}",
            task,
            f"{tol:.3f}"
        ])

        self.seq += 1

        if self.seq % self.flush_every == 0:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())

        self.last_saved_x = x
        self.last_saved_y = y
        self.last_saved_yaw = yaw

        rospy.loginfo(
            "[%04d] saved waypoint | x=%.3f y=%.3f yaw=%.3f | task=%s | %s",
            self.seq - 1, x, y, yaw, task, reason
        )

    def on_shutdown(self):
        try:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
        except Exception:
            pass

        rospy.loginfo("==============================================")
        rospy.loginfo("Waypoint recorder stopped.")
        rospy.loginfo("Saved csv file: %s", self.csv_path)
        rospy.loginfo("Total waypoints: %d", self.seq)
        rospy.loginfo("==============================================")


if __name__ == "__main__":
    rospy.init_node("record_waypoints", anonymous=False)
    recorder = WaypointRecorder()
    rospy.spin()
