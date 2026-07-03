#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import re

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointFollower:
    def __init__(self):
        # ---------------- 参数 ----------------
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/smoother_cmd_vel")
        self.control_rate = float(rospy.get_param("~control_rate", 10.0))

        self.max_linear = float(rospy.get_param("~max_linear", 0.6))
        self.max_angular = float(rospy.get_param("~max_angular", 0.60))
        self.k_linear = float(rospy.get_param("~k_linear", 0.80))
        self.k_angular = float(rospy.get_param("~k_angular", 1.60))

        self.final_yaw_k = float(rospy.get_param("~final_yaw_k", 1.50))
        self.final_yaw_tolerance = float(rospy.get_param("~final_yaw_tolerance", 0.08))

        self.goal_tolerance_default = float(rospy.get_param("~goal_tolerance_default", 0.30))
        self.rotate_in_place_angle = float(rospy.get_param("~rotate_in_place_angle", 1.10))
        self.slowdown_angle = float(rospy.get_param("~slowdown_angle", 0.50))

        self.finish_stop_time = float(rospy.get_param("~finish_stop_time", 1.0))
        self.detect_pause_time = float(rospy.get_param("~detect_pause_time", 2.0))
        self.external_task_timeout = float(rospy.get_param("~external_task_timeout", 180.0))
        self.unknown_task_policy = rospy.get_param("~unknown_task_policy", "skip").strip().lower()
        self.task_event_topic = rospy.get_param("~task_event_topic", "/waypoint_task_event")
        self.task_done_topic = rospy.get_param("~task_done_topic", "/waypoint_task_done")

        self.csv_path = rospy.get_param("~csv_path", "")
        if self.csv_path.strip() == "":
            self.csv_path = self.find_latest_csv()

        if not os.path.isfile(self.csv_path):
            raise FileNotFoundError(f"CSV file not found: {self.csv_path}")

        # ---------------- 状态 ----------------
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.has_odom = False

        self.state = "WAIT_ODOM"   # WAIT_ODOM / TRACK / TASK / ALIGN_FINAL / FINISH
        self.current_index = 0
        self.task_end_time = None
        self.pending_task_name = None
        self.pending_task_is_external = False
        self.final_stop_until = None
        self.after_task_next_state = "TRACK"

        # ---------------- 读取航迹点 ----------------
        self.waypoints = self.load_waypoints(self.csv_path)
        if len(self.waypoints) == 0:
            raise RuntimeError("No waypoint found in csv.")

        # ---------------- ROS ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.task_event_pub = rospy.Publisher(self.task_event_topic, String, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        self.task_done_sub = rospy.Subscriber(self.task_done_topic, String, self.task_done_callback, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("==============================================")
        rospy.loginfo("Waypoint follower started.")
        rospy.loginfo("CSV file       : %s", self.csv_path)
        rospy.loginfo("Odom topic     : %s", self.odom_topic)
        rospy.loginfo("Cmd topic      : %s", self.cmd_topic)
        rospy.loginfo("Task event     : %s", self.task_event_topic)
        rospy.loginfo("Task done      : %s", self.task_done_topic)
        rospy.loginfo("Waypoint count : %d", len(self.waypoints))
        rospy.loginfo("Max linear     : %.3f m/s", self.max_linear)
        rospy.loginfo("Max angular    : %.3f rad/s", self.max_angular)
        rospy.loginfo("==============================================")

    def find_latest_csv(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.normpath(os.path.join(script_dir, "..", "data"))
        if not os.path.isdir(data_dir):
            raise FileNotFoundError(f"Data directory not found: {data_dir}")

        csv_files = [
            os.path.join(data_dir, f)
            for f in os.listdir(data_dir)
            if f.endswith(".csv")
        ]
        if not csv_files:
            raise FileNotFoundError(f"No csv file found in: {data_dir}")

        csv_files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return csv_files[0]

    def load_waypoints(self, csv_path):
        waypoints = []
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                wp = {
                    "seq": int(row["seq"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "yaw": float(row["yaw"]),
                    "task": row.get("task", "none").strip() if row.get("task", "") else "none",
                    "tol": float(row["tol"]) if row.get("tol", "") else self.goal_tolerance_default,
                }
                waypoints.append(wp)
        return waypoints

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        _, _, self.current_yaw = euler_from_quaternion([qx, qy, qz, qw])

        if not self.has_odom:
            self.has_odom = True
            self.state = "TRACK"
            rospy.loginfo("First odometry received. Switching to TRACK state.")

    def task_done_callback(self, msg: String):
        if self.state != "TASK" or not self.pending_task_is_external or not self.pending_task_name:
            return

        done_msg = msg.data.strip()
        if done_msg == "":
            return

        # Accept generic completion message and task-specific completion message.
        expected_msg = f"done:{self.pending_task_name}"
        if done_msg in ("done", "all_done", self.pending_task_name, expected_msg):
            rospy.loginfo("External task '%s' acknowledged by '%s'.", self.pending_task_name, done_msg)
            self.task_end_time = rospy.Time.now()

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def publish_task_event(self, phase, task_name):
        event = String()
        event.data = f"{phase}:{task_name}:idx{self.current_index}"
        self.task_event_pub.publish(event)

    def parse_stop_seconds(self, task_name):
        # Supported format: stop_120s / hold_120s / pause_120s / stop_2m
        match = re.match(r"(?:stop|hold|pause)_(\d+(?:\.\d+)?)([sm])$", task_name)
        if not match:
            return None

        value = float(match.group(1))
        unit = match.group(2)
        return value * 60.0 if unit == "m" else value

    def start_task(self, task_name, next_state="TRACK"):
        if task_name == "none" or task_name == "":
            self.current_index += 1
            self.state = next_state
            return

        stop_seconds = self.parse_stop_seconds(task_name)
        if stop_seconds is not None:
            rospy.loginfo("Task %s triggered at waypoint %d. Stop for %.1f s.",
                          task_name, self.current_index, stop_seconds)
            self.task_end_time = rospy.Time.now() + rospy.Duration(stop_seconds)
            self.pending_task_name = task_name
            self.pending_task_is_external = False
            self.after_task_next_state = next_state
            self.publish_task_event("start", task_name)
            self.state = "TASK"
            return

        if task_name == "detect":
            rospy.loginfo("Task detect triggered at waypoint %d. Placeholder wait %.1f s.",
                          self.current_index, self.detect_pause_time)
            self.task_end_time = rospy.Time.now() + rospy.Duration(self.detect_pause_time)
            self.pending_task_name = task_name
            self.pending_task_is_external = False
            self.after_task_next_state = next_state
            self.publish_task_event("start", task_name)
            self.state = "TASK"
            return

        if task_name.startswith("ext:"):
            external_name = task_name[4:].strip()
            if external_name == "":
                rospy.logwarn("Task '%s' has empty external action name. Skip it.", task_name)
                self.current_index += 1
                self.state = next_state
                return
            rospy.loginfo("External task %s triggered at waypoint %d. Waiting for done message.",
                          external_name, self.current_index)
            if self.external_task_timeout > 0.0:
                self.task_end_time = rospy.Time.now() + rospy.Duration(self.external_task_timeout)
            else:
                self.task_end_time = None
            self.pending_task_name = external_name
            self.pending_task_is_external = True
            self.after_task_next_state = next_state
            self.publish_task_event("start", external_name)
            self.state = "TASK"
            return

        if self.unknown_task_policy == "hold":
            rospy.logwarn("Unknown task '%s'. Hold %.1f s as fallback.",
                          task_name, self.detect_pause_time)
            self.task_end_time = rospy.Time.now() + rospy.Duration(self.detect_pause_time)
            self.pending_task_name = task_name
            self.pending_task_is_external = False
            self.after_task_next_state = next_state
            self.publish_task_event("start", task_name)
            self.state = "TASK"
            return

        rospy.logwarn("Unknown task '%s'. Skip it (policy=%s).", task_name, self.unknown_task_policy)
        self.publish_task_event("skip", task_name)
        self.current_index += 1
        self.state = next_state
                
    def handle_task_state(self):
        self.stop_robot()

        if self.pending_task_is_external and self.task_end_time is None:
            # Timeout <= 0 means wait indefinitely for external done signal.
            return

        if self.task_end_time is None:
            self.publish_task_event("done", self.pending_task_name if self.pending_task_name else "none")
            self.pending_task_name = None
            self.pending_task_is_external = False
            self.current_index += 1
            self.state = self.after_task_next_state
            return

        if rospy.Time.now() >= self.task_end_time:
            if self.pending_task_is_external:
                rospy.logwarn("External task '%s' finished by timeout or done signal.",
                              self.pending_task_name)
            rospy.loginfo("Task '%s' finished.", self.pending_task_name)
            self.publish_task_event("done", self.pending_task_name if self.pending_task_name else "none")
            self.task_end_time = None
            self.pending_task_name = None
            self.pending_task_is_external = False
            self.current_index += 1
            self.state = self.after_task_next_state

    def handle_align_final(self):
        final_wp = self.waypoints[-1]
        target_yaw = final_wp["yaw"]
        yaw_error = wrap_to_pi(target_yaw - self.current_yaw)

        if abs(yaw_error) <= self.final_yaw_tolerance:
            rospy.loginfo("Final yaw aligned. yaw_error=%.4f rad", yaw_error)
            self.state = "FINISH"
            self.final_stop_until = rospy.Time.now() + rospy.Duration(self.finish_stop_time)
            self.stop_robot()
            return

        angular_z = clamp(self.final_yaw_k * yaw_error, -self.max_angular, self.max_angular)
        self.publish_cmd(0.0, angular_z)

    def control_step(self):
        if not self.has_odom:
            self.stop_robot()
            return

        if self.state == "TASK":
            self.handle_task_state()
            return

        if self.state == "ALIGN_FINAL":
            self.handle_align_final()
            return

        if self.state == "FINISH":
            self.stop_robot()
            return

        if self.current_index >= len(self.waypoints):
            self.state = "FINISH"
            self.stop_robot()
            return

        wp = self.waypoints[self.current_index]
        tx = wp["x"]
        ty = wp["y"]
        tol = wp["tol"]
        task = wp["task"]

        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.hypot(dx, dy)

        is_last = (self.current_index == len(self.waypoints) - 1)

        # 中间普通点：到点直接切下一个，不停车
        if distance <= tol and (not is_last) and task == "none":
            rospy.loginfo("Pass waypoint %d | x=%.3f y=%.3f", wp["seq"], tx, ty)
            self.current_index += 1
            return

        # 中间任务点：到点停车做任务
        if distance <= tol and (not is_last) and task != "none":
            rospy.loginfo("Reached task waypoint %d | task=%s", wp["seq"], task)
            self.stop_robot()
            self.start_task(task, next_state="TRACK")
            return

        # 终点：先到位置，再执行终点任务，再对齐 yaw
        if distance <= tol and is_last:
            rospy.loginfo("Reached final waypoint position %d | x=%.3f y=%.3f",
                          wp["seq"], tx, ty)
            self.stop_robot()
            if task != "none":
                self.start_task(task, next_state="ALIGN_FINAL")
            else:
                self.state = "ALIGN_FINAL"
            return

        # 正常追踪
        target_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_heading - self.current_yaw)

        linear_x = clamp(self.k_linear * distance, 0.0, self.max_linear)
        angular_z = clamp(self.k_angular * heading_error, -self.max_angular, self.max_angular)

        if abs(heading_error) > self.rotate_in_place_angle:
            linear_x = 0.0
        elif abs(heading_error) > self.slowdown_angle:
            linear_x *= 0.35

        self.publish_cmd(linear_x, angular_z)

    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self.control_step()
            rate.sleep()

    def on_shutdown(self):
        self.stop_robot()
        rospy.sleep(0.1)
        self.stop_robot()
        rospy.loginfo("Follower shutdown: robot stopped.")


if __name__ == "__main__":
    rospy.init_node("follow_waypoints", anonymous=False)
    follower = WaypointFollower()
    follower.spin()
