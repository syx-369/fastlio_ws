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


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class PurePursuitFollower:
    def __init__(self):
        # ---------------- 参数 ----------------
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/smoother_cmd_vel")
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))

        self.lookahead = float(rospy.get_param("~lookahead_distance", 0.8))
        self.min_lookahead = float(rospy.get_param("~min_lookahead", 0.4))
        self.max_lookahead = float(rospy.get_param("~max_lookahead", 1.5))
        self.lookahead_speed_ratio = float(rospy.get_param("~lookahead_speed_ratio", 1.5))

        self.target_speed = float(rospy.get_param("~target_speed", 0.5))
        self.max_linear = float(rospy.get_param("~max_linear", 0.6))
        self.max_angular = float(rospy.get_param("~max_angular", 1.0))
        self.vehicle_model = self.normalize_vehicle_model(rospy.get_param("~vehicle_model", "diff"))
        self.wheelbase = float(rospy.get_param("~wheelbase", 0.65))
        self.max_steer_angle = float(rospy.get_param("~max_steer_angle", 0.461))
        if self.wheelbase <= 0.0:
            rospy.logwarn("Invalid wheelbase %.3f, reset to 0.65.", self.wheelbase)
            self.wheelbase = 0.65
        if self.max_steer_angle <= 0.0:
            rospy.logwarn("Invalid max_steer_angle %.3f, reset to 0.461.", self.max_steer_angle)
            self.max_steer_angle = 0.461

        self.goal_reached_dist = float(rospy.get_param("~goal_reached_dist", 0.05))
        self.approach_switch_dist = float(rospy.get_param("~approach_switch_dist", 0.25))
        self.task_approach_dist = float(rospy.get_param("~task_approach_dist", 0.5))
        self.approach_k_linear = float(rospy.get_param("~approach_k_linear", 0.5))

        self.final_yaw_k = float(rospy.get_param("~final_yaw_k", 1.5))
        self.final_yaw_tolerance = float(rospy.get_param("~final_yaw_tolerance", 0.08))
        self.enable_final_yaw_align = parse_bool(rospy.get_param("~enable_final_yaw_align", True))
        self.finish_stop_time = float(rospy.get_param("~finish_stop_time", 1.0))

        self.external_task_timeout = float(rospy.get_param("~external_task_timeout", 180.0))
        self.unknown_task_policy = rospy.get_param("~unknown_task_policy", "skip").strip().lower()
        self.detect_pause_time = float(rospy.get_param("~detect_pause_time", 2.0))
        self.task_event_topic = rospy.get_param("~task_event_topic", "/waypoint_task_event")
        self.task_done_topic = rospy.get_param("~task_done_topic", "/waypoint_task_done")

        self.csv_path = rospy.get_param("~csv_path", "")
        if self.csv_path.strip() == "":
            self.csv_path = self.find_latest_csv()
        if not os.path.isfile(self.csv_path):
            raise FileNotFoundError(f"CSV file not found: {self.csv_path}")

        # ---------------- 状态 ----------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

        self.state = "WAIT_ODOM"
        self.path_index = 0  # 当前最近点索引（进度）
        self.task_end_time = None
        self.pending_task_name = None
        self.pending_task_is_external = False
        self.after_task_next_state = "TRACK"
        self.final_stop_until = None

        # ---------------- 加载路径 ----------------
        self.waypoints = self.load_waypoints(self.csv_path)
        if len(self.waypoints) == 0:
            raise RuntimeError("No waypoints in CSV.")

        self.task_indices = [
            i for i, wp in enumerate(self.waypoints) if wp["task"] != "none"
        ]

        # ---------------- ROS ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.task_event_pub = rospy.Publisher(self.task_event_topic, String, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        rospy.Subscriber(self.task_done_topic, String, self.task_done_callback, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("==============================================")
        rospy.loginfo("Pure Pursuit Follower started.")
        rospy.loginfo("CSV file       : %s", self.csv_path)
        rospy.loginfo("Path points    : %d", len(self.waypoints))
        rospy.loginfo("Task points    : %d", len(self.task_indices))
        rospy.loginfo("Lookahead      : %.2f m", self.lookahead)
        rospy.loginfo("Target speed   : %.2f m/s", self.target_speed)
        rospy.loginfo("Cmd topic      : %s", self.cmd_topic)
        rospy.loginfo("Vehicle model  : %s", self.vehicle_model)
        if self.is_ackermann():
            rospy.loginfo("Wheelbase      : %.3f m", self.wheelbase)
            rospy.loginfo("Max steer      : %.3f rad", self.max_steer_angle)
            rospy.loginfo("Final yaw align: %s", "on" if self.enable_final_yaw_align else "off")
        rospy.loginfo("==============================================")

    # ================================================================
    # CSV 加载
    # ================================================================
    def normalize_vehicle_model(self, value):
        model = str(value).strip().lower()
        if model in ("ackermann", "ackerman", "hunter", "car"):
            return "ackermann"
        if model in ("diff", "differential", "bunker", "track", "tracked"):
            return "diff"
        rospy.logwarn("Unknown vehicle_model '%s', fallback to diff.", model)
        return "diff"

    def is_ackermann(self):
        return self.vehicle_model == "ackermann"

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
                    "tol": float(row["tol"]) if row.get("tol", "") else 0.3,
                }
                waypoints.append(wp)
        return waypoints

    # ================================================================
    # 回调
    # ================================================================
    def odom_callback(self, msg):
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
            self.path_index = self.find_closest_index()
            rospy.loginfo("First odom received. Starting from path index %d.", self.path_index)

    def task_done_callback(self, msg):
        if self.state != "TASK" or not self.pending_task_is_external or not self.pending_task_name:
            return
        done_msg = msg.data.strip()
        if done_msg == "":
            return
        expected_msg = f"done:{self.pending_task_name}"
        if done_msg in ("done", "all_done", self.pending_task_name, expected_msg):
            rospy.loginfo("External task '%s' done.", self.pending_task_name)
            self.task_end_time = rospy.Time.now()

    # ================================================================
    # Pure Pursuit 核心
    # ================================================================
    def find_closest_index(self):
        min_dist = float('inf')
        min_idx = 0
        for i, wp in enumerate(self.waypoints):
            d = math.hypot(wp["x"] - self.current_x, wp["y"] - self.current_y)
            if d < min_dist:
                min_dist = d
                min_idx = i
        return min_idx

    def update_path_index(self):
        search_start = self.path_index
        search_end = min(len(self.waypoints), self.path_index + 10)

        # 不能跳过未处理的任务点
        for ti in self.task_indices:
            if ti > self.path_index:
                search_end = min(search_end, ti + 1)
                break

        min_dist = float('inf')
        min_idx = self.path_index

        for i in range(search_start, search_end):
            wp = self.waypoints[i]
            d = math.hypot(wp["x"] - self.current_x, wp["y"] - self.current_y)
            if d < min_dist:
                min_dist = d
                min_idx = i

        self.path_index = min_idx

    def find_lookahead_point(self, ld):
        for i in range(self.path_index, len(self.waypoints)):
            wp = self.waypoints[i]
            d = math.hypot(wp["x"] - self.current_x, wp["y"] - self.current_y)
            if d >= ld:
                return wp["x"], wp["y"], i
        last = self.waypoints[-1]
        return last["x"], last["y"], len(self.waypoints) - 1

    def compute_adaptive_lookahead(self, speed):
        ld = self.lookahead_speed_ratio * abs(speed)
        return clamp(ld, self.min_lookahead, self.max_lookahead)

    def get_next_task_index(self):
        for ti in self.task_indices:
            if ti >= self.path_index:
                return ti
        return None

    def distance_along_path(self, from_idx, to_idx):
        dist = 0.0
        for i in range(from_idx, to_idx):
            wp_a = self.waypoints[i]
            wp_b = self.waypoints[i + 1]
            dist += math.hypot(wp_b["x"] - wp_a["x"], wp_b["y"] - wp_a["y"])
        return dist

    # ================================================================
    # 任务处理（复用 follow_waypoints 逻辑）
    # ================================================================
    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def angular_cmd_from_curvature(self, curvature, speed):
        if self.is_ackermann():
            steer_angle = math.atan(self.wheelbase * curvature)
            return clamp(steer_angle, -self.max_steer_angle, self.max_steer_angle)
        yaw_rate = speed * curvature
        return clamp(yaw_rate, -self.max_angular, self.max_angular)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def publish_task_event(self, phase, task_name):
        event = String()
        event.data = f"{phase}:{task_name}:idx{self.path_index}"
        self.task_event_pub.publish(event)

    def parse_stop_seconds(self, task_name):
        match = re.match(r"(?:stop|hold|pause)_(\d+(?:\.\d+)?)([sm])$", task_name)
        if not match:
            return None
        value = float(match.group(1))
        unit = match.group(2)
        return value * 60.0 if unit == "m" else value

    def start_task(self, task_name, next_state="TRACK"):
        if task_name == "none" or task_name == "":
            self.state = next_state
            return

        stop_seconds = self.parse_stop_seconds(task_name)
        if stop_seconds is not None:
            rospy.loginfo("Task '%s': stop %.1f s.", task_name, stop_seconds)
            self.task_end_time = rospy.Time.now() + rospy.Duration(stop_seconds)
            self.pending_task_name = task_name
            self.pending_task_is_external = False
            self.after_task_next_state = next_state
            self.publish_task_event("start", task_name)
            self.state = "TASK"
            return

        if task_name == "detect":
            rospy.loginfo("Task 'detect': pause %.1f s.", self.detect_pause_time)
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
                rospy.logwarn("Empty external task name, skip.")
                self.state = next_state
                return
            rospy.loginfo("External task '%s': waiting for done.", external_name)
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
            rospy.logwarn("Unknown task '%s', hold %.1f s.", task_name, self.detect_pause_time)
            self.task_end_time = rospy.Time.now() + rospy.Duration(self.detect_pause_time)
            self.pending_task_name = task_name
            self.pending_task_is_external = False
            self.after_task_next_state = next_state
            self.publish_task_event("start", task_name)
            self.state = "TASK"
            return

        rospy.logwarn("Unknown task '%s', skip.", task_name)
        self.publish_task_event("skip", task_name)
        self.state = next_state

    def handle_task_state(self):
        self.stop_robot()

        if self.pending_task_is_external and self.task_end_time is None:
            return

        if self.task_end_time is None:
            self.publish_task_event("done", self.pending_task_name or "none")
            self.pending_task_name = None
            self.pending_task_is_external = False
            self.state = self.after_task_next_state
            return

        if rospy.Time.now() >= self.task_end_time:
            rospy.loginfo("Task '%s' finished.", self.pending_task_name)
            self.publish_task_event("done", self.pending_task_name or "none")
            self.task_end_time = None
            self.pending_task_name = None
            self.pending_task_is_external = False
            self.state = self.after_task_next_state

    # ================================================================
    # 精确逼近（最后几厘米用P控制）
    # ================================================================
    def approach_target(self, tx, ty):
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_heading - self.current_yaw)

        linear_x = clamp(self.approach_k_linear * dist, 0.0, 0.15)

        if self.is_ackermann():
            if dist > self.goal_reached_dist:
                linear_x = clamp(linear_x, 0.04, 0.15)
            if abs(heading_error) > 1.2:
                linear_x *= 0.4
                rospy.logwarn_throttle(
                    2.0,
                    "Ackermann approach heading error is large: %.2f rad. "
                    "Check waypoint heading/path smoothness.",
                    heading_error,
                )
            curvature = 2.0 * math.sin(heading_error) / max(dist, 0.10)
            angular_z = self.angular_cmd_from_curvature(curvature, linear_x)
        else:
            angular_z = clamp(1.2 * heading_error, -self.max_angular, self.max_angular)
            if abs(heading_error) > 0.8:
                linear_x = 0.0

        self.publish_cmd(linear_x, angular_z)

    # ================================================================
    # 终点对齐
    # ================================================================
    def handle_align_final(self):
        final_wp = self.waypoints[-1]
        yaw_error = wrap_to_pi(final_wp["yaw"] - self.current_yaw)

        if not self.enable_final_yaw_align:
            rospy.loginfo("Final yaw align disabled. Finish after reaching final position.")
            self.state = "FINISH"
            self.final_stop_until = rospy.Time.now() + rospy.Duration(self.finish_stop_time)
            self.stop_robot()
            return

        if abs(yaw_error) <= self.final_yaw_tolerance:
            rospy.loginfo("Final yaw aligned.")
            self.state = "FINISH"
            self.final_stop_until = rospy.Time.now() + rospy.Duration(self.finish_stop_time)
            self.stop_robot()
            return

        if self.is_ackermann():
            rospy.logwarn(
                "Ackermann vehicle cannot rotate in place for final yaw alignment "
                "(yaw_error=%.3f rad). Finish without in-place yaw alignment.",
                yaw_error,
            )
            self.state = "FINISH"
            self.final_stop_until = rospy.Time.now() + rospy.Duration(self.finish_stop_time)
            self.stop_robot()
            return

        angular_z = clamp(self.final_yaw_k * yaw_error, -self.max_angular, self.max_angular)
        self.publish_cmd(0.0, angular_z)

    # ================================================================
    # 主控制循环
    # ================================================================
    def control_step(self):
        if not self.has_odom:
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

        if self.state != "TRACK":
            return

        # 更新路径进度
        self.update_path_index()

        # 检查是否到达终点（必须已经走过大部分路径才判定）
        last_wp = self.waypoints[-1]
        dist_to_end = math.hypot(last_wp["x"] - self.current_x, last_wp["y"] - self.current_y)
        near_end_of_path = self.path_index >= len(self.waypoints) - 10
        if dist_to_end < self.goal_reached_dist and near_end_of_path:
            rospy.loginfo("Reached end of path.")
            self.stop_robot()
            task = last_wp["task"]
            if task != "none":
                self.start_task(task, next_state="ALIGN_FINAL")
            else:
                self.state = "ALIGN_FINAL"
            return

        # 终点精确逼近：切换为P控制直接朝目标走
        if dist_to_end < self.approach_switch_dist and near_end_of_path:
            self.approach_target(last_wp["x"], last_wp["y"])
            return

        # 检查是否接近任务点
        next_task_idx = self.get_next_task_index()
        if next_task_idx is not None:
            task_wp = self.waypoints[next_task_idx]
            dist_to_task = math.hypot(task_wp["x"] - self.current_x, task_wp["y"] - self.current_y)
            if dist_to_task < self.goal_reached_dist:
                rospy.loginfo("Reached task point seq=%d, task=%s",
                              task_wp["seq"], task_wp["task"])
                self.stop_robot()
                self.path_index = next_task_idx
                self.task_indices.remove(next_task_idx)
                self.start_task(task_wp["task"], next_state="TRACK")
                return
            # 任务点精确逼近
            if dist_to_task < self.approach_switch_dist:
                self.approach_target(task_wp["x"], task_wp["y"])
                return

        # 计算速度（接近任务点时减速）
        speed = self.target_speed
        if next_task_idx is not None:
            task_wp = self.waypoints[next_task_idx]
            dist_to_task = math.hypot(task_wp["x"] - self.current_x, task_wp["y"] - self.current_y)
            if dist_to_task < self.task_approach_dist:
                speed = self.target_speed * (dist_to_task / self.task_approach_dist)
                speed = max(speed, 0.1)

        # 接近终点时减速（仅在路径末段）
        if dist_to_end < self.task_approach_dist and near_end_of_path:
            speed = min(speed, self.target_speed * (dist_to_end / self.task_approach_dist))
            speed = max(speed, 0.1)

        # 自适应前视距离
        ld = self.compute_adaptive_lookahead(speed)

        # 找前视点
        goal_x, goal_y, _ = self.find_lookahead_point(ld)

        # Pure Pursuit 计算
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        alpha = wrap_to_pi(math.atan2(dy, dx) - self.current_yaw)

        # 曲率
        actual_ld = math.hypot(dx, dy)
        if actual_ld < 0.01:
            self.publish_cmd(speed, 0.0)
            return

        curvature = 2.0 * math.sin(alpha) / actual_ld

        # 限幅
        linear_x = clamp(speed, 0.0, self.max_linear)
        angular_z = self.angular_cmd_from_curvature(curvature, speed)

        # 如果航向偏差太大，降低线速度优先转向
        if abs(alpha) > 1.0:
            linear_x *= 0.3
        elif abs(alpha) > 0.5:
            linear_x *= 0.6

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
        rospy.loginfo("Pure Pursuit Follower shutdown.")


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_follower", anonymous=False)
    try:
        follower = PurePursuitFollower()
        follower.spin()
    except rospy.ROSInterruptException:
        pass
