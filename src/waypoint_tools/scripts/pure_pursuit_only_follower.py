#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


@dataclass
class Waypoint:
    seq: int
    x: float
    y: float
    yaw: float
    tol: float


class PurePursuitOnlyFollower:
    def __init__(self):
        # ---------------- ROS topic ----------------
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/smoother_cmd_vel")
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))

        # ---------------- Speed and tracking params ----------------
        # target_speed/max_linear 可以启动时传参，也可以运行中 rosparam set 修改。
        self.target_speed = float(rospy.get_param("~target_speed", 1.00))
        self.max_linear = float(rospy.get_param("~max_linear", 1.50))
        self.max_linear_accel = float(rospy.get_param("~max_linear_accel", 0.15))
        self.max_angular = float(rospy.get_param("~max_angular", 0.80))

        self.lookahead_distance = float(rospy.get_param("~lookahead_distance", 0.60))
        self.min_lookahead = float(rospy.get_param("~min_lookahead", 0.35))
        self.max_lookahead = float(rospy.get_param("~max_lookahead", 1.20))
        self.lookahead_speed_ratio = float(rospy.get_param("~lookahead_speed_ratio", 1.5))

        self.rotate_in_place_angle = float(rospy.get_param("~rotate_in_place_angle", 0.75))
        self.rotate_k_angular = float(rospy.get_param("~rotate_k_angular", 1.20))
        self.heading_slow_angle = float(rospy.get_param("~heading_slow_angle", 0.45))
        self.heading_hard_slow_angle = float(rospy.get_param("~heading_hard_slow_angle", 0.90))
        self.final_approach_dist = float(rospy.get_param("~final_approach_dist", 0.80))
        self.goal_tolerance_default = float(rospy.get_param("~goal_tolerance_default", 0.30))

        # 航点已经到车身后方且距离不远时，认为已经错过/通过，避免回头追旧点。
        self.skip_behind_x = float(rospy.get_param("~skip_behind_x", -0.35))
        self.skip_behind_dist = float(rospy.get_param("~skip_behind_dist", 1.00))
        self.search_ahead_points = max(1, int(rospy.get_param("~search_ahead_points", 20)))
        self.min_forward_target = float(rospy.get_param("~min_forward_target", -0.05))

        # ---------------- CSV path ----------------
        self.csv_path = rospy.get_param("~csv_path", "")
        if self.csv_path.strip() == "":
            self.csv_path = self.find_latest_csv()
        if not os.path.isfile(self.csv_path):
            raise FileNotFoundError(f"CSV file not found: {self.csv_path}")

        self.waypoints = self.load_waypoints(self.csv_path)
        if not self.waypoints:
            raise RuntimeError("No waypoint found in csv.")

        # ---------------- Runtime state ----------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.finished = False
        self.path_index = 0
        self.last_cmd_linear = 0.0
        self.last_cmd_stamp = rospy.Time(0)

        # ---------------- ROS pub/sub ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("==============================================")
        rospy.loginfo("Pure Pursuit ONLY follower started.")
        rospy.loginfo("CSV file       : %s", self.csv_path)
        rospy.loginfo("Path points    : %d", len(self.waypoints))
        rospy.loginfo("Odom topic     : %s", self.odom_topic)
        rospy.loginfo("Cmd topic      : %s", self.cmd_topic)
        rospy.loginfo("Target speed   : %.2f m/s", self.target_speed)
        rospy.loginfo("Max linear     : %.2f m/s", self.max_linear)
        rospy.loginfo("Max angular    : %.2f rad/s", self.max_angular)
        rospy.loginfo("Lookahead      : %.2f m", self.lookahead_distance)
        rospy.loginfo("No obstacle avoidance, no A*, no traffic light, no flag/task handling.")
        rospy.loginfo("==============================================")

    # ================================================================
    # CSV and odom
    # ================================================================
    def find_latest_csv(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.normpath(os.path.join(script_dir, "..", "data"))
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
        waypoints: List[Waypoint] = []
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                # task 字段故意不读取；这个文件只做路径跟踪。
                waypoints.append(
                    Waypoint(
                        seq=int(row["seq"]),
                        x=float(row["x"]),
                        y=float(row["y"]),
                        yaw=float(row["yaw"]),
                        tol=float(row["tol"]) if row.get("tol", "") else self.goal_tolerance_default,
                    )
                )
        return waypoints

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        q = pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.has_odom:
            self.has_odom = True
            self.path_index = self.find_closest_index()
            rospy.loginfo("First odom received. Start from waypoint index %d.", self.path_index)

    # ================================================================
    # Pure Pursuit helpers
    # ================================================================
    def reload_speed_params(self):
        """允许运行中用 rosparam set 调线速度。"""
        self.target_speed = float(rospy.get_param("~target_speed", self.target_speed))
        self.max_linear = float(rospy.get_param("~max_linear", self.max_linear))
        self.max_linear_accel = float(rospy.get_param("~max_linear_accel", self.max_linear_accel))
        self.max_angular = float(rospy.get_param("~max_angular", self.max_angular))

    def point_in_body(self, x, y):
        rx = x - self.current_x
        ry = y - self.current_y
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        xb = cy * rx + sy * ry
        yb = -sy * rx + cy * ry
        return xb, yb

    def find_closest_index(self):
        best_i = 0
        best_d = float("inf")
        for i, wp in enumerate(self.waypoints):
            d = math.hypot(wp.x - self.current_x, wp.y - self.current_y)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def advance_waypoint_index(self):
        """按顺序推进 CSV 航点，已经在车后的近距离点直接跳过。"""
        while self.path_index < len(self.waypoints) - 1:
            wp = self.waypoints[self.path_index]
            d = math.hypot(wp.x - self.current_x, wp.y - self.current_y)
            xb, _ = self.point_in_body(wp.x, wp.y)
            if d <= wp.tol:
                rospy.loginfo("Pass waypoint %d | d=%.2f", wp.seq, d)
                self.path_index += 1
                continue
            if xb < self.skip_behind_x and d < self.skip_behind_dist:
                rospy.logwarn("Skip behind waypoint %d | d=%.2f xb=%.2f", wp.seq, d, xb)
                self.path_index += 1
                continue
            break

        # 在前方小窗口内找离车最近的点，避免路径索引滞后。
        search_end = min(len(self.waypoints), self.path_index + self.search_ahead_points)
        best_i = self.path_index
        best_d = float("inf")
        for i in range(self.path_index, search_end):
            wp = self.waypoints[i]
            d = math.hypot(wp.x - self.current_x, wp.y - self.current_y)
            if d < best_d:
                best_d = d
                best_i = i
        self.path_index = max(self.path_index, best_i)

    def compute_lookahead(self, speed):
        lookahead = max(self.lookahead_distance, abs(speed) * self.lookahead_speed_ratio)
        return clamp(lookahead, self.min_lookahead, self.max_lookahead)

    def find_lookahead_point(self, lookahead) -> Optional[Tuple[float, float]]:
        """从当前进度向前找前视点，这是 Pure Pursuit 的目标点。"""
        for i in range(self.path_index, len(self.waypoints)):
            wp = self.waypoints[i]
            xb, _ = self.point_in_body(wp.x, wp.y)
            if xb < self.min_forward_target and i < len(self.waypoints) - 1:
                continue
            d = math.hypot(wp.x - self.current_x, wp.y - self.current_y)
            if d >= lookahead:
                return wp.x, wp.y

        last = self.waypoints[-1]
        return last.x, last.y

    def angular_cmd_from_curvature(self, curvature, speed):
        return clamp(speed * curvature, -self.max_angular, self.max_angular)

    # ================================================================
    # Command and control loop
    # ================================================================
    def publish_cmd(self, linear_x, angular_z):
        now = rospy.Time.now()
        if self.last_cmd_stamp == rospy.Time(0):
            dt = 1.0 / max(1.0, self.control_rate)
        else:
            dt = max(0.0, (now - self.last_cmd_stamp).to_sec())

        # 只限制加速，停车仍然立即生效。
        if linear_x > self.last_cmd_linear:
            linear_x = min(linear_x, self.last_cmd_linear + self.max_linear_accel * dt)
        self.last_cmd_linear = linear_x
        self.last_cmd_stamp = now

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def control_step(self):
        if not self.has_odom or self.finished:
            self.stop_robot()
            return

        self.reload_speed_params()
        self.advance_waypoint_index()

        last = self.waypoints[-1]
        dist_to_end = math.hypot(last.x - self.current_x, last.y - self.current_y)
        if self.path_index >= len(self.waypoints) - 1 and dist_to_end <= last.tol:
            rospy.loginfo("Final waypoint reached. Stop.")
            self.finished = True
            self.stop_robot()
            return

        speed = clamp(self.target_speed, 0.0, self.max_linear)
        if dist_to_end < self.final_approach_dist:
            scale = clamp(dist_to_end / max(self.final_approach_dist, 1e-6), 0.25, 1.0)
            speed *= scale

        lookahead = self.compute_lookahead(speed)
        target = self.find_lookahead_point(lookahead)
        if target is None:
            self.stop_robot()
            return

        goal_x, goal_y = target
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        actual_ld = math.hypot(dx, dy)
        if actual_ld < 0.01:
            self.stop_robot()
            return

        # Pure Pursuit 核心：前视点方向 alpha -> 曲率 -> 角速度。
        alpha = wrap_to_pi(math.atan2(dy, dx) - self.current_yaw)
        curvature = 2.0 * math.sin(alpha) / actual_ld
        linear_x = speed
        angular_z = self.angular_cmd_from_curvature(curvature, speed)

        # Bunker 可以原地转，偏航角太大时先转正再走，减少起步跑偏。
        if abs(alpha) > self.rotate_in_place_angle:
            linear_x = 0.0
            angular_z = clamp(self.rotate_k_angular * alpha, -self.max_angular, self.max_angular)
        elif abs(alpha) > self.heading_hard_slow_angle:
            linear_x *= 0.30
        elif abs(alpha) > self.heading_slow_angle:
            linear_x *= 0.60

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
        rospy.loginfo("Pure Pursuit ONLY follower shutdown: robot stopped.")


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_only_follower", anonymous=False)
    follower = PurePursuitOnlyFollower()
    follower.spin()
