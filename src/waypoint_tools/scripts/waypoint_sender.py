#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_sender.py
读取 CSV 航迹点，依次通过 move_base action 发送目标。
支持 follow_waypoints.py 中相同的 task 机制（stop_Xs / ext:xxx）。
"""

import csv
import math
import os
import re

import actionlib
import rospy
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class WaypointSender:
    def __init__(self):
        # ---- 参数 ----
        self.csv_path = rospy.get_param("~csv_path", "")
        if self.csv_path.strip() == "":
            self.csv_path = self.find_latest_csv()

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.30))
        self.skip_distance = float(rospy.get_param("~skip_distance", 1.5))
        self.external_task_timeout = float(rospy.get_param("~external_task_timeout", 180.0))
        self.unknown_task_policy = rospy.get_param("~unknown_task_policy", "skip").strip().lower()
        self.task_event_topic = rospy.get_param("~task_event_topic", "/waypoint_task_event")
        self.task_done_topic = rospy.get_param("~task_done_topic", "/waypoint_task_done")

        # ---- 读取航迹点 ----
        if not os.path.isfile(self.csv_path):
            raise FileNotFoundError(f"CSV not found: {self.csv_path}")
        self.waypoints = self.load_waypoints(self.csv_path)
        if len(self.waypoints) == 0:
            raise RuntimeError("No waypoints in CSV.")
        rospy.loginfo("Loaded %d raw waypoints from %s", len(self.waypoints), self.csv_path)

        # 稀疏化：保留有 task 的点、首尾点、以及间距 >= skip_distance 的点
        self.waypoints = self.sparsify(self.waypoints, self.skip_distance)
        rospy.loginfo("After sparsify: %d goal waypoints", len(self.waypoints))

        # ---- task 机制 ----
        self.task_event_pub = rospy.Publisher(self.task_event_topic, String, queue_size=1)
        self.task_done_received = False
        rospy.Subscriber(self.task_done_topic, String, self.task_done_cb)

        # ---- move_base action client ----
        rospy.loginfo("Waiting for move_base action server ...")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("move_base connected.")

    # ----------------------------------------------------------------
    def find_latest_csv(self):
        data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
        if not os.path.isdir(data_dir):
            raise FileNotFoundError(f"Data dir not found: {data_dir}")
        csvs = sorted([f for f in os.listdir(data_dir) if f.endswith(".csv")])
        if not csvs:
            raise FileNotFoundError("No CSV in data dir.")
        return os.path.join(data_dir, csvs[-1])

    # ----------------------------------------------------------------
    def load_waypoints(self, path):
        wps = []
        with open(path, newline="") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                try:
                    wp = {
                        "seq": int(row["seq"]),
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "z": float(row.get("z", 0)),
                        "qx": float(row.get("qx", 0)),
                        "qy": float(row.get("qy", 0)),
                        "qz": float(row.get("qz", 0)),
                        "qw": float(row.get("qw", 1)),
                        "yaw": float(row.get("yaw", 0)),
                        "task": row.get("task", "none").strip(),
                        "tol": float(row.get("tol", self.goal_tolerance)),
                        "frame_id": row.get("frame_id", "map").strip(),
                    }
                    wps.append(wp)
                except (ValueError, KeyError) as e:
                    rospy.logwarn("Skip bad row: %s", e)
        return wps

    # ----------------------------------------------------------------
    def sparsify(self, wps, min_dist):
        """保留首尾、有 task 的点、以及与上一个保留点距离 >= min_dist 的点。"""
        if len(wps) <= 2:
            return wps
        keep = [wps[0]]
        for wp in wps[1:-1]:
            if wp["task"] != "none":
                keep.append(wp)
                continue
            dx = wp["x"] - keep[-1]["x"]
            dy = wp["y"] - keep[-1]["y"]
            if math.sqrt(dx * dx + dy * dy) >= min_dist:
                keep.append(wp)
        keep.append(wps[-1])
        return keep

    # ----------------------------------------------------------------
    def task_done_cb(self, msg):
        self.task_done_received = True

    # ----------------------------------------------------------------
    def handle_task(self, task_str):
        """处理航迹点上的 task，返回后继续下一个航迹点。"""
        if task_str == "none":
            return

        # stop_Xs
        m = re.match(r"stop_(\d+(?:\.\d+)?)s?", task_str)
        if m:
            dur = float(m.group(1))
            rospy.loginfo("[TASK] stop %.1fs", dur)
            self.task_event_pub.publish(String(data=f"start:{task_str}"))
            rospy.sleep(dur)
            self.task_event_pub.publish(String(data=f"done:{task_str}"))
            return

        # ext:xxx  外部任务
        if task_str.startswith("ext:"):
            ext_name = task_str[4:]
            rospy.loginfo("[TASK] external: %s (timeout %.0fs)", ext_name, self.external_task_timeout)
            self.task_done_received = False
            self.task_event_pub.publish(String(data=f"start:{task_str}"))
            deadline = rospy.Time.now() + rospy.Duration(self.external_task_timeout)
            rate = rospy.Rate(5)
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                if self.task_done_received:
                    rospy.loginfo("[TASK] external done: %s", ext_name)
                    break
                rate.sleep()
            else:
                rospy.logwarn("[TASK] external timeout: %s", ext_name)
            self.task_event_pub.publish(String(data=f"done:{task_str}"))
            return

        # 未知 task
        if self.unknown_task_policy == "stop":
            rospy.logwarn("[TASK] unknown '%s', stopping.", task_str)
            rospy.signal_shutdown("Unknown task with stop policy")
        else:
            rospy.logwarn("[TASK] unknown '%s', skipping.", task_str)

    # ----------------------------------------------------------------
    def send_goal(self, wp):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = Quaternion(
            x=wp["qx"], y=wp["qy"], z=wp["qz"], w=wp["qw"]
        )
        self.client.send_goal(goal)

    # ----------------------------------------------------------------
    def run(self):
        for i, wp in enumerate(self.waypoints):
            if rospy.is_shutdown():
                break

            rospy.loginfo("=== Goal %d/%d  seq=%d  (%.2f, %.2f) task=%s ===",
                          i + 1, len(self.waypoints), wp["seq"],
                          wp["x"], wp["y"], wp["task"])

            self.send_goal(wp)

            # 等待到达
            finished = self.client.wait_for_result(rospy.Duration(120.0))
            state = self.client.get_state()

            if not finished:
                rospy.logwarn("Goal %d timed out, cancelling.", wp["seq"])
                self.client.cancel_goal()
                rospy.sleep(1.0)
            elif state != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn("Goal %d failed (state=%d), moving to next.", wp["seq"], state)
                rospy.sleep(0.5)
            else:
                rospy.loginfo("Goal %d reached.", wp["seq"])

            # 处理 task
            self.handle_task(wp["task"])

        rospy.loginfo("All waypoints done!")


if __name__ == "__main__":
    rospy.init_node("waypoint_sender", anonymous=False)
    try:
        sender = WaypointSender()
        sender.run()
    except Exception as e:
        rospy.logfatal("waypoint_sender error: %s", e)
