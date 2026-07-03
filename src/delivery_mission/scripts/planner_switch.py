#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Planner Switch Node
-------------------
正常: 转发 pure_pursuit 的 /pp_cmd_vel
障碍物: 切换到 move_base(TEB) 的 /teb_cmd_vel
障碍物消失: 切回 pure_pursuit
"""

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
import tf.transformations as tft


class PlannerSwitch:
    def __init__(self):
        rospy.init_node('planner_switch')

        # 参数
        self.obstacle_dist_thresh = rospy.get_param("~obstacle_dist", 1.2)
        self.clear_dist_thresh = rospy.get_param("~clear_dist", 1.8)
        self.forward_angle = rospy.get_param("~forward_angle", 60.0)
        self.switch_cooldown = rospy.get_param("~switch_cooldown", 2.0)
        self.goal_forward_dist = rospy.get_param("~goal_forward_dist", 3.0)

        # 状态
        self.mode = "pure_pursuit"
        self.last_switch_time = rospy.Time.now()
        self.min_front_dist = float('inf')
        self.current_odom = None
        self.pp_cmd = Twist()
        self.teb_cmd = Twist()
        self.teb_goal_sent = False
        self.pp_active = False  # pure_pursuit 是否已开始发布指令

        # 订阅
        rospy.Subscriber('/pp_cmd_vel', Twist, self.pp_cmd_callback)
        rospy.Subscriber('/teb_cmd_vel', Twist, self.teb_cmd_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.mb_status_callback)

        # 发布
        self.cmd_pub = rospy.Publisher('/smoother_cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.mode_pub = rospy.Publisher('/planner_mode', String, queue_size=1, latch=True)

        self.mode_pub.publish(String(data=self.mode))
        rospy.loginfo("PlannerSwitch: obstacle_dist=%.2f, clear_dist=%.2f, forward_angle=%.0f",
                      self.obstacle_dist_thresh, self.clear_dist_thresh, self.forward_angle)

    def pp_cmd_callback(self, msg):
        self.pp_cmd = msg
        if not self.pp_active:
            self.pp_active = True
            self.last_switch_time = rospy.Time.now()
            rospy.loginfo("PlannerSwitch: pure_pursuit is now active.")

    def teb_cmd_callback(self, msg):
        self.teb_cmd = msg

    def odom_callback(self, msg):
        self.current_odom = msg

    def mb_status_callback(self, msg):
        if self.mode != "teb":
            return
        for status in msg.status_list:
            if status.status == 3 and self.teb_goal_sent:
                rospy.loginfo("TEB goal reached, switching back to pure_pursuit")
                self.switch_to_pp()
                break

    def scan_callback(self, msg):
        half_angle = math.radians(self.forward_angle / 2.0)
        min_dist = float('inf')

        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= half_angle:
                if r < min_dist:
                    min_dist = r

        self.min_front_dist = min_dist
        self.evaluate_switch()

    def evaluate_switch(self):
        if not self.pp_active:
            return

        now = rospy.Time.now()
        elapsed = (now - self.last_switch_time).to_sec()

        if elapsed < self.switch_cooldown:
            return

        if self.mode == "pure_pursuit":
            if self.min_front_dist < self.obstacle_dist_thresh:
                rospy.logwarn("Obstacle at %.2fm! Switching to TEB.", self.min_front_dist)
                self.switch_to_teb()
        else:
            if self.min_front_dist > self.clear_dist_thresh:
                rospy.loginfo("Path clear (%.2fm). Switching back to pure_pursuit.", self.min_front_dist)
                self.switch_to_pp()

    def switch_to_teb(self):
        self.mode = "teb"
        self.last_switch_time = rospy.Time.now()
        self.teb_goal_sent = False
        self.mode_pub.publish(String(data=self.mode))
        self.send_forward_goal()

    def switch_to_pp(self):
        self.mode = "pure_pursuit"
        self.last_switch_time = rospy.Time.now()
        self.teb_goal_sent = False
        self.mode_pub.publish(String(data=self.mode))

    def send_forward_goal(self):
        if self.current_odom is None:
            rospy.logwarn("No odom yet, cannot send TEB goal")
            return

        pos = self.current_odom.pose.pose.position
        ori = self.current_odom.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = pos.x + self.goal_forward_dist * math.cos(yaw)
        goal.pose.position.y = pos.y + self.goal_forward_dist * math.sin(yaw)
        goal.pose.position.z = 0.0
        q = tft.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.goal_pub.publish(goal)
        self.teb_goal_sent = True
        rospy.loginfo("Sent TEB goal: (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.mode == "pure_pursuit":
                self.cmd_pub.publish(self.pp_cmd)
            else:
                self.cmd_pub.publish(self.teb_cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        node = PlannerSwitch()
        node.run()
    except rospy.ROSInterruptException:
        pass
