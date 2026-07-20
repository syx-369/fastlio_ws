#!/usr/bin/env python3
"""Closed-loop simulator for the independent hybrid_avoid package."""

import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def clamp_value(value, lower, upper):
    return max(lower, min(upper, value))


class HybridObstacleSimulator:
    def __init__(self):
        rospy.init_node("hybrid_obstacle_simulator")
        self.lock = threading.Lock()
        self.x = float(rospy.get_param("~initial_x", 0.0))
        self.y = float(rospy.get_param("~initial_y", 0.0))
        self.yaw = float(rospy.get_param("~initial_yaw", 0.0))
        self.linear = 0.0
        self.angular = 0.0
        self.last_command = rospy.Time.now()
        self.lidar_x = float(rospy.get_param("~lidar_x", 0.40))
        self.lidar_y = float(rospy.get_param("~lidar_y", 0.0))
        self.lidar_z = float(rospy.get_param("~lidar_z", 0.50))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        self.cmd_topic = str(rospy.get_param("~cmd_topic", "/smoother_cmd_vel"))
        self.odom_topic = str(rospy.get_param("~odom_topic", "/Odometry"))
        self.cloud_topic = str(rospy.get_param("~cloud_topic", "/cloud_registered_body"))
        self.cloud_blind_range = max(0.0, float(rospy.get_param("~cloud_blind_range", 0.0)))
        configured = rospy.get_param("~obstacles", [])
        self.obstacles = []
        for value in configured if isinstance(configured, list) else []:
            if isinstance(value, dict):
                self.obstacles.append(
                    (float(value.get("x", 3.0)), float(value.get("y", 0.0)), float(value.get("radius", 0.28)))
                )
        if not self.obstacles and bool(rospy.get_param("~use_default_obstacle", True)):
            self.obstacles = [(3.2, 0.0, 0.30)]
        self.moving_obstacle = bool(rospy.get_param("~moving_obstacle", False))
        self.moving_x = float(rospy.get_param("~moving_obstacle_x", 4.5))
        self.moving_center_y = float(rospy.get_param("~moving_obstacle_center_y", 0.0))
        self.moving_amplitude = float(rospy.get_param("~moving_obstacle_amplitude", 1.5))
        self.moving_period = max(2.0, float(rospy.get_param("~moving_obstacle_period", 8.0)))
        self.moving_radius = float(rospy.get_param("~moving_obstacle_radius", 0.28))
        self.moving_delay = max(0.0, float(rospy.get_param("~moving_obstacle_start_delay", 5.0)))
        self.start_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.cmd_topic, Twist, self.command_callback, queue_size=1)

    def command_callback(self, message: Twist):
        with self.lock:
            self.linear = float(message.linear.x)
            self.angular = float(message.angular.z)
            self.last_command = rospy.Time.now()

    def integrate(self, dt: float):
        with self.lock:
            if (rospy.Time.now() - self.last_command).to_sec() > self.command_timeout:
                self.linear = 0.0
                self.angular = 0.0
            self.x += self.linear * math.cos(self.yaw) * dt
            self.y += self.linear * math.sin(self.yaw) * dt
            self.yaw = math.atan2(math.sin(self.yaw + self.angular * dt), math.cos(self.yaw + self.angular * dt))

    def publish_odometry(self, stamp: rospy.Time):
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        message = Odometry()
        message.header = Header(stamp=stamp, frame_id="camera_init")
        message.child_frame_id = "body"
        message.pose.pose.position.x = self.x + cosine * self.lidar_x - sine * self.lidar_y
        message.pose.pose.position.y = self.y + sine * self.lidar_x + cosine * self.lidar_y
        message.pose.pose.position.z = self.lidar_z
        message.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        message.twist.twist.linear.x = self.linear
        message.twist.twist.angular.z = self.angular
        self.odom_pub.publish(message)

    def publish_cloud(self, stamp: rospy.Time):
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        points = []
        obstacles = list(self.obstacles)
        if self.moving_obstacle:
            elapsed = (stamp - self.start_time).to_sec()
            ratio = clamp_value((elapsed - self.moving_delay) / self.moving_period, 0.0, 1.0)
            moving_y = self.moving_center_y + self.moving_amplitude * (2.0 * ratio - 1.0)
            obstacles.append((self.moving_x, moving_y, self.moving_radius))
        for obstacle_x, obstacle_y, radius in obstacles:
            for degree in range(0, 360, 4):
                angle = math.radians(degree)
                world_x = obstacle_x + radius * math.cos(angle)
                world_y = obstacle_y + radius * math.sin(angle)
                dx, dy = world_x - self.x, world_y - self.y
                x_base = cosine * dx + sine * dy
                y_base = -sine * dx + cosine * dy
                x_lidar, y_lidar = x_base - self.lidar_x, y_base - self.lidar_y
                if math.hypot(x_lidar, y_lidar) < self.cloud_blind_range:
                    continue
                for z_base in (0.15, 0.30, 0.50, 0.75):
                    points.append((x_lidar, y_lidar, z_base - self.lidar_z))
        self.cloud_pub.publish(pc2.create_cloud_xyz32(Header(stamp=stamp, frame_id="body"), points))

    def run(self):
        rate = rospy.Rate(50.0)
        previous = rospy.Time.now()
        counter = 0
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = min(0.10, max(0.0, (now - previous).to_sec()))
            previous = now
            self.integrate(dt)
            self.publish_odometry(now)
            counter += 1
            if counter >= 5:
                counter = 0
                self.publish_cloud(now)
            rate.sleep()


if __name__ == "__main__":
    try:
        HybridObstacleSimulator().run()
    except rospy.ROSInterruptException:
        pass
