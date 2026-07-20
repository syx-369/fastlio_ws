#!/usr/bin/env python3
"""Small ROS simulator for zone-aware static/moving-obstacle navigation.

It integrates differential-drive commands, publishes FAST-LIO-like odometry at
the front lidar pose, and publishes a synthetic circular obstacle in the lidar
frame.  It is deliberately simple and is only a software-flow test, not a
replacement for Gazebo or a hardware safety test.
"""

import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class StaticObstacleSimulator:
    def __init__(self):
        rospy.init_node("static_obstacle_simulator")
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
        self.obstacle_x = float(rospy.get_param("~obstacle_x", 3.5))
        self.obstacle_y = float(rospy.get_param("~obstacle_y", 0.0))
        self.obstacle_radius = float(rospy.get_param("~obstacle_radius", 0.35))
        configured_obstacles = rospy.get_param("~obstacles", [])
        self.obstacles = []
        for value in configured_obstacles if isinstance(configured_obstacles, list) else []:
            if not isinstance(value, dict):
                continue
            self.obstacles.append(
                (
                    float(value.get("x", 3.5)),
                    float(value.get("y", 0.0)),
                    float(value.get("radius", 0.35)),
                )
            )
        self.use_default_obstacle = bool(rospy.get_param("~use_default_obstacle", True))
        if not self.obstacles and self.use_default_obstacle:
            self.obstacles = [(self.obstacle_x, self.obstacle_y, self.obstacle_radius)]
        self.moving_obstacle = bool(rospy.get_param("~moving_obstacle", False))
        self.moving_obstacle_x = float(rospy.get_param("~moving_obstacle_x", 4.2))
        self.moving_obstacle_center_y = float(rospy.get_param("~moving_obstacle_center_y", 0.0))
        self.moving_obstacle_amplitude = float(rospy.get_param("~moving_obstacle_amplitude", 0.85))
        self.moving_obstacle_period = max(2.0, float(rospy.get_param("~moving_obstacle_period", 8.0)))
        self.moving_obstacle_radius = float(rospy.get_param("~moving_obstacle_radius", 0.28))
        self.moving_obstacle_motion = str(rospy.get_param("~moving_obstacle_motion", "oscillate")).strip().lower()
        self.moving_obstacle_start_delay = max(0.0, float(rospy.get_param("~moving_obstacle_start_delay", 0.0)))
        self.start_time = rospy.Time.now()
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        self.cmd_topic = rospy.get_param("~cmd_topic", "/smoother_cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered_body")

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
            self.yaw += self.angular * dt

    def publish_odometry(self, stamp: rospy.Time):
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        lidar_world_x = self.x + cosine * self.lidar_x - sine * self.lidar_y
        lidar_world_y = self.y + sine * self.lidar_x + cosine * self.lidar_y
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "camera_init"
        message.child_frame_id = "body"
        message.pose.pose.position.x = lidar_world_x
        message.pose.pose.position.y = lidar_world_y
        message.pose.pose.position.z = self.lidar_z
        message.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        message.twist.twist.linear.x = self.linear
        message.twist.twist.angular.z = self.angular
        self.odom_pub.publish(message)

    def publish_cloud(self, stamp: rospy.Time):
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        points = []
        obstacles = list(self.obstacles)
        if self.moving_obstacle:
            elapsed = (stamp - self.start_time).to_sec()
            if self.moving_obstacle_motion == "cross_once":
                ratio = max(
                    0.0,
                    min(1.0, (elapsed - self.moving_obstacle_start_delay) / self.moving_obstacle_period),
                )
                moving_y = self.moving_obstacle_center_y + self.moving_obstacle_amplitude * (2.0 * ratio - 1.0)
            else:
                moving_y = self.moving_obstacle_center_y + self.moving_obstacle_amplitude * math.sin(
                    2.0 * math.pi * elapsed / self.moving_obstacle_period
                )
            obstacles.append((self.moving_obstacle_x, moving_y, self.moving_obstacle_radius))
        for obstacle_x, obstacle_y, obstacle_radius in obstacles:
            for degree in range(0, 360, 4):
                angle = math.radians(degree)
                world_x = obstacle_x + obstacle_radius * math.cos(angle)
                world_y = obstacle_y + obstacle_radius * math.sin(angle)
                dx = world_x - self.x
                dy = world_y - self.y
                x_base = cosine * dx + sine * dy
                y_base = -sine * dx + cosine * dy
                x_lidar = x_base - self.lidar_x
                y_lidar = y_base - self.lidar_y
                for z_base in (0.15, 0.30, 0.50, 0.75):
                    points.append((x_lidar, y_lidar, z_base - self.lidar_z))
        header = Header(stamp=stamp, frame_id="body")
        self.cloud_pub.publish(pc2.create_cloud_xyz32(header, points))

    def run(self):
        rate_hz = 50.0
        rate = rospy.Rate(rate_hz)
        previous = rospy.Time.now()
        cloud_counter = 0
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = min(0.10, max(0.0, (now - previous).to_sec()))
            previous = now
            self.integrate(dt)
            self.publish_odometry(now)
            cloud_counter += 1
            if cloud_counter >= 5:
                cloud_counter = 0
                self.publish_cloud(now)
            rate.sleep()


if __name__ == "__main__":
    try:
        StaticObstacleSimulator().run()
    except rospy.ROSInterruptException:
        pass
