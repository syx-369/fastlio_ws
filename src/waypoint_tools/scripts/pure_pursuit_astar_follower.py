#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pure_pursuit_astar_follower.py

Bunker 履带车测试版：Pure Pursuit 路径跟踪 + 2D A* 局部重规划避障。

这个文件刻意不接入 delivery_mission：
- 不等待红旗
- 不识别红绿灯
- 不处理 CSV 里的 task
- 不等待外部任务完成

运行后只做一件事：
CSV 航点作为全局参考目标，点云/scan 生成局部障碍栅格，A* 滚动生成局部路径，
再用 Pure Pursuit 跟踪这段局部路径。
"""

import csv
import heapq
import math
import os
import threading
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from tf.transformations import euler_from_quaternion


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def base_xy_from_lidar_pose(x, y, yaw, lidar_x, lidar_y):
    """Convert a front sensor/IMU pose into the vehicle-centre position."""
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        x - cy * lidar_x + sy * lidar_y,
        y - sy * lidar_x - cy * lidar_y,
    )


def minimum_circular_inflation(
    footprint_front,
    footprint_rear,
    footprint_half_width,
    front_half_width,
    planner_clearance,
    safety_stop_dist,
    side_safety_radius,
):
    """Conservative lower bound for point-robot A* obstacle inflation."""
    swept_radius = math.hypot(
        max(footprint_front, footprint_rear), footprint_half_width
    )
    return max(
        swept_radius + planner_clearance,
        math.hypot(footprint_front + safety_stop_dist, front_half_width),
        math.hypot(
            max(footprint_front, footprint_rear),
            footprint_half_width + side_safety_radius,
        ),
    )


def point_to_segment_distance_sq(px, py, x0, y0, x1, y1):
    """Squared distance from a point to a finite 2-D line segment."""
    vx = x1 - x0
    vy = y1 - y0
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        dx = px - x0
        dy = py - y0
        return dx * dx + dy * dy
    t = ((px - x0) * vx + (py - y0) * vy) / length_sq
    t = clamp(t, 0.0, 1.0)
    dx = px - (x0 + t * vx)
    dy = py - (y0 + t * vy)
    return dx * dx + dy * dy


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def parse_float_list(value, default):
    if isinstance(value, str):
        result = []
        for item in value.split(","):
            item = item.strip()
            if not item:
                continue
            try:
                result.append(float(item))
            except ValueError:
                rospy.logwarn("Invalid float list item '%s', use default %s", item, default)
                return list(default)
        return result if result else list(default)
    if isinstance(value, (list, tuple)):
        return [float(v) for v in value]
    return list(default)


@dataclass
class Waypoint:
    seq: int
    x: float
    y: float
    yaw: float
    tol: float


class PurePursuitAStarFollower:
    def __init__(self):
        # ---------------- ROS topic ----------------
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/smoother_cmd_vel")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))

        # ---------------- Pure Pursuit control ----------------
        self.target_speed = float(rospy.get_param("~target_speed", 0.28))
        self.max_linear = float(rospy.get_param("~max_linear", 0.35))
        self.max_angular = float(rospy.get_param("~max_angular", 1.0))
        self.lookahead_distance = float(rospy.get_param("~lookahead_distance", 0.55))
        self.min_lookahead = float(rospy.get_param("~min_lookahead", 0.35))
        self.max_lookahead = float(rospy.get_param("~max_lookahead", 1.0))
        self.lookahead_speed_ratio = float(rospy.get_param("~lookahead_speed_ratio", 1.6))
        self.heading_slow_angle = float(rospy.get_param("~heading_slow_angle", 0.50))
        self.heading_hard_slow_angle = float(rospy.get_param("~heading_hard_slow_angle", 1.00))
        self.rotate_in_place_angle = float(rospy.get_param("~rotate_in_place_angle", 0.70))
        self.rotate_k_angular = float(rospy.get_param("~rotate_k_angular", 1.40))
        self.blocked_rotate_angle = float(rospy.get_param("~blocked_rotate_angle", 0.20))
        self.near_obstacle_angular = float(rospy.get_param("~near_obstacle_angular", 0.35))
        self.final_approach_dist = float(rospy.get_param("~final_approach_dist", 0.80))
        self.goal_tolerance_default = float(rospy.get_param("~goal_tolerance_default", 0.30))

        # ---------------- Obstacle source ----------------
        # cloud: 使用 FAST-LIO /cloud_registered，默认点云和 odom 在同一世界坐标系。
        # scan : 使用 2D LaserScan，默认 scan 在车体坐标系，代码用当前 yaw 转到世界系。
        # none : 不使用障碍，只验证 pure pursuit + A* 空地图流程。
        self.obstacle_source = rospy.get_param("~obstacle_source", "cloud").strip().lower()
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")

        # ---------------- Vehicle/reference geometry ----------------
        # FAST-LIO publishes the MID360/IMU pose.  On this Bunker the sensor is
        # mounted at the front centre, 0.40 m ahead of the body centre.  CSV
        # routes are recorded from the same /Odometry topic, so both odometry
        # and waypoints must be converted together before doing footprint math.
        self.vehicle_length = max(0.10, float(rospy.get_param("~vehicle_length", 0.80)))
        self.vehicle_width = max(0.10, float(rospy.get_param("~vehicle_width", 0.70)))
        self.lidar_x = float(rospy.get_param("~lidar_x", 0.40))
        self.lidar_y = float(rospy.get_param("~lidar_y", 0.0))
        self.lidar_z = float(rospy.get_param("~lidar_z", 0.50))
        # FAST-LIO odometry is tied to its IMU state origin. Keep this offset
        # separate from the physical lidar origin used by LaserScan projection;
        # they currently share the measured front-centre approximation but can
        # be calibrated independently without changing route geometry code.
        self.odom_origin_x = float(
            rospy.get_param("~odom_origin_x", self.lidar_x)
        )
        self.odom_origin_y = float(
            rospy.get_param("~odom_origin_y", self.lidar_y)
        )
        # Measured vertical distance from the FAST-LIO odometry origin (IMU)
        # to the floor in the saved map. It is not identical to lidar_z because
        # the MID360-to-IMU extrinsic and map datum both contribute.
        self.odom_ground_offset = max(
            0.05, float(rospy.get_param("~odom_ground_offset", 0.42))
        )
        self.odom_pose_is_lidar = parse_bool(rospy.get_param("~odom_pose_is_lidar", True))

        # Height is measured from the ground/body frame, not from the lidar.
        # This excludes the floor while retaining cones and boxes above 6 cm.
        self.min_obstacle_height = float(rospy.get_param("~min_obstacle_height", 0.06))
        self.max_obstacle_height = float(rospy.get_param("~max_obstacle_height", 1.40))
        self.cloud_keep_radius = float(rospy.get_param("~cloud_keep_radius", 12.0))
        self.cloud_stride = max(1, int(rospy.get_param("~cloud_stride", 1)))
        self.max_obstacle_points = max(100, int(rospy.get_param("~max_obstacle_points", 8000)))
        self.obstacle_voxel_size = max(
            0.03, float(rospy.get_param("~obstacle_voxel_size", 0.08))
        )
        self.obstacle_support_radius = max(
            self.obstacle_voxel_size,
            float(rospy.get_param("~obstacle_support_radius", 0.12)),
        )
        self.min_points_per_voxel = max(
            1, int(rospy.get_param("~min_points_per_voxel", 2))
        )
        self.scan_min_range = float(rospy.get_param("~scan_min_range", 0.05))
        self.scan_max_range = float(rospy.get_param("~scan_max_range", 8.0))
        self.obstacle_timeout = float(rospy.get_param("~obstacle_timeout", 1.0))
        # Ignore points inside the robot footprint. Registered point clouds often
        # contain returns from the chassis, cable guard, or near-ground self hits;
        # treating those as obstacles makes the safety layer stop forever.
        self.footprint_front = float(
            rospy.get_param("~footprint_front", 0.5 * self.vehicle_length)
        )
        self.footprint_rear = float(
            rospy.get_param("~footprint_rear", 0.5 * self.vehicle_length)
        )
        self.footprint_half_width = float(
            rospy.get_param("~footprint_half_width", 0.5 * self.vehicle_width)
        )
        # Self-return filtering is intentionally smaller than the conservative
        # collision footprint. Otherwise a real obstacle just outside the bare
        # chassis but inside the safety envelope becomes invisible.
        self.self_filter_front = max(
            0.0, float(rospy.get_param("~self_filter_front", self.footprint_front))
        )
        self.self_filter_rear = max(
            0.0, float(rospy.get_param("~self_filter_rear", self.footprint_rear))
        )
        self.self_filter_half_width = max(
            0.0,
            float(rospy.get_param("~self_filter_half_width", self.footprint_half_width)),
        )
        self.self_filter_padding = max(
            0.0, float(rospy.get_param("~self_filter_padding", 0.0))
        )

        # ---------------- Local A* planner ----------------
        self.planner_enabled = parse_bool(rospy.get_param("~planner_enabled", True))
        self.grid_resolution = float(rospy.get_param("~grid_resolution", 0.10))
        self.xy_margin = float(rospy.get_param("~xy_margin", 2.0))
        self.inflation_radius = float(rospy.get_param("~inflation_radius", 0.55))
        self.goal_clear_radius = float(rospy.get_param("~goal_clear_radius", 0.25))
        self.goal_search_radius = float(rospy.get_param("~goal_search_radius", 1.20))
        self.plan_goal_max_dist = float(rospy.get_param("~plan_goal_max_dist", 6.0))
        self.planning_goal_min_dist = float(rospy.get_param("~planning_goal_min_dist", 1.80))
        self.planning_goal_index_ahead = max(0, int(rospy.get_param("~planning_goal_index_ahead", 10)))
        self.planning_goal_min_forward = float(rospy.get_param("~planning_goal_min_forward", 0.40))
        self.skip_behind_waypoint_x = float(rospy.get_param("~skip_behind_waypoint_x", -0.25))
        self.skip_behind_waypoint_dist = float(rospy.get_param("~skip_behind_waypoint_dist", 1.00))
        self.min_valid_plan_dist = float(rospy.get_param("~min_valid_plan_dist", 0.60))
        self.min_valid_plan_points = max(1, int(rospy.get_param("~min_valid_plan_points", 2)))
        self.allow_behind_target_dist = float(rospy.get_param("~allow_behind_target_dist", 0.80))
        self.local_waypoint_spacing = float(rospy.get_param("~local_waypoint_spacing", 0.30))
        self.local_goal_tolerance = float(rospy.get_param("~local_goal_tolerance", 0.15))
        self.execute_points = max(1, int(rospy.get_param("~execute_points", 5)))
        self.max_grid_cells = max(10000, int(rospy.get_param("~max_grid_cells", 300000)))
        self.replan_min_interval = float(rospy.get_param("~replan_min_interval", 0.30))
        self.blocked_replan_delay = float(rospy.get_param("~blocked_replan_delay", 0.50))
        self.detour_lock_time = float(rospy.get_param("~detour_lock_time", 4.0))
        self.min_forward_target = float(rospy.get_param("~min_forward_target", -0.05))
        self.prefix_backtrack_tolerance = clamp(
            float(rospy.get_param("~prefix_backtrack_tolerance", 0.05)),
            0.0,
            0.05,
        )
        self.plan_accept_translation_tolerance = clamp(
            float(rospy.get_param("~plan_accept_translation_tolerance", 0.10)),
            0.02,
            0.10,
        )
        self.plan_accept_yaw_tolerance = clamp(
            float(rospy.get_param("~plan_accept_yaw_tolerance", 0.10)),
            0.02,
            0.10,
        )
        self.detour_forward_samples = parse_float_list(
            rospy.get_param("~detour_forward_samples", "1.2,2.0,3.0"),
            [1.2, 2.0, 3.0],
        )
        self.detour_lateral_offsets = parse_float_list(
            rospy.get_param("~detour_lateral_offsets", "0.6,0.9,1.2"),
            [0.6, 0.9, 1.2],
        )

        # ---------------- Reactive safety layer ----------------
        # A* 负责绕路；这一层负责防止突然出现障碍或点云延迟导致碰撞。
        self.front_half_width = float(rospy.get_param("~front_half_width", 0.28))
        self.safety_slow_dist = float(rospy.get_param("~safety_slow_dist", 0.60))
        self.safety_stop_dist = float(rospy.get_param("~safety_stop_dist", 0.28))
        self.safety_emergency_dist = float(rospy.get_param("~safety_emergency_dist", 0.16))
        self.side_safety_radius = float(rospy.get_param("~side_safety_radius", 0.18))
        self.side_stop_dist = max(
            0.0, float(rospy.get_param("~side_stop_dist", 0.08))
        )
        self.side_safety_radius = max(self.side_safety_radius, self.side_stop_dist)
        self.escape_angular = float(rospy.get_param("~escape_angular", 0.28))
        self.planner_clearance = max(
            0.0, float(rospy.get_param("~planner_clearance", 0.12))
        )
        self.rotation_stop_clearance = max(
            0.0, float(rospy.get_param("~rotation_stop_clearance", 0.08))
        )
        self.rotation_sweep_angle = max(
            0.05, float(rospy.get_param("~rotation_sweep_angle", 0.35))
        )
        self.rotation_sweep_step = max(
            0.02, float(rospy.get_param("~rotation_sweep_step", 0.05))
        )
        self.avoidance_max_speed = max(
            0.0, float(rospy.get_param("~avoidance_max_speed", self.max_linear))
        )

        # A* uses a circular configuration-space obstacle.  Keep that circle at
        # least as large as the body swept radius, the front-stop envelope and
        # the side envelope.  Grid rounding may enlarge it further.
        self.vehicle_swept_radius = math.hypot(
            max(self.footprint_front, self.footprint_rear),
            self.footprint_half_width,
        )
        required_inflation = minimum_circular_inflation(
            self.footprint_front,
            self.footprint_rear,
            self.footprint_half_width,
            self.front_half_width,
            self.planner_clearance,
            self.safety_stop_dist,
            self.side_safety_radius,
        )
        self.minimum_inflation_radius = required_inflation
        if self.inflation_radius + 1e-6 < required_inflation:
            rospy.logwarn(
                "Raise A* inflation %.2fm -> %.2fm to match vehicle/safety envelope.",
                self.inflation_radius,
                required_inflation,
            )
        self.inflation_radius = max(self.inflation_radius, required_inflation)

        # ---------------- CSV path ----------------
        self.csv_path = rospy.get_param("~csv_path", "")
        if self.csv_path.strip() == "":
            self.csv_path = self.find_latest_csv()
        if not os.path.isfile(self.csv_path):
            raise FileNotFoundError(f"CSV file not found: {self.csv_path}")

        self.global_waypoints = self.load_waypoints(self.csv_path)
        if not self.global_waypoints:
            raise RuntimeError("No waypoint found in csv.")
        if self.odom_pose_is_lidar:
            for waypoint in self.global_waypoints:
                waypoint.x, waypoint.y = base_xy_from_lidar_pose(
                    waypoint.x,
                    waypoint.y,
                    waypoint.yaw,
                    self.odom_origin_x,
                    self.odom_origin_y,
                )

        # ---------------- Runtime state ----------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_sensor_x = 0.0
        self.current_sensor_y = 0.0
        self.current_sensor_z = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.state = "WAIT_ODOM"  # WAIT_ODOM / TRACK / FINISH

        self.global_index = 0
        self.local_path: List[Tuple[float, float]] = []
        self.local_index = 0
        self.last_full_plan: List[Tuple[float, float]] = []
        self.last_plan_stamp = rospy.Time(0)
        self.blocked_since = None
        self.blocked_replan_pending = False
        self.last_blocked_replan_stamp = rospy.Time(0)
        self.detour_side_lock = 0  # +1 left, -1 right, 0 unlocked
        self.detour_lock_until = rospy.Time(0)

        self.obstacle_lock = threading.Lock()
        self.obstacles_xy: List[Tuple[float, float]] = []
        self.obstacle_stamp = rospy.Time(0)

        # ---------------- ROS pub/sub ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.full_path_pub = rospy.Publisher("~planned_path", Path, queue_size=1, latch=True)
        self.exec_path_pub = rospy.Publisher("~execute_path", Path, queue_size=1, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)

        if self.obstacle_source == "cloud":
            rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)
        elif self.obstacle_source == "scan":
            rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        elif self.obstacle_source == "none":
            rospy.logwarn("obstacle_source=none: A* will plan on an empty grid.")
        else:
            raise ValueError("~obstacle_source must be cloud, scan, or none")

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("==============================================")
        rospy.loginfo("Pure Pursuit + A* follower started.")
        rospy.loginfo("CSV file       : %s", self.csv_path)
        rospy.loginfo("Path points    : %d", len(self.global_waypoints))
        rospy.loginfo("Odom topic     : %s", self.odom_topic)
        rospy.loginfo("Cmd topic      : %s", self.cmd_topic)
        rospy.loginfo("Obstacle source: %s", self.obstacle_source)
        rospy.loginfo(
            "Vehicle geometry: %.2fx%.2fm lidar=(%.2f, %.2f, %.2f) "
            "odom_origin=(%.2f, %.2f) convert_odom=%s",
            self.vehicle_length,
            self.vehicle_width,
            self.lidar_x,
            self.lidar_y,
            self.lidar_z,
            self.odom_origin_x,
            self.odom_origin_y,
            self.odom_pose_is_lidar,
        )
        rospy.loginfo("Target speed   : %.2f m/s", self.target_speed)
        rospy.loginfo("Lookahead      : %.2f m", self.lookahead_distance)
        rospy.loginfo("Grid/inflate   : %.2f / %.2f m", self.grid_resolution, self.inflation_radius)
        rospy.loginfo("Body footprint : front=%.2f rear=%.2f half_width=%.2f",
                      self.footprint_front, self.footprint_rear, self.footprint_half_width)
        rospy.loginfo("Safety front   : width=%.2f slow=%.2f stop=%.2f emergency=%.2f",
                      self.front_half_width, self.safety_slow_dist,
                      self.safety_stop_dist, self.safety_emergency_dist)
        rospy.loginfo("Detour samples : forward=%s lateral=%s",
                      ",".join(f"{v:.1f}" for v in self.detour_forward_samples),
                      ",".join(f"{v:.1f}" for v in self.detour_lateral_offsets))
        rospy.loginfo("Execute points : %d", self.execute_points)
        rospy.loginfo("No flag, no traffic light, no task handling.")
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
        waypoints = []
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                # task 字段故意不读取；这个测试节点只验证路径跟踪和局部规划。
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
        q = pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_sensor_x = float(pose.position.x)
        self.current_sensor_y = float(pose.position.y)
        self.current_sensor_z = float(pose.position.z)
        if self.odom_pose_is_lidar:
            self.current_x, self.current_y = base_xy_from_lidar_pose(
                self.current_sensor_x,
                self.current_sensor_y,
                self.current_yaw,
                self.odom_origin_x,
                self.odom_origin_y,
            )
        else:
            self.current_x = self.current_sensor_x
            self.current_y = self.current_sensor_y
        # Keep z at the odometry sensor origin; ground-relative cloud filtering
        # uses the separately measured odom_ground_offset below.
        self.current_z = self.current_sensor_z

        if not self.has_odom:
            self.has_odom = True
            self.state = "TRACK"
            rospy.loginfo("First odom received. Start tracking directly.")

    # ================================================================
    # Obstacle callbacks
    # ================================================================
    def cloud_callback(self, msg: PointCloud2):
        if not self.has_odom:
            return

        voxel_bins = {}
        r2_keep = self.cloud_keep_radius * self.cloud_keep_radius
        ground_z = self.current_sensor_z - self.odom_ground_offset
        min_z = ground_z + self.min_obstacle_height
        max_z = ground_z + self.max_obstacle_height
        raw_count = 0
        height_count = 0
        candidate_count = 0

        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            raw_count += 1
            if raw_count % self.cloud_stride != 0:
                continue
            if z < min_z or z > max_z:
                continue
            height_count += 1
            dx = x - self.current_x
            dy = y - self.current_y
            if dx * dx + dy * dy > r2_keep:
                continue
            if self.inside_robot_self_filter(x, y):
                continue
            candidate_count += 1
            key = (
                int(math.floor(float(x) / self.obstacle_voxel_size)),
                int(math.floor(float(y) / self.obstacle_voxel_size)),
            )
            voxel_bins.setdefault(key, []).append((float(x), float(y)))

        points = []
        support_radius_sq = self.obstacle_support_radius * self.obstacle_support_radius
        neighbor_cells = max(
            1,
            int(math.ceil(self.obstacle_support_radius / self.obstacle_voxel_size)),
        )
        for key, voxel_points in voxel_bins.items():
            neighbor_points = []
            for key_y in range(key[1] - neighbor_cells, key[1] + neighbor_cells + 1):
                for key_x in range(key[0] - neighbor_cells, key[0] + neighbor_cells + 1):
                    neighbor_points.extend(voxel_bins.get((key_x, key_y), ()))

            voxel_supported = False
            for x, y in voxel_points:
                support_count = 0
                for other_x, other_y in neighbor_points:
                    dx = other_x - x
                    dy = other_y - y
                    if dx * dx + dy * dy <= support_radius_sq:
                        support_count += 1
                        if support_count >= self.min_points_per_voxel:
                            break
                if support_count < self.min_points_per_voxel:
                    continue
                voxel_supported = True
                break
            if voxel_supported:
                # Keep one deterministic geometric centre per supported voxel
                # and carry its half-diagonal as uncertainty in planning/safety.
                # This bounds CPU cost without letting a centroid move the
                # closest physical return outside the protected envelope.
                points.append(
                    (
                        (key[0] + 0.5) * self.obstacle_voxel_size,
                        (key[1] + 0.5) * self.obstacle_voxel_size,
                    )
                )

        # If the cap is reached, retain the voxels closest to the collision
        # footprint; input point ordering must never evict a nearby obstacle.
        points.sort(key=lambda point: self.point_footprint_clearance_sq(*point))
        points = points[: self.max_obstacle_points]

        with self.obstacle_lock:
            self.obstacles_xy = points
            self.obstacle_stamp = rospy.Time.now()

        rospy.loginfo_throttle(
            2.0,
            "Cloud obstacle filter: raw=%d height=%d candidates=%d supported=%d "
            "z_window=[%.2f, %.2f] sensor_z=%.2f",
            raw_count,
            height_count,
            candidate_count,
            len(points),
            min_z,
            max_z,
            self.current_sensor_z,
        )

    def scan_callback(self, msg: LaserScan):
        if not self.has_odom:
            return

        points = []
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        if self.odom_pose_is_lidar:
            scan_x = self.current_x + cy * self.lidar_x - sy * self.lidar_y
            scan_y = self.current_y + sy * self.lidar_x + cy * self.lidar_y
        else:
            scan_x = self.current_x
            scan_y = self.current_y
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and self.scan_min_range <= r <= self.scan_max_range:
                yaw = self.current_yaw + angle
                x = scan_x + r * math.cos(yaw)
                y = scan_y + r * math.sin(yaw)
                if self.inside_robot_self_filter(x, y):
                    angle += msg.angle_increment
                    continue
                points.append((x, y))
            angle += msg.angle_increment

        with self.obstacle_lock:
            self.obstacles_xy = points[: self.max_obstacle_points]
            self.obstacle_stamp = rospy.Time.now()

    def get_obstacles_snapshot(self):
        if self.obstacle_source == "none":
            return []

        return self.get_cached_obstacles_snapshot()

    def get_cached_obstacles_snapshot(self):
        """Return the raw cached obstacle set without avoidance-zone gating."""

        with self.obstacle_lock:
            obs = list(self.obstacles_xy)
            stamp = self.obstacle_stamp

        if stamp != rospy.Time(0) and (rospy.Time.now() - stamp).to_sec() > self.obstacle_timeout:
            rospy.logwarn_throttle(2.0, "Obstacle data is stale; using last obstacle snapshot.")
        return obs

    def obstacle_data_fresh(self):
        if self.obstacle_source == "none":
            return True
        with self.obstacle_lock:
            stamp = self.obstacle_stamp
        if stamp == rospy.Time(0):
            return False
        return (rospy.Time.now() - stamp).to_sec() <= self.obstacle_timeout

    def obstacle_data_required_now(self):
        if self.obstacle_source == "none":
            return False
        if bool(getattr(self, "zone_only", False)):
            return bool(getattr(self, "zone_active", False))
        return True

    def inside_robot_footprint(self, x, y, padding=0.0):
        """Return True when a point is probably on/inside the Bunker body."""
        rx = x - self.current_x
        ry = y - self.current_y
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        xb = cy * rx + sy * ry
        yb = -sy * rx + cy * ry
        pad = max(0.0, float(padding))
        return (
            -self.footprint_rear - pad <= xb <= self.footprint_front + pad
            and abs(yb) <= self.footprint_half_width + pad
        )

    def inside_robot_self_filter(self, x, y):
        """Return True only for the smaller physical self-return rectangle."""
        rx = x - self.current_x
        ry = y - self.current_y
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        xb = cy * rx + sy * ry
        yb = -sy * rx + cy * ry
        pad = self.self_filter_padding
        return (
            -self.self_filter_rear - pad <= xb <= self.self_filter_front + pad
            and abs(yb) <= self.self_filter_half_width + pad
        )

    def obstacle_uncertainty_radius(self):
        """Radius represented by one cached obstacle point."""
        if getattr(self, "obstacle_source", "none") != "cloud":
            return 0.0
        return 0.5 * math.sqrt(2.0) * self.obstacle_voxel_size

    def point_footprint_clearance_sq(self, x, y):
        """Squared distance from a world point to the collision rectangle."""
        xb, yb = self.point_in_body(x, y)
        if xb < -self.footprint_rear:
            dx = -self.footprint_rear - xb
        elif xb > self.footprint_front:
            dx = xb - self.footprint_front
        else:
            dx = 0.0
        dy = max(0.0, abs(yb) - self.footprint_half_width)
        return dx * dx + dy * dy

    # ================================================================
    # Local A* planner
    # ================================================================
    def clipped_goal(self, wp: Waypoint):
        """局部规划只规划一定距离；远处 CSV 点会被裁剪成局部目标。"""
        dx = wp.x - self.current_x
        dy = wp.y - self.current_y
        dist = math.hypot(dx, dy)
        if dist <= self.plan_goal_max_dist or dist < 1e-6:
            return wp.x, wp.y
        scale = self.plan_goal_max_dist / dist
        return self.current_x + dx * scale, self.current_y + dy * scale

    def obstacle_side_scores(self):
        """Return weighted obstacle clutter on the left and right front side."""
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        left_score = 0.0
        right_score = 0.0
        for ox, oy in self.get_obstacles_snapshot():
            rx = ox - self.current_x
            ry = oy - self.current_y
            xb = cy * rx + sy * ry
            yb = -sy * rx + cy * ry
            if xb < 0.0 or xb > self.safety_slow_dist + self.footprint_front:
                continue
            if abs(yb) > max(0.8, self.front_half_width * 3.0):
                continue
            weight = 1.0 / max(0.15, math.hypot(xb, yb))
            if yb > 0.05:
                left_score += weight
            elif yb < -0.05:
                right_score += weight
            else:
                # A point on the centreline obstructs both choices equally.
                left_score += weight
                right_score += weight
        return left_score, right_score

    def preferred_detour_side(self):
        """Choose +1 left / -1 right and preserve one recovery direction."""
        now = rospy.Time.now()
        if self.detour_side_lock and (
            self.blocked_since is not None or now < self.detour_lock_until
        ):
            return self.detour_side_lock
        left_score, right_score = self.obstacle_side_scores()
        return 1 if left_score <= right_score else -1

    def local_goal_candidates(self, wp: Waypoint):
        """Generate local A* targets.

        The direct clipped CSV target is enough in open space. When a waypoint or
        the line toward it is blocked, side candidates give the planner something
        reachable beside the obstacle, so the tracked vehicle can rotate and pass
        around instead of stopping in front of the wall.
        """
        direct_x, direct_y = self.clipped_goal(wp)
        dx = wp.x - self.current_x
        dy = wp.y - self.current_y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return [(direct_x, direct_y, "direct")]

        ux = dx / norm
        uy = dy / norm
        px = -uy
        py = ux
        max_forward = min(self.plan_goal_max_dist, norm)

        side_candidates = []
        preferred_side = self.preferred_detour_side()
        if preferred_side > 0:
            side_order = ((1.0, "left"), (-1.0, "right"))
        else:
            side_order = ((-1.0, "right"), (1.0, "left"))

        for forward in self.detour_forward_samples:
            fwd = min(forward, max_forward)
            if fwd < 0.25:
                continue
            for lateral in self.detour_lateral_offsets:
                for sign, name in side_order:
                    x = self.current_x + ux * fwd + px * lateral * sign
                    y = self.current_y + uy * fwd + py * lateral * sign
                    side_candidates.append((x, y, f"{name}_{fwd:.1f}_{lateral:.1f}"))

        if self.blocked_since is not None:
            return side_candidates + [(direct_x, direct_y, "direct")]
        return [(direct_x, direct_y, "direct")] + side_candidates

    def build_inflation_offsets(self, cells):
        offsets = []
        for iy in range(-cells, cells + 1):
            for ix in range(-cells, cells + 1):
                if ix * ix + iy * iy <= cells * cells:
                    offsets.append((ix, iy))
        return offsets

    def nearest_free_goal_cell(self, goal_x, goal_y, w, h, occ, to_ix, to_iy, cell_x, cell_y, inside):
        """Pick a free grid cell near the requested goal.

        Dynamic obstacles may sit exactly on the local goal or on a CSV waypoint.
        Clearing the goal would make A* plan into the obstacle. Instead, search
        a small neighborhood and use the nearest free cell as a temporary target.
        """
        gx = to_ix(goal_x)
        gy = to_iy(goal_y)
        if not inside(gx, gy):
            return None
        if not occ[gy * w + gx]:
            return gx, gy

        max_r = max(1, int(math.ceil(self.goal_search_radius / max(0.05, self.grid_resolution))))
        best = None
        best_score = float("inf")
        for r in range(1, max_r + 1):
            found_this_ring = False
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    ix = gx + dx
                    iy = gy + dy
                    if not inside(ix, iy):
                        continue
                    if occ[iy * w + ix]:
                        continue
                    wx = cell_x(ix)
                    wy = cell_y(iy)
                    score = math.hypot(wx - goal_x, wy - goal_y)
                    if score < best_score:
                        best_score = score
                        best = (ix, iy)
                        found_this_ring = True
            if found_this_ring:
                break
        if best is not None:
            rospy.logwarn_throttle(
                1.0,
                "A* local goal occupied, use nearby free cell offset %.2fm.",
                best_score,
            )
        return best

    def line_of_sight(self, x0, y0, x1, y1, w, occ):
        """Bresenham 直线检测，用于把 A* 锯齿路径拉直。"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if occ[y0 * w + x0]:
                return False
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    @staticmethod
    def path_has_obstacle_clearance(start_xy, points, obstacles, clearance):
        """Validate the continuous smoothed path against measured points.

        Raster occupancy is necessarily quantized. This final exact-coordinate
        check prevents a line-of-sight shortcut from cutting inside the
        configuration-space clearance even when every sampled grid cell is free.
        """
        if not points or not obstacles:
            return True
        clearance_sq = max(0.0, float(clearance)) ** 2
        x0, y0 = start_xy
        for x1, y1 in points:
            for ox, oy in obstacles:
                if point_to_segment_distance_sq(ox, oy, x0, y0, x1, y1) < clearance_sq:
                    return False
            x0, y0 = x1, y1
        return True

    def astar_plan(
        self,
        goal_x,
        goal_y,
        xy_margin=None,
        inflation_radius=None,
        start_xy=None,
    ):
        if start_xy is None:
            sx_world = self.current_x
            sy_world = self.current_y
        else:
            sx_world = float(start_xy[0])
            sy_world = float(start_xy[1])
        res = max(0.05, self.grid_resolution)
        margin = max(0.5, self.xy_margin if xy_margin is None else xy_margin)

        min_x = min(sx_world, goal_x) - margin
        max_x = max(sx_world, goal_x) + margin
        min_y = min(sy_world, goal_y) - margin
        max_y = max(sy_world, goal_y) + margin
        w = int(math.ceil((max_x - min_x) / res)) + 1
        h = int(math.ceil((max_y - min_y) / res)) + 1
        if w <= 2 or h <= 2 or w * h > self.max_grid_cells:
            rospy.logwarn("A* grid invalid/too large: w=%d h=%d cells=%d", w, h, w * h)
            return []

        def to_ix(x):
            return int(round((x - min_x) / res))

        def to_iy(y):
            return int(round((y - min_y) / res))

        def cell_x(ix):
            return min_x + ix * res

        def cell_y(iy):
            return min_y + iy * res

        def inside(ix, iy):
            return 0 <= ix < w and 0 <= iy < h

        def index(ix, iy):
            return iy * w + ix

        occ = bytearray(w * h)
        requested_inflation = self.inflation_radius if inflation_radius is None else inflation_radius
        inflate = max(requested_inflation, self.minimum_inflation_radius)
        # The obstacle is rounded to its nearest grid cell. Add half a cell
        # diagonal while rasterizing so that this rounding can never reduce the
        # requested physical clearance. The final polyline is also checked
        # against the original (unrounded) points below.
        obstacle_uncertainty = self.obstacle_uncertainty_radius()
        protected_inflate = inflate + obstacle_uncertainty
        raster_inflate = protected_inflate + 0.5 * math.sqrt(2.0) * res
        inflate_cells = int(math.ceil(raster_inflate / res))
        inflate_offsets = self.build_inflation_offsets(inflate_cells)
        obstacles = self.get_obstacles_snapshot()

        # 点云/scan 投影到 2D 栅格，并做膨胀。膨胀半径约等于车体半宽 + 安全距离。
        for ox, oy in obstacles:
            if (
                ox < min_x - raster_inflate
                or ox > max_x + raster_inflate
                or oy < min_y - raster_inflate
                or oy > max_y + raster_inflate
            ):
                continue
            cx = to_ix(ox)
            cy = to_iy(oy)
            for dx, dy in inflate_offsets:
                ix = cx + dx
                iy = cy + dy
                if inside(ix, iy):
                    occ[index(ix, iy)] = 1

        sx = to_ix(sx_world)
        sy = to_iy(sy_world)
        if not inside(sx, sy):
            return []

        # Occupancy cells represent candidate *vehicle-centre* positions after
        # obstacle inflation. Clearing a whole body-shaped area here would erase
        # valid configuration-space obstacles. Only the exact current state is
        # released so A* can attempt to leave a marginal start pose.
        occ[index(sx, sy)] = 0

        free_goal = self.nearest_free_goal_cell(goal_x, goal_y, w, h, occ, to_ix, to_iy, cell_x, cell_y, inside)
        if free_goal is None:
            rospy.logwarn_throttle(1.0, "A* failed: no free cell near local goal.")
            return []
        gx, gy = free_goal

        start = index(sx, sy)
        finish = index(gx, gy)
        g_score = [float("inf")] * (w * h)
        parent = [-1] * (w * h)
        open_heap = []

        def heuristic(ix, iy):
            return math.hypot(cell_x(ix) - goal_x, cell_y(iy) - goal_y)

        g_score[start] = 0.0
        heapq.heappush(open_heap, (heuristic(sx, sy), start))

        dirs = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )

        while open_heap and not rospy.is_shutdown():
            _, cur = heapq.heappop(open_heap)
            if cur == finish:
                break

            ix = cur % w
            iy = cur // w
            for dx, dy in dirs:
                nx = ix + dx
                ny = iy + dy
                if not inside(nx, ny):
                    continue
                ni = index(nx, ny)
                if occ[ni]:
                    continue
                # 禁止斜向穿越两个障碍格子的夹角，减少贴边擦碰。
                if dx != 0 and dy != 0:
                    if occ[index(ix + dx, iy)] or occ[index(ix, iy + dy)]:
                        continue
                step = res * math.sqrt(2.0) if dx != 0 and dy != 0 else res
                tentative = g_score[cur] + step
                if tentative >= g_score[ni]:
                    continue
                parent[ni] = cur
                g_score[ni] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(nx, ny), ni))

        if parent[finish] < 0 and finish != start:
            rospy.logwarn_throttle(1.0, "A* failed: no path to local goal.")
            return []

        path_cells = []
        at = finish
        while at >= 0:
            path_cells.append(at)
            if at == start:
                break
            at = parent[at]
        if not path_cells or path_cells[-1] != start:
            return []
        path_cells.reverse()

        # 路径平滑：保留必要转折点，去掉 8 邻域搜索造成的锯齿。
        turns = [path_cells[0]]
        anchor = 0
        for i in range(1, len(path_cells)):
            ax = path_cells[anchor] % w
            ay = path_cells[anchor] // w
            cx = path_cells[i] % w
            cy = path_cells[i] // w
            if not self.line_of_sight(ax, ay, cx, cy, w, occ):
                turns.append(path_cells[i - 1])
                anchor = i - 1
        turns.append(path_cells[-1])

        # 将平滑后的折线按固定距离细分，给 Pure Pursuit 足够连续的局部路径点。
        out = []
        max_seg = max(res, self.local_waypoint_spacing)
        for i in range(1, len(turns)):
            x0 = cell_x(turns[i - 1] % w)
            y0 = cell_y(turns[i - 1] // w)
            x1 = cell_x(turns[i] % w)
            y1 = cell_y(turns[i] // w)
            seg = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(math.ceil(seg / max_seg)))
            for s in range(1, steps + 1):
                t = float(s) / steps
                out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))

        if not self.path_has_obstacle_clearance(
            (sx_world, sy_world), out, obstacles, protected_inflate
        ):
            rospy.logerr_throttle(
                1.0,
                "A* reject path: smoothed segment violates %.2fm obstacle clearance.",
                protected_inflate,
            )
            return []

        rospy.loginfo(
            "A* plan: cells=%d turns=%d points=%d obstacles=%d margin=%.2f "
            "inflate=%.2f raster=%.2f goal=(%.2f, %.2f)",
            len(path_cells), len(turns), len(out), len(obstacles), margin,
            inflate, raster_inflate, goal_x, goal_y,
        )
        return out

    def publish_path(self, publisher, points: Sequence[Tuple[float, float]]):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        publisher.publish(msg)

    def replan_to_global_waypoint(self, wp: Waypoint):
        if not self.planner_enabled:
            self.last_full_plan = [(wp.x, wp.y)]
            self.local_path = [(wp.x, wp.y)]
            self.local_index = 0
            self.last_plan_stamp = rospy.Time.now()
            self.publish_path(self.full_path_pub, self.last_full_plan)
            self.publish_path(self.exec_path_pub, self.local_path)
            return True

        attempts = (
            (self.xy_margin, self.inflation_radius),
            (self.xy_margin * 1.6, self.inflation_radius),
            (self.xy_margin * 2.2, self.inflation_radius),
        )

        # Odom can update on another ROS callback while A* runs. Validate every
        # candidate against the same pose snapshot used to start this replan.
        plan_x = self.current_x
        plan_y = self.current_y
        plan_yaw = self.current_yaw
        plan_cy = math.cos(plan_yaw)
        plan_sy = math.sin(plan_yaw)

        def body_x_at_plan_start(x, y):
            rx = x - plan_x
            ry = y - plan_y
            return plan_cy * rx + plan_sy * ry

        full_plan = []
        chosen_name = ""
        for goal_x, goal_y, goal_name in self.local_goal_candidates(wp):
            for margin, inflation in attempts:
                candidate_plan = self.astar_plan(
                    goal_x,
                    goal_y,
                    xy_margin=margin,
                    inflation_radius=inflation,
                    start_xy=(plan_x, plan_y),
                )
                if not candidate_plan:
                    continue

                accept_x = self.current_x
                accept_y = self.current_y
                accept_yaw = self.current_yaw
                pose_drift = math.hypot(accept_x - plan_x, accept_y - plan_y)
                yaw_drift = abs(wrap_to_pi(accept_yaw - plan_yaw))
                if (
                    pose_drift > self.plan_accept_translation_tolerance
                    or yaw_drift > self.plan_accept_yaw_tolerance
                ):
                    rospy.logwarn_throttle(
                        1.0,
                        "A* result stale while planning: pose drift=%.3fm/%.3fm "
                        "yaw drift=%.3frad/%.3frad; stop and replan from fresh odom.",
                        pose_drift,
                        self.plan_accept_translation_tolerance,
                        yaw_drift,
                        self.plan_accept_yaw_tolerance,
                    )
                    return False

                end_x, end_y = candidate_plan[-1]
                end_dist = math.hypot(end_x - plan_x, end_y - plan_y)
                end_xb = body_x_at_plan_start(end_x, end_y)
                execute_prefix = candidate_plan[: self.execute_points]
                prefix_min_xb = min(
                    body_x_at_plan_start(x, y) for x, y in execute_prefix
                )
                accept_cy = math.cos(accept_yaw)
                accept_sy = math.sin(accept_yaw)
                prefix_min_current_xb = min(
                    accept_cy * (x - accept_x) + accept_sy * (y - accept_y)
                    for x, y in execute_prefix
                )
                prefix_tolerance = min(
                    self.prefix_backtrack_tolerance,
                    0.5 * max(0.01, self.grid_resolution),
                )
                prefix_min_allowed = -prefix_tolerance
                is_final_goal = self.global_index >= len(self.global_waypoints) - 1
                unsafe_prefix = (
                    prefix_min_xb < prefix_min_allowed
                    or prefix_min_current_xb < prefix_min_allowed
                )
                invalid_nonfinal = not is_final_goal and (
                    len(candidate_plan) < self.min_valid_plan_points
                    or end_dist < self.min_valid_plan_dist
                    or end_xb < self.min_forward_target
                )
                if unsafe_prefix or invalid_nonfinal:
                    rospy.logwarn_throttle(
                        1.0,
                        "A* reject unsafe/short plan: points=%d dist=%.2f "
                        "end_xb=%.2f prefix_min_xb=%.2f current_min_xb=%.2f "
                        "goal=%s",
                        len(candidate_plan), end_dist, end_xb,
                        prefix_min_xb, prefix_min_current_xb, goal_name,
                    )
                    continue

                full_plan = candidate_plan
                chosen_name = goal_name
                if full_plan:
                    break
            if full_plan:
                break
        if not full_plan:
            # Planning is transactional: a failed refresh must not erase the
            # last accepted path and turn the next cycle into a fake "behind"
            # event. The safety branch will keep the vehicle stopped meanwhile.
            rospy.logwarn_throttle(1.0, "A* refresh failed; keep last accepted local path.")
            return False

        # 滚动重规划：规划一整段，但只执行前几个点，走完再从当前位置重新规划。
        if chosen_name and chosen_name != "direct":
            rospy.logwarn_throttle(1.0, "A* use detour local goal: %s", chosen_name)
            now = rospy.Time.now()
            chosen_side = 0
            if chosen_name.startswith("left"):
                chosen_side = 1
            elif chosen_name.startswith("right"):
                chosen_side = -1
            if chosen_side and (
                chosen_side != self.detour_side_lock
                or (
                    now >= self.detour_lock_until
                    and self.blocked_since is None
                )
            ):
                self.detour_side_lock = chosen_side
                self.detour_lock_until = now + rospy.Duration(self.detour_lock_time)
        elif (
            self.blocked_since is None
            and rospy.Time.now() >= self.detour_lock_until
        ):
            self.detour_side_lock = 0
        self.last_full_plan = full_plan
        self.local_path = list(full_plan[: self.execute_points])
        self.local_index = 0
        self.last_plan_stamp = rospy.Time.now()
        self.publish_path(self.full_path_pub, self.last_full_plan)
        self.publish_path(self.exec_path_pub, self.local_path)
        return True

    # ================================================================
    # Pure Pursuit helpers
    # ================================================================
    def compute_adaptive_lookahead(self, speed):
        ld = self.lookahead_speed_ratio * abs(speed)
        ld = max(ld, self.lookahead_distance)
        return clamp(ld, self.min_lookahead, self.max_lookahead)

    def update_local_index(self):
        if not self.local_path:
            return
        search_end = min(len(self.local_path), self.local_index + 8)
        best_i = self.local_index
        best_d = float("inf")
        for i in range(self.local_index, search_end):
            x, y = self.local_path[i]
            xb, _ = self.point_in_body(x, y)
            if xb < self.min_forward_target and i < len(self.local_path) - 1:
                continue
            d = math.hypot(x - self.current_x, y - self.current_y)
            if d < best_d:
                best_d = d
                best_i = i
        self.local_index = best_i

    def local_segment_done(self):
        if not self.local_path:
            return True
        end_x, end_y = self.local_path[-1]
        return math.hypot(end_x - self.current_x, end_y - self.current_y) <= self.local_goal_tolerance

    def find_lookahead_point(self, lookahead):
        self.update_local_index()
        if not self.local_path:
            return None
        for i in range(self.local_index, len(self.local_path)):
            x, y = self.local_path[i]
            xb, _ = self.point_in_body(x, y)
            if xb < self.min_forward_target and i < len(self.local_path) - 1:
                continue
            if math.hypot(x - self.current_x, y - self.current_y) >= lookahead:
                return x, y
        last_x, last_y = self.local_path[-1]
        last_xb, _ = self.point_in_body(last_x, last_y)
        if last_xb < self.min_forward_target:
            last_dist = math.hypot(last_x - self.current_x, last_y - self.current_y)
            if not self.planner_enabled and last_dist >= self.allow_behind_target_dist:
                rospy.logwarn_throttle(
                    1.0,
                    "Non-planner target is behind but far enough: rotate toward it. "
                    "xb=%.2f dist=%.2f",
                    last_xb, last_dist,
                )
                return last_x, last_y
            return None
        return last_x, last_y

    def angular_cmd_from_curvature(self, curvature, speed):
        yaw_rate = speed * curvature
        return clamp(yaw_rate, -self.max_angular, self.max_angular)

    def point_in_body(self, x, y):
        rx = x - self.current_x
        ry = y - self.current_y
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        xb = cy * rx + sy * ry
        yb = -sy * rx + cy * ry
        return xb, yb

    # ================================================================
    # Reactive safety layer
    # ================================================================
    def safety_distances(self):
        obstacles = self.get_obstacles_snapshot()
        if not obstacles:
            return float("inf"), float("inf")

        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        min_front = float("inf")
        min_side_clear = float("inf")
        front_start = max(0.0, self.footprint_front)
        point_radius = self.obstacle_uncertainty_radius()

        for ox, oy in obstacles:
            rx = ox - self.current_x
            ry = oy - self.current_y
            xf = cy * rx + sy * ry
            yf = -sy * rx + cy * ry
            # Self returns were already removed with the smaller physical-body
            # filter. Any accepted point now inside the conservative collision
            # footprint is an intrusion, not a negative clearance to ignore.
            if (
                -self.footprint_rear - point_radius
                <= xf
                <= self.footprint_front + point_radius
                and abs(yf) <= self.footprint_half_width + point_radius
            ):
                min_front = 0.0
                min_side_clear = 0.0
                continue
            if (
                -self.footprint_rear - point_radius
                <= xf
                <= self.footprint_front + point_radius
            ):
                side_clear = abs(yf) - self.footprint_half_width - point_radius
                if side_clear >= 0.0:
                    min_side_clear = min(min_side_clear, side_clear)
            # Front safety is measured as clear distance from the front bumper,
            # not from the vehicle center. This avoids false stops when a side
            # wall is close to the middle of the chassis but not blocking travel.
            front_clear = xf - front_start - point_radius
            if (
                0.0 <= front_clear < self.safety_slow_dist
                and abs(yf) <= self.front_half_width + point_radius
            ):
                min_front = min(min_front, front_clear)
        return min_front, min_side_clear

    def blocked_replan_due(self):
        now = rospy.Time.now()
        if self.blocked_since is None:
            self.blocked_since = now
            return False
        return (now - self.blocked_since).to_sec() >= self.blocked_replan_delay

    def latch_blocked_replan(self, front_clear):
        """Latch zero-translation recovery on the very first stop observation."""
        if front_clear < self.safety_stop_dist and self.blocked_since is None:
            self.blocked_since = rospy.Time.now()
            self.blocked_replan_pending = True

    def safety_hard_stop_reason(self, front_clear, side_clear):
        if front_clear <= 0.0:
            return "footprint_intrusion"
        if side_clear <= self.side_stop_dist:
            return "side_clearance"
        return None

    def clear_blocked_state(self):
        self.blocked_since = None

    # ================================================================
    # Main control loop
    # ================================================================
    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def publish_safety_cmd(self, angular_z):
        """Publish an immediate stop requested by the reactive safety layer.

        Zone-aware subclasses may smooth ordinary heading corrections, but a
        real obstacle stop must remain distinguishable and take effect now.
        """
        self.publish_cmd(0.0, angular_z)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def advance_global_waypoint_if_needed(self):
        while self.global_index < len(self.global_waypoints):
            wp = self.global_waypoints[self.global_index]
            d = math.hypot(wp.x - self.current_x, wp.y - self.current_y)
            xb, _ = self.point_in_body(wp.x, wp.y)
            if d > wp.tol:
                if xb < self.skip_behind_waypoint_x and d < self.skip_behind_waypoint_dist:
                    rospy.logwarn(
                        "Skip behind CSV waypoint %d | d=%.2f xb=%.2f",
                        wp.seq, d, xb,
                    )
                    self.global_index += 1
                    self.local_path = []
                    self.local_index = 0
                    self.detour_side_lock = 0
                    self.detour_lock_until = rospy.Time(0)
                    continue
                return
            rospy.loginfo("Pass CSV waypoint %d | d=%.2f", wp.seq, d)
            self.global_index += 1
            self.local_path = []
            self.local_index = 0
            self.detour_side_lock = 0
            self.detour_lock_until = rospy.Time(0)

    def select_planning_waypoint(self):
        """Choose a waypoint ahead of the current pass/check waypoint.

        The current CSV waypoint can be very close to the robot or exactly behind
        a box. Planning only to that near point produces a tiny A* segment, so
        the safety layer stops before the vehicle has enough room to detour.
        """
        if self.global_index >= len(self.global_waypoints):
            return None

        best = None
        fallback = self.global_waypoints[self.global_index]
        last_i = min(
            len(self.global_waypoints) - 1,
            self.global_index + self.planning_goal_index_ahead,
        )
        for i in range(self.global_index, last_i + 1):
            wp = self.global_waypoints[i]
            d = math.hypot(wp.x - self.current_x, wp.y - self.current_y)
            xb, _ = self.point_in_body(wp.x, wp.y)
            if xb < self.planning_goal_min_forward and i < len(self.global_waypoints) - 1:
                continue
            best = wp
            if d >= self.planning_goal_min_dist:
                return wp
            if d >= self.plan_goal_max_dist:
                return wp
        if best is None:
            for i in range(last_i + 1, len(self.global_waypoints)):
                wp = self.global_waypoints[i]
                xb, _ = self.point_in_body(wp.x, wp.y)
                if xb >= self.planning_goal_min_forward or i == len(self.global_waypoints) - 1:
                    rospy.logwarn_throttle(
                        1.0,
                        "No forward CSV waypoint nearby; jump planning target to waypoint %d.",
                        wp.seq,
                    )
                    return wp
            rospy.logwarn_throttle(1.0, "No forward CSV waypoint available; use current waypoint fallback.")
            return fallback
        return best

    def preferred_escape_turn(self, alpha):
        """Pick a slow in-place turn direction when the front is blocked."""
        now = rospy.Time.now()
        if self.detour_side_lock and now < self.detour_lock_until:
            return self.detour_side_lock * self.escape_angular

        # Establish one committed direction before rotating.  A changing Pure
        # Pursuit alpha must never override this lock during the attempt.
        side = self.preferred_detour_side()
        self.detour_side_lock = side
        self.detour_lock_until = now + rospy.Duration(self.detour_lock_time)
        return side * self.escape_angular

    def rotation_sweep_is_clear(self, angular_z, obstacles=None):
        """Check the near-future rectangular footprint sweep for an in-place turn."""
        if abs(angular_z) < 1e-6:
            return False
        direction = 1.0 if angular_z > 0.0 else -1.0
        sweep_angle = self.rotation_sweep_angle
        sweep_step = self.rotation_sweep_step
        margin = self.rotation_stop_clearance + self.obstacle_uncertainty_radius()
        if obstacles is None:
            obstacles = self.get_obstacles_snapshot()
        sample_count = max(1, int(math.ceil(sweep_angle / sweep_step)))
        for sample in range(sample_count + 1):
            yaw = self.current_yaw + direction * sweep_angle * sample / sample_count
            cy = math.cos(yaw)
            sy = math.sin(yaw)
            for ox, oy in obstacles:
                rx = ox - self.current_x
                ry = oy - self.current_y
                xb = cy * rx + sy * ry
                yb = -sy * rx + cy * ry
                if (
                    -self.footprint_rear - margin <= xb <= self.footprint_front + margin
                    and abs(yb) <= self.footprint_half_width + margin
                ):
                    return False
        return True

    def replan_while_blocked(self, planning_goal):
        """Refresh a blocked path without deleting the last accepted plan."""
        # A synchronous Python A* may take tens or hundreds of milliseconds on
        # a dense cloud. Cancel the previous forward command before doing any
        # planning work so stop distance is not silently extended by CPU time.
        self.stop_robot()
        if not self.blocked_replan_due():
            return
        if self.replan_to_global_waypoint(planning_goal):
            self.blocked_replan_pending = False
            rospy.logwarn_throttle(1.0, "Blocked path refreshed without clearing execution state.")
        else:
            self.blocked_replan_pending = True
            rospy.logwarn_throttle(1.0, "Blocked replan failed; remain stopped on last safe state.")
        now = rospy.Time.now()
        self.last_blocked_replan_stamp = now
        self.blocked_since = now

    def angular_direction_change_requires_stop(self, angular_z):
        """Require one explicit zero-command cycle before reversing rotation."""
        last_angular = float(getattr(self, "last_cmd_angular", 0.0))
        requested = float(angular_z)
        return (
            abs(last_angular) > 1e-3
            and abs(requested) > 1e-3
            and last_angular * requested < 0.0
        )

    def publish_checked_escape_turn(self, alpha, angular_z=None):
        """Rotate only when the next footprint sweep is free; otherwise hard-stop."""
        if angular_z is None:
            angular_z = self.preferred_escape_turn(alpha)
        if self.angular_direction_change_requires_stop(angular_z):
            rospy.logwarn_throttle(
                0.5,
                "Escape turn direction change: publish an explicit zero command first.",
            )
            self.stop_robot()
            return False
        if not self.rotation_sweep_is_clear(angular_z):
            rospy.logerr_throttle(
                0.5,
                "Escape rotation inhibited: obstacle intersects future body sweep; hard stop.",
            )
            self.stop_robot()
            return False
        self.publish_safety_cmd(angular_z)
        return True

    def handle_front_safety_stop(self, alpha, planning_goal):
        """Stop/replan a blocked front, preserving a full zero reversal cycle."""
        requested_escape = self.preferred_escape_turn(alpha)
        needs_zero_cycle = self.angular_direction_change_requires_stop(
            requested_escape
        )
        self.replan_while_blocked(planning_goal)
        if needs_zero_cycle:
            return False
        return self.publish_checked_escape_turn(alpha, requested_escape)

    def control_step(self):
        if not self.has_odom:
            self.stop_robot()
            return
        if self.state == "FINISH":
            self.stop_robot()
            return

        self.advance_global_waypoint_if_needed()
        if bool(getattr(self, "progress_fault", False)):
            self.stop_robot()
            return
        if self.obstacle_data_required_now() and not self.obstacle_data_fresh():
            rospy.logerr_throttle(
                0.5,
                "Obstacle data missing/stale while avoidance is active; fail-safe hard stop.",
            )
            self.stop_robot()
            return
        if self.global_index >= len(self.global_waypoints):
            rospy.loginfo("All CSV waypoints reached. Finish.")
            self.state = "FINISH"
            self.stop_robot()
            return

        current_goal = self.global_waypoints[self.global_index]
        dist_to_goal = math.hypot(current_goal.x - self.current_x, current_goal.y - self.current_y)
        planning_goal = self.select_planning_waypoint()
        if planning_goal is None:
            self.stop_robot()
            return

        pre_front, _ = self.safety_distances()
        self.latch_blocked_replan(pre_front)

        # A failed blocked refresh keeps the old path only as a rollback copy;
        # it must never be executed after rotation merely makes the front sensor
        # look clear. Require a new accepted plan before translating again.
        if self.blocked_replan_pending and pre_front >= self.safety_stop_dist:
            now = rospy.Time.now()
            retry_age = (now - self.last_blocked_replan_stamp).to_sec()
            if retry_age >= self.blocked_replan_delay:
                self.stop_robot()
                if self.replan_to_global_waypoint(planning_goal):
                    self.blocked_replan_pending = False
                    self.last_blocked_replan_stamp = now
                else:
                    self.last_blocked_replan_stamp = now
            if self.blocked_replan_pending:
                rospy.logwarn_throttle(
                    1.0,
                    "Blocked replan is still pending; hold zero speed instead of executing stale path.",
                )
                self.stop_robot()
                return

        # 局部段走完后重新 A*，这是“滚动重规划”的核心节奏。
        if self.local_segment_done():
            plan_age = (rospy.Time.now() - self.last_plan_stamp).to_sec()
            if self.blocked_since is not None and not self.blocked_replan_due():
                pass
            elif self.local_path and plan_age < self.replan_min_interval:
                # Keep tracking the last local point briefly instead of replanning
                # every control tick when the segment is very short.
                pass
            else:
                # In an active obstacle zone, do not let the previous velocity
                # command remain live while synchronous A* builds the next
                # rolling segment.
                if self.obstacle_data_required_now():
                    self.stop_robot()
                if not self.replan_to_global_waypoint(planning_goal):
                    self.stop_robot()
                    return

        speed = clamp(self.target_speed, 0.0, self.max_linear)
        if bool(getattr(self, "zone_active", False)) and self.avoidance_max_speed > 0.0:
            limited_speed = min(speed, self.avoidance_max_speed)
            if limited_speed + 1e-6 < speed:
                rospy.logwarn_throttle(
                    2.0,
                    "Avoidance-zone speed cap: %.2f -> %.2f m/s.",
                    speed,
                    limited_speed,
                )
            speed = limited_speed
        if dist_to_goal < self.final_approach_dist:
            speed *= clamp(dist_to_goal / max(self.final_approach_dist, 1e-6), 0.25, 1.0)

        lookahead = self.compute_adaptive_lookahead(speed)
        target = self.find_lookahead_point(lookahead)
        if target is None:
            if not self.local_path:
                reason = "local path unavailable"
            else:
                last_xb, _ = self.point_in_body(*self.local_path[-1])
                reason = (
                    "local path target is behind"
                    if last_xb < self.min_forward_target
                    else "local path has no usable lookahead"
                )
            rospy.logwarn_throttle(1.0, "%s; request replan.", reason)
            self.stop_robot()
            if not self.replan_to_global_waypoint(planning_goal):
                self.stop_robot()
                return
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

        # Pure Pursuit 核心：前视点相对车体的角度 alpha -> 曲率 -> 角速度。
        alpha = wrap_to_pi(math.atan2(dy, dx) - self.current_yaw)
        curvature = 2.0 * math.sin(alpha) / actual_ld
        angular_z = self.angular_cmd_from_curvature(curvature, speed)
        linear_x = speed

        # 履带车可以原地自转：偏航误差很大时先转正，再前进。
        # 这比普通 Pure Pursuit 一边前进一边转更不容易一开始就跑歪。
        if abs(alpha) > self.rotate_in_place_angle:
            linear_x = 0.0
            angular_z = clamp(self.rotate_k_angular * alpha, -self.max_angular, self.max_angular)
            rospy.logwarn_throttle(
                0.5,
                "Motion limit [heading_rotate]: alpha=%.2frad > %.2frad, "
                "request controlled stop and rotate.",
                alpha,
                self.rotate_in_place_angle,
            )
        elif abs(alpha) > self.heading_hard_slow_angle:
            linear_x *= 0.30
            rospy.logwarn_throttle(
                0.5,
                "Motion limit [heading_hard_slow]: alpha=%.2frad speed_scale=0.30.",
                alpha,
            )
        elif abs(alpha) > self.heading_slow_angle:
            linear_x *= 0.60
            rospy.logwarn_throttle(
                0.5,
                "Motion limit [heading_slow]: alpha=%.2frad speed_scale=0.60.",
                alpha,
            )

        min_front, min_side_clear = self.safety_distances()
        if min_side_clear < self.side_safety_radius * 2.0 or min_front < self.safety_slow_dist:
            angular_z = clamp(angular_z, -self.near_obstacle_angular, self.near_obstacle_angular)

        hard_stop_reason = self.safety_hard_stop_reason(
            min_front, min_side_clear
        )
        if hard_stop_reason is not None:
            if hard_stop_reason == "footprint_intrusion":
                rospy.logerr_throttle(
                    0.5,
                    "Safety footprint intrusion: accepted obstacle is inside "
                    "the collision envelope; hard stop without rotation.",
                )
            else:
                rospy.logerr_throttle(
                    0.5,
                    "Safety side stop: side_clear=%.2fm <= %.2fm; hard stop "
                    "without rotation.",
                    min_side_clear,
                    self.side_stop_dist,
                )
            if self.blocked_since is None:
                self.blocked_since = rospy.Time.now()
            self.blocked_replan_pending = True
            self.replan_while_blocked(planning_goal)
            self.stop_robot()
            return
        if min_front < self.safety_emergency_dist:
            rospy.logwarn_throttle(
                0.5,
                "Safety emergency: front_clear=%.2f side_clear=%.2f, request checked escape.",
                min_front, min_side_clear,
            )
            self.handle_front_safety_stop(alpha, planning_goal)
            return
        if min_front < self.safety_stop_dist:
            rospy.logwarn_throttle(
                0.5,
                "Safety stop: front_clear=%.2f side_clear=%.2f, request checked escape.",
                min_front, min_side_clear,
            )
            self.handle_front_safety_stop(alpha, planning_goal)
            return
        self.clear_blocked_state()

        if abs(linear_x) < 1e-6 and abs(angular_z) > 1e-6:
            if self.angular_direction_change_requires_stop(angular_z):
                rospy.logwarn_throttle(
                    0.5,
                    "Heading rotation direction change: publish zero before reversing.",
                )
                self.stop_robot()
                return
            if not self.rotation_sweep_is_clear(angular_z):
                rospy.logerr_throttle(
                    0.5,
                    "Heading rotation inhibited: obstacle intersects future body sweep; hard stop.",
                )
                self.stop_robot()
                return
            # Zone-aware subclasses slew ordinary commands. A checked in-place
            # rotation must force linear velocity to zero immediately, rather
            # than spending several cycles moving while the fixed-centre sweep
            # assumption is no longer valid.
            self.publish_safety_cmd(angular_z)
            return
        if min_front < self.safety_slow_dist:
            scale = (min_front - self.safety_stop_dist) / (
                self.safety_slow_dist - self.safety_stop_dist + 1e-6
            )
            scale = clamp(scale, 0.0, 1.0)
            linear_x *= scale
            rospy.logwarn_throttle(
                0.5,
                "Motion limit [front_slow]: front_clear=%.2fm "
                "range=[%.2f, %.2f] speed_scale=%.2f.",
                min_front,
                self.safety_stop_dist,
                self.safety_slow_dist,
                scale,
            )
        if min_side_clear < self.side_safety_radius:
            # Side wall is close but not necessarily blocking. Slow down instead
            # of freezing the robot; front safety still stops true head-on risk.
            linear_x *= 0.45
            rospy.logwarn_throttle(
                0.5,
                "Motion limit [side_slow]: side_clear=%.2fm < %.2fm speed_scale=0.45.",
                min_side_clear,
                self.side_safety_radius,
            )

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
        rospy.loginfo("Pure Pursuit + A* follower shutdown: robot stopped.")


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_astar_follower", anonymous=False)
    follower = PurePursuitAStarFollower()
    follower.spin()
