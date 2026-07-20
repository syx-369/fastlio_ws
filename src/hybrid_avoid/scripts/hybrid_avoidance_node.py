#!/usr/bin/env python3
"""Zone-aware rolling-costmap A-star navigation for Bunker Mini."""

import csv
import json
import math
import os
import threading
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from hybrid_avoid.hybrid_planner import (
    CostmapConfig,
    HybridAStarPlanner,
    Pose2D,
    ReferencePath,
    VehicleGeometry,
    base_pose_from_lidar_odometry,
    clamp,
    extract_avoidance_zones,
    lidar_point_to_world,
    raw_lidar_clearances,
    smooth_reference_points,
    wrap_angle,
)


Point = Tuple[float, float]


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def yaw_from_quaternion(quaternion) -> float:
    sine = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosine = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sine, cosine)


class HybridAvoidanceNode:
    def __init__(self):
        rospy.init_node("hybrid_avoid")
        self.lock = threading.RLock()
        self.csv_path = str(rospy.get_param("~csv_path", ""))
        if not self.csv_path or not os.path.isfile(self.csv_path):
            raise RuntimeError("~csv_path must point to an existing waypoint CSV")

        self.odom_topic = str(rospy.get_param("~odom_topic", "/Odometry"))
        self.cloud_topic = str(rospy.get_param("~cloud_topic", "/cloud_registered_body"))
        self.cmd_topic = str(rospy.get_param("~cmd_topic", "/hybrid_avoid/cmd_vel"))
        self.world_frame = str(rospy.get_param("~world_frame", "camera_init"))
        self.odom_pose_is_lidar = as_bool(rospy.get_param("~odom_pose_is_lidar", True))

        self.vehicle = VehicleGeometry(
            float(rospy.get_param("~vehicle_length", 0.80)),
            float(rospy.get_param("~vehicle_width", 0.70)),
            float(rospy.get_param("~lidar_x", 0.40)),
            float(rospy.get_param("~lidar_y", 0.0)),
            float(rospy.get_param("~lidar_z", 0.50)),
        )
        costmap_config = CostmapConfig(
            size_x=float(rospy.get_param("~costmap_size_x", 10.0)),
            size_y=float(rospy.get_param("~costmap_size_y", 10.0)),
            resolution=float(rospy.get_param("~costmap_resolution", 0.10)),
            collision_margin=float(rospy.get_param("~collision_margin", 0.12)),
            soft_inflation=float(rospy.get_param("~soft_inflation", 0.25)),
            cost_weight=float(rospy.get_param("~cost_weight", 2.0)),
            reference_weight=float(rospy.get_param("~reference_weight", 0.35)),
            lane_half_width=float(rospy.get_param("~lane_half_width", 0.0)),
            lane_boundary_margin=float(rospy.get_param("~lane_boundary_margin", 0.08)),
            fallback_center_limit=float(rospy.get_param("~fallback_center_limit", 1.80)),
            goal_search_distance=float(rospy.get_param("~goal_search_distance", 2.00)),
            goal_search_step=float(rospy.get_param("~goal_search_step", 0.10)),
            minimum_goal_distance=float(rospy.get_param("~minimum_goal_distance", 1.20)),
        )
        self.planner = HybridAStarPlanner(self.vehicle, costmap_config)

        raw_points, tasks = self.load_csv(self.csv_path)
        smooth_points = smooth_reference_points(
            raw_points, int(rospy.get_param("~reference_smoothing_window", 5))
        )
        self.reference = ReferencePath(smooth_points)
        self.zone_mode = as_bool(rospy.get_param("~zone_mode", True))
        self.avoid_start_task = str(rospy.get_param("~avoid_start_task", "avoid_start")).strip().lower()
        self.avoid_end_task = str(rospy.get_param("~avoid_end_task", "avoid_end")).strip().lower()
        self.zones = extract_avoidance_zones(
            self.reference, raw_points, tasks, self.avoid_start_task, self.avoid_end_task
        )
        if self.zone_mode and not self.zones:
            raise RuntimeError("zone_mode requires at least one complete avoid_start/avoid_end pair")
        self.zone_enter_margin = max(0.0, float(rospy.get_param("~zone_enter_margin", 0.15)))
        self.zone_exit_margin = max(0.0, float(rospy.get_param("~zone_exit_margin", 0.10)))
        self.local_goal_distance = max(2.0, float(rospy.get_param("~local_goal_distance", 4.50)))
        self.reference_path_step = max(0.05, float(rospy.get_param("~reference_path_step", 0.10)))
        self.path_reuse_min_distance = max(
            0.30, float(rospy.get_param("~path_reuse_min_distance", 1.00))
        )

        self.min_height = float(rospy.get_param("~min_obstacle_height", 0.06))
        self.max_height = float(rospy.get_param("~max_obstacle_height", 1.40))
        self.cloud_min_range = float(rospy.get_param("~cloud_min_range", 0.12))
        self.cloud_max_range = float(rospy.get_param("~cloud_max_range", 8.0))
        self.cloud_forward_min = float(rospy.get_param("~cloud_forward_min", -1.2))
        self.cloud_forward_max = float(rospy.get_param("~cloud_forward_max", 8.0))
        self.cloud_lateral_max = float(rospy.get_param("~cloud_lateral_max", 4.0))
        self.self_filter_padding = float(rospy.get_param("~self_filter_padding", 0.05))
        self.voxel_size = max(0.04, float(rospy.get_param("~obstacle_voxel_size", 0.08)))
        self.min_points_per_voxel = max(1, int(rospy.get_param("~min_points_per_voxel", 2)))
        self.max_cloud_points = max(1000, int(rospy.get_param("~max_cloud_points", 50000)))
        self.processed_memory_time = max(0.1, float(rospy.get_param("~processed_obstacle_memory_time", 2.0)))
        self.raw_memory_time = max(0.1, float(rospy.get_param("~raw_obstacle_memory_time", 0.70)))
        self.obstacle_memory_radius = max(2.0, float(rospy.get_param("~obstacle_memory_radius", 9.0)))

        self.use_raw_safety = as_bool(rospy.get_param("~use_raw_livox_safety", False))
        self.use_raw_planning = as_bool(rospy.get_param("~use_raw_livox_planning", False))
        self.raw_topic = str(rospy.get_param("~raw_livox_topic", "/livox/lidar"))
        self.raw_timeout = float(rospy.get_param("~raw_livox_timeout", 0.40))
        self.raw_stride = max(1, int(rospy.get_param("~raw_livox_stride", 2)))
        self.raw_max_range = float(rospy.get_param("~raw_livox_max_range", 3.0))
        self.raw_min_points = max(1, int(rospy.get_param("~raw_safety_min_points", 3)))

        self.control_rate = max(2.0, float(rospy.get_param("~control_rate", 20.0)))
        self.plan_rate = max(0.2, float(rospy.get_param("~plan_rate", 3.0)))
        self.blocked_retry_rate = max(0.2, float(rospy.get_param("~blocked_retry_rate", 2.0)))
        self.target_speed = float(rospy.get_param("~target_speed", 0.12))
        self.max_linear = float(rospy.get_param("~max_linear", 0.30))
        self.max_angular = float(rospy.get_param("~max_angular", 0.80))
        self.max_linear_accel = float(rospy.get_param("~max_linear_accel", 0.25))
        self.max_angular_accel = float(rospy.get_param("~max_angular_accel", 1.20))
        self.lookahead = float(rospy.get_param("~lookahead", 0.50))
        self.min_lookahead = float(rospy.get_param("~min_lookahead", 0.35))
        self.max_lookahead = float(rospy.get_param("~max_lookahead", 0.85))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.20))
        self.safety_slow = float(rospy.get_param("~safety_slow_distance", 0.90))
        self.safety_stop = float(rospy.get_param("~safety_stop_distance", 0.35))
        self.safety_lateral_padding = float(rospy.get_param("~safety_lateral_padding", 0.12))
        self.safety_clear_hold = float(rospy.get_param("~safety_clear_hold_time", 0.80))
        self.cloud_timeout = float(rospy.get_param("~cloud_timeout", 1.0))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.60))
        self.require_cloud = as_bool(rospy.get_param("~require_cloud", True))
        self.enabled = as_bool(rospy.get_param("~enabled", False))

        self.pose: Optional[Pose2D] = None
        self.pose_history = deque(maxlen=200)
        self.progress_s = 0.0
        self.zone_index = -1
        self.local_path: List[Point] = []
        self.local_goal_s = 0.0
        self.state = "INITIALIZING"
        self.state_detail = ""
        self.processed_voxels: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
        self.raw_voxels: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
        self.raw_min_front = float("inf")
        self.raw_min_radial = float("inf")
        self.last_odom = rospy.Time(0)
        self.last_cloud = rospy.Time(0)
        self.last_raw = rospy.Time(0)
        self.last_plan = rospy.Time(0)
        self.last_command_time = rospy.Time.now()
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.safety_clear_since: Optional[rospy.Time] = None
        # Keep the close-range stop independent from the planner state.  A new
        # A* result must never overwrite SAFETY_STOP and release one velocity
        # command while the raw/processed point cloud is still occupied.
        self.front_safety_latched = False
        self.consecutive_plan_failures = 0
        # A* can take several hundred milliseconds in a dense indoor cloud.
        # Keep it off the 20 Hz velocity-control thread so the Bunker command
        # watchdog continues receiving smooth commands while a path is built.
        self.plan_in_progress = False

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=10, latch=True)
        self.reference_pub = rospy.Publisher("~reference_path", Path, queue_size=1, latch=True)
        self.local_pub = rospy.Publisher("~local_path", Path, queue_size=1, latch=True)
        self.costmap_pub = rospy.Publisher("~local_costmap", OccupancyGrid, queue_size=1, latch=True)
        self.obstacle_pub = rospy.Publisher("~obstacles", PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)
        self.footprint_pub = rospy.Publisher("~vehicle_footprint", Marker, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)
        rospy.Subscriber("~enable", Bool, self.enable_callback, queue_size=1)
        if self.use_raw_safety or self.use_raw_planning:
            try:
                from livox_ros_driver2.msg import CustomMsg
            except ImportError as error:
                raise RuntimeError("source livox_ws before enabling raw Livox input") from error
            rospy.Subscriber(self.raw_topic, CustomMsg, self.raw_callback, queue_size=1)

        self.publish_path(self.reference_pub, self.reference.segment(0.0, self.reference.length, 0.10))
        self.publish_zone_markers()
        self.set_state("DISABLED" if not self.enabled else "WAITING_FOR_SENSORS", "startup")
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "Hybrid avoid ready: route=%.2fm zones=%d costmap=%.1fx%.1f@%.2f cmd=%s",
            self.reference.length,
            len(self.zones),
            costmap_config.size_x,
            costmap_config.size_y,
            costmap_config.resolution,
            self.cmd_topic,
        )

    @staticmethod
    def load_csv(path: str) -> Tuple[List[Point], List[str]]:
        points, tasks = [], []
        with open(path, "r", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames or "x" not in reader.fieldnames or "y" not in reader.fieldnames:
                raise RuntimeError("CSV must contain x and y columns")
            for row in reader:
                try:
                    points.append((float(row["x"]), float(row["y"])))
                    tasks.append(str(row.get("task", "none") or "none").strip().lower())
                except (ValueError, TypeError, KeyError):
                    continue
        if len(points) < 2:
            raise RuntimeError("CSV contains fewer than two valid points")
        return points, tasks

    def zone_at(self, progress_s: float) -> int:
        if not self.zone_mode:
            return 0
        for index, (start_s, end_s) in enumerate(self.zones):
            if start_s - self.zone_enter_margin <= progress_s <= end_s + self.zone_exit_margin:
                return index
        return -1

    def set_state(self, state: str, detail: str):
        if state == self.state and detail == self.state_detail:
            return
        self.state, self.state_detail = state, detail
        message = {
            "state": state,
            "detail": detail,
            "progress_s": round(self.progress_s, 3),
            "zone_active": self.zone_index >= 0 or not self.zone_mode,
            "zone_index": self.zone_index + 1 if self.zone_index >= 0 else 0,
            "local_path_points": len(self.local_path),
            "local_goal_s": round(self.local_goal_s, 3),
        }
        self.state_pub.publish(String(data=json.dumps(message, ensure_ascii=False, sort_keys=True)))
        rospy.loginfo("Hybrid state: %s | %s", state, detail)

    def enable_callback(self, message: Bool):
        with self.lock:
            self.enabled = bool(message.data)
            if not self.enabled:
                self.local_path = []
                self.safety_clear_since = None
                self.publish_path(self.local_pub, [])
                self.set_state("DISABLED", "enable_topic_false")
                self.stop()

    def odom_callback(self, message: Odometry):
        pose = message.pose.pose
        sensor_pose = Pose2D(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        base_pose = base_pose_from_lidar_odometry(sensor_pose, self.vehicle) if self.odom_pose_is_lidar else sensor_pose
        stamp = message.header.stamp if message.header.stamp != rospy.Time(0) else rospy.Time.now()
        with self.lock:
            self.pose = base_pose
            self.pose_history.append((stamp.to_sec(), base_pose))
            self.last_odom = rospy.Time.now()
        self.publish_footprint(base_pose)

    def pose_at(self, stamp: rospy.Time) -> Optional[Pose2D]:
        with self.lock:
            if not self.pose_history:
                return self.pose
            if stamp == rospy.Time(0):
                return self.pose_history[-1][1]
            target = stamp.to_sec()
            return min(self.pose_history, key=lambda item: abs(item[0] - target))[1]

    def _accept_point(self, point, base_pose: Pose2D) -> Optional[Tuple[float, float]]:
        x_lidar, y_lidar, z_lidar = float(point[0]), float(point[1]), float(point[2])
        x_base, y_base, z_base = x_lidar + self.vehicle.lidar_x, y_lidar + self.vehicle.lidar_y, z_lidar + self.vehicle.lidar_z
        if not self.min_height <= z_base <= self.max_height:
            return None
        planar = math.hypot(x_lidar, y_lidar)
        if planar < self.cloud_min_range or planar > self.cloud_max_range:
            return None
        if not self.cloud_forward_min <= x_base <= self.cloud_forward_max or abs(y_base) > self.cloud_lateral_max:
            return None
        if abs(x_base) <= self.vehicle.half_length + self.self_filter_padding and abs(y_base) <= self.vehicle.half_width + self.self_filter_padding:
            return None
        world = lidar_point_to_world((x_lidar, y_lidar, z_lidar), base_pose, self.vehicle)
        return world[0], world[1]

    def cloud_callback(self, message: PointCloud2):
        pose = self.pose_at(message.header.stamp)
        if pose is None:
            return
        bins: Dict[Tuple[int, int], List[float]] = {}
        count = 0
        for point in pc2.read_points(message, field_names=("x", "y", "z"), skip_nans=True):
            count += 1
            if count > self.max_cloud_points:
                break
            accepted = self._accept_point(point, pose)
            if accepted is None:
                continue
            key = (int(math.floor(accepted[0] / self.voxel_size)), int(math.floor(accepted[1] / self.voxel_size)))
            value = bins.setdefault(key, [0.0, 0.0, 0.0])
            value[0] += accepted[0]
            value[1] += accepted[1]
            value[2] += 1.0
        now = rospy.Time.now().to_sec()
        with self.lock:
            for key, value in bins.items():
                if value[2] >= self.min_points_per_voxel:
                    self.processed_voxels[key] = (value[0] / value[2], value[1] / value[2], now)
            self.last_cloud = rospy.Time.now()
            self.prune_obstacles(now)

    def raw_callback(self, message):
        points = []
        for index, point in enumerate(message.points):
            if index % self.raw_stride == 0:
                points.append((float(point.x), float(point.y), float(point.z)))
        front, radial = raw_lidar_clearances(
            points,
            self.vehicle,
            self.min_height,
            self.max_height,
            self.raw_max_range,
            self.safety_lateral_padding,
            self.raw_min_points,
        )
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.raw_min_front, self.raw_min_radial = front, radial
            self.last_raw = rospy.Time.now()
            pose = self.pose
            if self.use_raw_planning and pose is not None:
                for point in points:
                    if math.hypot(point[0], point[1]) > self.raw_max_range:
                        continue
                    accepted = self._accept_point(point, pose)
                    if accepted is None:
                        continue
                    key = (int(math.floor(accepted[0] / self.voxel_size)), int(math.floor(accepted[1] / self.voxel_size)))
                    self.raw_voxels[key] = (accepted[0], accepted[1], now)
            self.prune_obstacles(now)

    def prune_obstacles(self, now: float):
        pose = self.pose
        for storage, lifetime in ((self.processed_voxels, self.processed_memory_time), (self.raw_voxels, self.raw_memory_time)):
            expired = []
            for key, value in storage.items():
                too_old = now - value[2] > lifetime
                too_far = pose is not None and (value[0] - pose.x) ** 2 + (value[1] - pose.y) ** 2 > self.obstacle_memory_radius ** 2
                if too_old or too_far:
                    expired.append(key)
            for key in expired:
                storage.pop(key, None)

    def obstacles(self) -> List[Point]:
        self.prune_obstacles(rospy.Time.now().to_sec())
        merged = {(value[0], value[1]) for value in self.processed_voxels.values()}
        merged.update((value[0], value[1]) for value in self.raw_voxels.values())
        return list(merged)

    def sensor_error(self) -> Optional[str]:
        now = rospy.Time.now()
        if self.pose is None:
            return "no_odometry_pose"
        if self.last_odom == rospy.Time(0):
            return "no_odometry_timestamp"
        if (now - self.last_odom).to_sec() > self.odom_timeout:
            return "odometry_stale"
        if self.require_cloud and (self.last_cloud == rospy.Time(0) or (now - self.last_cloud).to_sec() > self.cloud_timeout):
            return "pointcloud_stale"
        if (self.use_raw_safety or self.use_raw_planning) and (
            self.last_raw == rospy.Time(0) or (now - self.last_raw).to_sec() > self.raw_timeout
        ):
            return "raw_livox_stale"
        return None

    def update_progress(self, pose: Pose2D):
        projection = self.reference.project(
            pose.x,
            pose.y,
            max(0.0, self.progress_s - 0.8),
            min(self.reference.length, self.progress_s + 3.0),
        )
        self.progress_s = max(self.progress_s, projection.s)
        return projection

    def publish_path(self, publisher, points: Sequence[Point]):
        message = Path()
        message.header = Header(stamp=rospy.Time.now(), frame_id=self.world_frame)
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = point
            if len(points) > 1:
                neighbor = points[index + 1] if index < len(points) - 1 else points[index - 1]
                dx = neighbor[0] - point[0] if index < len(points) - 1 else point[0] - neighbor[0]
                dy = neighbor[1] - point[1] if index < len(points) - 1 else point[1] - neighbor[1]
                yaw = math.atan2(dy, dx)
                pose.pose.orientation.z = math.sin(0.5 * yaw)
                pose.pose.orientation.w = math.cos(0.5 * yaw)
            else:
                pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        publisher.publish(message)

    def publish_costmap(self, grid):
        message = OccupancyGrid()
        message.header = Header(stamp=rospy.Time.now(), frame_id=self.world_frame)
        message.info.resolution = grid.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = grid.origin_x
        message.info.origin.position.y = grid.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = list(grid.data)
        self.costmap_pub.publish(message)

    def publish_obstacles(self, obstacles: Sequence[Point]):
        self.obstacle_pub.publish(
            pc2.create_cloud_xyz32(
                Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
                [(x, y, 0.10) for x, y in obstacles],
            )
        )

    def publish_zone_markers(self):
        markers = MarkerArray()
        now = rospy.Time.now()
        for index, (start_s, end_s) in enumerate(self.zones):
            for boundary_index, (s_value, label, red, green) in enumerate(
                ((start_s, "HYBRID START", 0.1, 0.9), (end_s, "HYBRID END", 0.9, 0.2))
            ):
                x, y, yaw = self.reference.sample(s_value)
                marker = Marker()
                marker.header = Header(stamp=now, frame_id=self.world_frame)
                marker.ns = "hybrid_zone"
                marker.id = index * 2 + boundary_index
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.65
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.28
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = red, green, 0.2, 1.0
                marker.text = "{} {}".format(label, index + 1)
                markers.markers.append(marker)
                line = Marker()
                line.header = marker.header
                line.ns = "hybrid_zone_boundary"
                line.id = marker.id
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.06
                line.color = marker.color
                for lateral in (-1.4, 1.4):
                    point = RosPoint()
                    point.x = x - math.sin(yaw) * lateral
                    point.y = y + math.cos(yaw) * lateral
                    point.z = 0.05
                    line.points.append(point)
                markers.markers.append(line)
        self.marker_pub.publish(markers)

    def publish_footprint(self, pose: Pose2D):
        marker = Marker()
        marker.header = Header(stamp=rospy.Time.now(), frame_id=self.world_frame)
        marker.ns, marker.id = "hybrid_vehicle", 0
        marker.type, marker.action = Marker.CUBE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose.x, pose.y, 0.08
        marker.pose.orientation.z, marker.pose.orientation.w = math.sin(0.5 * pose.yaw), math.cos(0.5 * pose.yaw)
        marker.scale.x, marker.scale.y, marker.scale.z = self.vehicle.length, self.vehicle.width, 0.16
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.6, 1.0, 0.55
        self.footprint_pub.publish(marker)

    def request_plan(self, pose: Pose2D, obstacles: Sequence[Point]):
        if self.plan_in_progress:
            return
        self.plan_in_progress = True
        self.last_plan = rospy.Time.now()
        progress_s = self.progress_s
        worker = threading.Thread(
            target=self.plan_now,
            args=(pose, list(obstacles), progress_s),
            name="hybrid_astar_worker",
        )
        worker.daemon = True
        worker.start()

    def plan_now(self, pose: Pose2D, obstacles: Sequence[Point], progress_s: float):
        requested_goal_s = min(self.reference.length, progress_s + self.local_goal_distance)
        try:
            # Deliberately run without self.lock. Sensor callbacks and the 20 Hz
            # tracker must remain live while this pure planner uses its snapshot.
            result = self.planner.plan(pose, progress_s, self.reference, obstacles, requested_goal_s)
        except Exception as error:
            with self.lock:
                self.plan_in_progress = False
                self.last_plan = rospy.Time.now()
                self.set_state("ERROR_STOP", "planner_exception={}".format(error))
            return

        with self.lock:
            self.plan_in_progress = False
            self.last_plan = rospy.Time.now()
            avoidance_active = not self.zone_mode or self.zone_index >= 0
            if not self.enabled or not avoidance_active or rospy.is_shutdown():
                return
            current_pose = self.pose if self.pose is not None else pose
            self.apply_plan_result(current_pose, result)

    def set_planning_state(self, state: str, detail: str):
        # A plan that was already running may finish after the close-range
        # safety latch engages. It may update the cached path, but it must not
        # take ownership of the safety state or velocity decision.
        if not self.front_safety_latched:
            self.set_state(state, detail)

    def apply_plan_result(self, pose: Pose2D, result):
        self.publish_costmap(result.costmap)
        old_path_clear, old_remaining, old_reason = self.planner.remaining_path_status(
            self.local_path, pose, result.costmap
        )
        if result.status != "path":
            self.consecutive_plan_failures += 1
            if old_path_clear and old_remaining >= self.path_reuse_min_distance:
                # A single point-cloud frame may cover the rolling goal or make
                # A* temporarily fail.  Continue on the complete old path only
                # after rechecking it against the newest costmap.  This avoids
                # the observed move/zero/move command pulse without hiding a
                # newly occupied route.
                self.set_planning_state(
                    "FOLLOW_HYBRID_PATH",
                    "replan_deferred={} failures={} old_remaining={:.2f}".format(
                        result.reason, self.consecutive_plan_failures, old_remaining
                    ),
                )
                return
            self.local_path = []
            self.publish_path(self.local_pub, [])
            self.set_planning_state(
                "BLOCKED",
                "{} old_path={}".format(result.reason, old_reason),
            )
            return
        self.consecutive_plan_failures = 0

        # Stable-path hysteresis: do not replace a safe, sufficiently long path
        # at every 3 Hz planning tick.  A new path is accepted when the current
        # one is occupied or nearly consumed.  Moving obstacles still trigger
        # an immediate switch because old_path_clear then becomes false.
        if old_path_clear and old_remaining >= self.path_reuse_min_distance:
            self.set_planning_state(
                "FOLLOW_HYBRID_PATH",
                "keep_safe_path old_remaining={:.2f} candidate_goal_s={:.2f}".format(
                    old_remaining, result.goal_s
                ),
            )
            return
        self.local_path = result.path
        self.local_goal_s = result.goal_s
        self.publish_path(self.local_pub, self.local_path)
        maximum_deviation = 0.0
        for point in self.local_path:
            maximum_deviation = max(
                maximum_deviation,
                abs(self.reference.project(point[0], point[1], max(0.0, self.progress_s - 0.5), self.local_goal_s + 1.0).d),
            )
        detail = "astar expanded={} max_d={:.2f} goal_s={:.2f}".format(
            result.expanded, maximum_deviation, self.local_goal_s
        )
        self.set_planning_state("FOLLOW_HYBRID_PATH", detail)

    @staticmethod
    def nearest_index(points: Sequence[Point], pose: Pose2D) -> int:
        return min(range(len(points)), key=lambda index: (points[index][0] - pose.x) ** 2 + (points[index][1] - pose.y) ** 2)

    def lookahead_target(self, points: Sequence[Point], pose: Pose2D, distance: float) -> Optional[Point]:
        if not points:
            return None
        start = self.nearest_index(points, pose)
        accumulated = 0.0
        previous = (pose.x, pose.y)
        for point in points[start:]:
            accumulated += math.hypot(point[0] - previous[0], point[1] - previous[1])
            if accumulated >= distance:
                return point
            previous = point
        return points[-1]

    def safety_distances(self, pose: Pose2D, obstacles: Sequence[Point]) -> Tuple[float, float]:
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        front, radial = float("inf"), float("inf")
        for x, y in obstacles:
            dx, dy = x - pose.x, y - pose.y
            xb, yb = cosine * dx + sine * dy, -sine * dx + cosine * dy
            radial = min(radial, math.hypot(dx, dy))
            clearance = xb - self.vehicle.half_length
            if clearance >= 0.0 and abs(yb) <= self.vehicle.half_width + self.safety_lateral_padding:
                front = min(front, clearance)
        if self.use_raw_safety:
            front = min(front, self.raw_min_front)
            radial = min(radial, self.raw_min_radial)
        return front, radial

    def publish_command(self, linear: float, angular: float, immediate: bool = False):
        now = rospy.Time.now()
        dt = max(0.001, (now - self.last_command_time).to_sec())
        if immediate:
            linear, angular = 0.0, 0.0
        else:
            linear = clamp(linear, self.last_linear - self.max_linear_accel * dt, self.last_linear + self.max_linear_accel * dt)
            angular = clamp(angular, self.last_angular - self.max_angular_accel * dt, self.last_angular + self.max_angular_accel * dt)
        message = Twist()
        message.linear.x = clamp(linear, -self.max_linear, self.max_linear)
        message.angular.z = clamp(angular, -self.max_angular, self.max_angular)
        self.cmd_pub.publish(message)
        self.last_linear, self.last_angular, self.last_command_time = message.linear.x, message.angular.z, now

    def stop(self):
        self.publish_command(0.0, 0.0, True)

    def track(self, pose: Pose2D, obstacles: Sequence[Point]):
        if self.local_path:
            points = self.local_path
        else:
            points = self.reference.segment(
                self.progress_s,
                min(self.reference.length, self.progress_s + 6.0),
                self.reference_path_step,
            )
        lookahead = clamp(self.lookahead + 0.8 * self.target_speed, self.min_lookahead, self.max_lookahead)
        target = self.lookahead_target(points, pose, lookahead)
        if target is None:
            self.stop()
            return
        dx, dy = target[0] - pose.x, target[1] - pose.y
        distance = max(0.02, math.hypot(dx, dy))
        alpha = wrap_angle(math.atan2(dy, dx) - pose.yaw)
        curvature = 2.0 * math.sin(alpha) / distance
        speed = clamp(self.target_speed, 0.0, self.max_linear)
        angular = clamp(speed * curvature, -self.max_angular, self.max_angular)
        linear = speed / (1.0 + 0.9 * abs(curvature))
        front, radial = self.safety_distances(pose, obstacles)
        if front <= self.safety_stop:
            self.front_safety_latched = True
            self.safety_clear_since = None
            self.set_state("SAFETY_STOP", "front_clearance={:.3f}".format(front))
            self.stop()
            return
        if self.front_safety_latched:
            if front < self.safety_slow:
                self.safety_clear_since = None
                self.set_state("SAFETY_STOP", "front_clearance={:.3f}".format(front))
                self.stop()
                return
            if self.safety_clear_since is None:
                self.safety_clear_since = rospy.Time.now()
                self.set_state("SAFETY_STOP", "waiting_for_clear_hold")
                self.stop()
                return
            if (rospy.Time.now() - self.safety_clear_since).to_sec() < self.safety_clear_hold:
                self.stop()
                return
            self.front_safety_latched = False
            self.safety_clear_since = None
            self.set_state(
                "FOLLOW_HYBRID_PATH" if self.local_path else "FOLLOW_REFERENCE",
                "safety_clear",
            )
        if front < self.safety_slow:
            linear *= clamp((front - self.safety_stop) / max(0.01, self.safety_slow - self.safety_stop), 0.0, 1.0)
        if abs(alpha) > 1.0:
            if radial < self.vehicle.half_diagonal + self.planner.config.collision_margin + 0.15:
                self.set_state("SAFETY_STOP", "rotation_clearance_insufficient")
                self.stop()
                return
            linear = 0.0
            angular = clamp(0.55 * alpha, -0.45, 0.45)
        elif abs(alpha) > 0.65:
            linear *= 0.45
        self.publish_command(linear, angular)

    def control_step(self):
        with self.lock:
            if not self.enabled:
                self.set_state("DISABLED", "navigation_disabled")
                self.stop()
                return
            error = self.sensor_error()
            if error:
                rospy.logwarn_throttle(
                    2.0,
                    "Hybrid sensor gate: %s pose=%s odom_age=%.3f cloud_age=%.3f",
                    error,
                    "set" if self.pose is not None else "none",
                    (rospy.Time.now() - self.last_odom).to_sec() if self.last_odom != rospy.Time(0) else -1.0,
                    (rospy.Time.now() - self.last_cloud).to_sec() if self.last_cloud != rospy.Time(0) else -1.0,
                )
                self.set_state("SENSOR_STOP", error)
                self.stop()
                return
            if self.state == "SENSOR_STOP":
                self.set_state("FOLLOW_REFERENCE", "sensors_recovered")
            pose = self.pose
            if pose is None:
                self.stop()
                return
            self.update_progress(pose)
            previous_zone = self.zone_index
            self.zone_index = self.zone_at(self.progress_s)
            obstacles = self.obstacles()
            self.publish_obstacles(obstacles)
            if previous_zone < 0 <= self.zone_index:
                self.set_state("FOLLOW_REFERENCE", "avoidance_zone_enter")
            elif previous_zone >= 0 and self.zone_index < 0:
                self.local_path = []
                self.publish_path(self.local_pub, [])
                self.set_state("FOLLOW_REFERENCE", "avoidance_zone_exit")

            # The safety latch owns velocity output until the forward corridor
            # has stayed clear for safety_clear_hold.  In particular, do not
            # call plan_now() here: it changes the navigation state and was the
            # cause of the observed FOLLOW_* <-> SAFETY_STOP command pulsing.
            if self.front_safety_latched:
                self.track(pose, obstacles)
                return

            if self.progress_s >= self.reference.length - 0.05:
                goal_x, goal_y, _ = self.reference.sample(self.reference.length)
                if math.hypot(goal_x - pose.x, goal_y - pose.y) <= self.goal_tolerance:
                    self.set_state("FINISHED", "reference_path_complete")
                    self.stop()
                    return

            avoidance_active = not self.zone_mode or self.zone_index >= 0
            now = rospy.Time.now()
            interval = 1.0 / (self.blocked_retry_rate if self.state == "BLOCKED" else self.plan_rate)
            if avoidance_active and (now - self.last_plan).to_sec() >= interval:
                self.request_plan(pose, obstacles)
            elif not avoidance_active and self.local_path:
                self.local_path = []
                self.publish_path(self.local_pub, [])

            if self.state in ("BLOCKED", "SENSOR_STOP", "DISABLED", "FINISHED", "ERROR_STOP"):
                self.stop()
                return
            if not avoidance_active:
                self.set_state("FOLLOW_REFERENCE", "outside_avoidance_zone")
            self.track(pose, obstacles)

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            try:
                self.control_step()
            except Exception as error:
                rospy.logerr_throttle(1.0, "Hybrid control exception: %s", error)
                self.set_state("ERROR_STOP", str(error))
                self.stop()
            rate.sleep()

    def on_shutdown(self):
        self.stop()
        rospy.sleep(0.05)
        self.stop()


if __name__ == "__main__":
    try:
        HybridAvoidanceNode().run()
    except rospy.ROSInterruptException:
        pass
