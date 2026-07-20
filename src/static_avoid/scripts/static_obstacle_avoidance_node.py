#!/usr/bin/env python3
"""Standalone static-obstacle navigation node for Bunker Mini.

Inputs:
  * nav_msgs/Odometry: FAST-LIO pose (lidar pose by default)
  * sensor_msgs/PointCloud2: current scan in lidar/body coordinates
  * CSV reference path

Outputs:
  * geometry_msgs/Twist on a configurable raw command topic
  * reference/local nav_msgs/Path topics for RViz
  * planner state and filtered obstacle cloud for diagnostics

The node never rotates blindly when blocked.  If sensing is stale or no safe
detour exists it continuously commands zero velocity and retries planning.
"""

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
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from static_avoid.static_obstacle_planner import (
    ObstacleIndex,
    PlanResult,
    PlannerConfig,
    Pose2D,
    ReferencePath,
    StaticObstaclePlanner,
    VehicleGeometry,
    base_pose_from_lidar_odometry,
    clamp,
    extract_avoidance_zones,
    lidar_point_to_world,
    raw_lidar_safety_distances,
    smooth_path_points,
    wrap_angle,
)


Point = Tuple[float, float]


def parse_float_tuple(value, default):
    if isinstance(value, str):
        try:
            parsed = tuple(float(item.strip()) for item in value.split(",") if item.strip())
            return parsed or tuple(default)
        except ValueError:
            return tuple(default)
    if isinstance(value, (list, tuple)):
        try:
            return tuple(float(item) for item in value)
        except (TypeError, ValueError):
            return tuple(default)
    return tuple(default)


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class StaticObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node("static_avoid")
        self.lock = threading.RLock()

        self.csv_path = rospy.get_param("~csv_path", "")
        if not self.csv_path or not os.path.isfile(self.csv_path):
            raise RuntimeError("~csv_path must point to an existing waypoint CSV")

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered_body")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/static_avoid/cmd_vel")
        self.world_frame = rospy.get_param("~world_frame", "camera_init")
        self.odom_pose_is_lidar = parse_bool(rospy.get_param("~odom_pose_is_lidar", True))

        self.vehicle = VehicleGeometry(
            length=float(rospy.get_param("~vehicle_length", 0.80)),
            width=float(rospy.get_param("~vehicle_width", 0.70)),
            lidar_x=float(rospy.get_param("~lidar_x", 0.40)),
            lidar_y=float(rospy.get_param("~lidar_y", 0.0)),
            lidar_z=float(rospy.get_param("~lidar_z", 0.50)),
        )
        if self.vehicle.length <= 0.0 or self.vehicle.width <= 0.0:
            raise RuntimeError("Vehicle dimensions must be positive")

        default_offsets = (0.65, 0.85, 1.05, 1.20)
        self.planner_config = PlannerConfig(
            planning_horizon=float(rospy.get_param("~planning_horizon", 5.0)),
            path_sample_step=float(rospy.get_param("~path_sample_step", 0.10)),
            collision_margin=float(rospy.get_param("~collision_margin", 0.16)),
            obstacle_check_extra=float(rospy.get_param("~obstacle_check_extra", 0.05)),
            prepare_distance=float(rospy.get_param("~prepare_distance", 1.40)),
            obstacle_pass_distance=float(rospy.get_param("~obstacle_pass_distance", 0.55)),
            rejoin_distance=float(rospy.get_param("~rejoin_distance", 1.80)),
            min_transition_length=float(rospy.get_param("~min_transition_length", 1.00)),
            transition_length_per_meter=float(rospy.get_param("~transition_length_per_meter", 1.80)),
            rejoin_length_per_meter=float(rospy.get_param("~rejoin_length_per_meter", 1.80)),
            lateral_offsets=parse_float_tuple(rospy.get_param("~lateral_offsets", list(default_offsets)), default_offsets),
            max_lateral_offset=float(rospy.get_param("~max_lateral_offset", 1.25)),
            lane_half_width=float(rospy.get_param("~lane_half_width", 0.0)),
            lane_boundary_margin=float(rospy.get_param("~lane_boundary_margin", 0.08)),
            max_curvature=float(rospy.get_param("~max_curvature", 2.20)),
            min_clearance=float(rospy.get_param("~min_clearance", 0.12)),
            weight_length=float(rospy.get_param("~weight_length", 1.0)),
            weight_deviation=float(rospy.get_param("~weight_deviation", 0.65)),
            weight_curvature=float(rospy.get_param("~weight_curvature", 0.35)),
            weight_curvature_change=float(rospy.get_param("~weight_curvature_change", 0.15)),
            weight_clearance=float(rospy.get_param("~weight_clearance", 0.55)),
            enable_multi_obstacle_lattice=parse_bool(rospy.get_param("~enable_multi_obstacle_lattice", True)),
            lattice_lateral_step=float(rospy.get_param("~lattice_lateral_step", 0.05)),
            lattice_smoothing_passes=int(rospy.get_param("~lattice_smoothing_passes", 4)),
            lattice_weight_deviation=float(rospy.get_param("~lattice_weight_deviation", 0.30)),
            lattice_weight_slope=float(rospy.get_param("~lattice_weight_slope", 1.20)),
            lattice_weight_acceleration=float(rospy.get_param("~lattice_weight_acceleration", 10.0)),
        )

        self.reference_smoothing_window = int(rospy.get_param("~reference_smoothing_window", 5))
        self.zone_mode = parse_bool(rospy.get_param("~zone_mode", True))
        self.avoid_start_task = str(rospy.get_param("~avoid_start_task", "avoid_start")).strip().lower()
        self.avoid_end_task = str(rospy.get_param("~avoid_end_task", "avoid_end")).strip().lower()
        self.zone_enter_margin = max(0.0, float(rospy.get_param("~zone_enter_margin", 0.15)))
        self.zone_exit_margin = max(0.0, float(rospy.get_param("~zone_exit_margin", 0.10)))

        raw_path_points, route_tasks = self.load_csv_route(self.csv_path)
        path_points = smooth_path_points(raw_path_points, self.reference_smoothing_window)
        self.reference = ReferencePath(path_points)
        self.planner = StaticObstaclePlanner(self.reference, self.vehicle, self.planner_config)
        self.avoidance_zones = self.build_avoidance_zones(raw_path_points, route_tasks)
        if self.zone_mode and not self.avoidance_zones:
            raise RuntimeError(
                "zone_mode is enabled but CSV has no complete '{}'/'{}' pair".format(
                    self.avoid_start_task, self.avoid_end_task
                )
            )

        # Point-cloud filtering in body-centred coordinates.
        self.min_obstacle_height = float(rospy.get_param("~min_obstacle_height", 0.06))
        self.max_obstacle_height = float(rospy.get_param("~max_obstacle_height", 1.40))
        self.cloud_min_range = float(rospy.get_param("~cloud_min_range", 0.12))
        self.cloud_max_range = float(rospy.get_param("~cloud_max_range", 8.0))
        self.cloud_forward_min = float(rospy.get_param("~cloud_forward_min", -1.2))
        self.cloud_forward_max = float(rospy.get_param("~cloud_forward_max", 8.0))
        self.cloud_lateral_max = float(rospy.get_param("~cloud_lateral_max", 4.0))
        self.self_filter_padding = float(rospy.get_param("~self_filter_padding", 0.05))
        self.obstacle_voxel_size = float(rospy.get_param("~obstacle_voxel_size", 0.08))
        self.min_points_per_voxel = max(1, int(rospy.get_param("~min_points_per_voxel", 2)))
        self.obstacle_memory_time = float(rospy.get_param("~obstacle_memory_time", 12.0))
        self.obstacle_memory_radius = float(rospy.get_param("~obstacle_memory_radius", 9.0))
        self.max_obstacle_voxels = max(100, int(rospy.get_param("~max_obstacle_voxels", 5000)))
        self.cloud_stride = max(1, int(rospy.get_param("~cloud_stride", 1)))
        self.max_cloud_points = max(1000, int(rospy.get_param("~max_cloud_points", 50000)))
        self.obstacle_publish_rate = float(rospy.get_param("~obstacle_publish_rate", 2.0))

        # Optional close-range safety source.  S-FAST-LIO's body cloud may have
        # a large blind radius, so raw Livox points are used only for stop/slow
        # decisions, never as an unregistered world-frame planning cloud.
        self.use_raw_livox_safety = parse_bool(rospy.get_param("~use_raw_livox_safety", False))
        self.raw_livox_topic = str(rospy.get_param("~raw_livox_topic", "/livox/lidar"))
        self.raw_livox_timeout = float(rospy.get_param("~raw_livox_timeout", 0.40))
        self.raw_livox_stride = max(1, int(rospy.get_param("~raw_livox_stride", 2)))
        self.raw_livox_max_range = float(rospy.get_param("~raw_livox_max_range", 3.0))
        self.raw_safety_min_points = max(1, int(rospy.get_param("~raw_safety_min_points", 3)))

        # RViz visualization.  Paths carry orientation in every PoseStamped;
        # markers make those headings visible without relying on RViz Path pose style.
        self.marker_spacing = max(0.20, float(rospy.get_param("~marker_spacing", 0.50)))
        self.marker_arrow_length = max(0.10, float(rospy.get_param("~marker_arrow_length", 0.25)))
        self.pose_visualization_rate = max(1.0, float(rospy.get_param("~pose_visualization_rate", 10.0)))

        # Tracking and independent safety layer.
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.plan_rate = float(rospy.get_param("~plan_rate", 2.0))
        self.blocked_retry_rate = float(rospy.get_param("~blocked_retry_rate", 1.0))
        self.target_speed = float(rospy.get_param("~target_speed", 0.22))
        self.max_linear = float(rospy.get_param("~max_linear", 0.30))
        self.max_angular = float(rospy.get_param("~max_angular", 0.80))
        self.lookahead = float(rospy.get_param("~lookahead", 0.50))
        self.min_lookahead = float(rospy.get_param("~min_lookahead", 0.35))
        self.max_lookahead = float(rospy.get_param("~max_lookahead", 0.80))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.20))
        self.rejoin_lateral_tolerance = float(rospy.get_param("~rejoin_lateral_tolerance", 0.15))
        self.rejoin_yaw_tolerance = float(rospy.get_param("~rejoin_yaw_tolerance", math.radians(10.0)))
        self.safety_slow_distance = float(rospy.get_param("~safety_slow_distance", 0.85))
        self.safety_stop_distance = float(rospy.get_param("~safety_stop_distance", 0.30))
        self.safety_lateral_padding = float(rospy.get_param("~safety_lateral_padding", 0.12))
        self.safety_clear_hold_time = float(rospy.get_param("~safety_clear_hold_time", 0.80))
        self.dynamic_replan_limit = max(1, int(rospy.get_param("~dynamic_replan_limit", 2)))
        self.dynamic_replan_window = float(rospy.get_param("~dynamic_replan_window", 4.0))
        self.dynamic_wait_clear_hold = float(rospy.get_param("~dynamic_wait_clear_hold", 1.0))
        self.rotate_clearance = float(rospy.get_param("~rotate_clearance", 0.80))
        self.max_linear_accel = float(rospy.get_param("~max_linear_accel", 0.30))
        self.max_angular_accel = float(rospy.get_param("~max_angular_accel", 1.20))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.60))
        self.cloud_timeout = float(rospy.get_param("~cloud_timeout", 1.00))
        self.require_cloud = parse_bool(rospy.get_param("~require_cloud", True))
        self.enabled = parse_bool(rospy.get_param("~enabled", False))

        self.pose: Optional[Pose2D] = None
        self.pose_history = deque(maxlen=250)
        self.last_odom_receipt = rospy.Time(0)
        self.last_cloud_receipt = rospy.Time(0)
        self.last_raw_livox_receipt = rospy.Time(0)
        self.raw_min_front_clearance = float("inf")
        self.raw_min_radial = float("inf")
        self.obstacle_voxels: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
        self.progress_s = 0.0
        self.current_zone_index = -1
        self.local_path: List[Point] = []
        self.local_rejoin_s = 0.0
        self.local_side = "none"
        self.state = "WAIT_DATA"
        self.last_state_detail = ""
        self.last_plan_time = rospy.Time(0)
        self.last_cmd_time = rospy.Time.now()
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0
        self.last_obstacle_publish = rospy.Time(0)
        self.last_pose_visualization = rospy.Time(0)
        self.safety_clear_since = None
        self.dynamic_replan_events = deque()
        self.dynamic_clear_since = None

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.reference_path_pub = rospy.Publisher("~reference_path", Path, queue_size=1, latch=True)
        self.local_path_pub = rospy.Publisher("~local_path", Path, queue_size=1, latch=True)
        self.obstacle_pub = rospy.Publisher("~obstacles", PointCloud2, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.path_marker_pub = rospy.Publisher("~path_markers", MarkerArray, queue_size=1, latch=True)
        self.vehicle_pose_pub = rospy.Publisher("~vehicle_pose", PoseStamped, queue_size=1)
        self.target_pose_pub = rospy.Publisher("~target_pose", PoseStamped, queue_size=1, latch=True)
        self.vehicle_footprint_pub = rospy.Publisher("~vehicle_footprint", Marker, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)
        if self.use_raw_livox_safety:
            try:
                from livox_ros_driver2.msg import CustomMsg
            except ImportError as error:
                raise RuntimeError(
                    "Raw Livox safety requested but livox_ros_driver2 Python messages are unavailable; "
                    "source ~/livox_ws/devel/setup.bash before ~/fastlio_ws/devel/setup.bash"
                ) from error
            rospy.Subscriber(self.raw_livox_topic, CustomMsg, self.raw_livox_callback, queue_size=1)
        rospy.Subscriber("~enable", Bool, self.enable_callback, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)

        self.publish_path(self.reference_path_pub, self.reference.points)
        self.publish_navigation_markers()
        self.set_state("WAIT_DATA", "waiting_for_odometry_and_pointcloud")
        rospy.loginfo(
            "Static obstacle avoidance ready: vehicle=%.2fx%.2f lidar=(%.2f,%.2f,%.2f) path=%.2fm cmd=%s",
            self.vehicle.length,
            self.vehicle.width,
            self.vehicle.lidar_x,
            self.vehicle.lidar_y,
            self.vehicle.lidar_z,
            self.reference.length,
            self.cmd_topic,
        )
        if self.zone_mode:
            for index, zone in enumerate(self.avoidance_zones):
                rospy.loginfo("Avoidance zone %d: s=[%.2f, %.2f]", index + 1, zone[0], zone[1])
        else:
            rospy.logwarn("zone_mode is disabled: obstacle planning is active along the complete route")
        if self.planner_config.lane_half_width <= 0.0:
            rospy.logwarn("lane_half_width is disabled; set the measured legal corridor width before competition")
        if self.use_raw_livox_safety:
            rospy.loginfo("Close-range raw Livox safety enabled on %s", self.raw_livox_topic)

    @staticmethod
    def load_csv_route(csv_path: str) -> Tuple[List[Point], List[str]]:
        points: List[Point] = []
        tasks: List[str] = []
        with open(csv_path, "r", newline="") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames or "x" not in reader.fieldnames or "y" not in reader.fieldnames:
                raise RuntimeError("CSV must contain x and y columns")
            for row in reader:
                try:
                    points.append((float(row["x"]), float(row["y"])))
                    tasks.append(str(row.get("task", "none") or "none").strip().lower())
                except (TypeError, ValueError, KeyError):
                    continue
        if len(points) < 2:
            raise RuntimeError("CSV contains fewer than two valid path points")
        return points, tasks

    @staticmethod
    def load_csv_path(csv_path: str) -> List[Point]:
        points, _ = StaticObstacleAvoidanceNode.load_csv_route(csv_path)
        return points

    def build_avoidance_zones(
        self, raw_points: Sequence[Point], tasks: Sequence[str]
    ) -> List[Tuple[float, float]]:
        try:
            return extract_avoidance_zones(
                self.reference,
                raw_points,
                tasks,
                self.avoid_start_task,
                self.avoid_end_task,
            )
        except ValueError as error:
            raise RuntimeError(str(error))

    def avoidance_zone_index(self, progress_s: float) -> int:
        if not self.zone_mode:
            return 0
        for index, (start_s, end_s) in enumerate(self.avoidance_zones):
            if start_s - self.zone_enter_margin <= progress_s <= end_s + self.zone_exit_margin:
                return index
        return -1

    def set_state(self, state: str, detail: str = ""):
        if state == self.state and detail == self.last_state_detail:
            return
        self.state = state
        self.last_state_detail = detail
        payload = {
            "state": state,
            "detail": detail,
            "progress_s": round(self.progress_s, 3),
            "rejoin_s": round(self.local_rejoin_s, 3),
            "side": self.local_side,
            "zone_active": self.current_zone_index >= 0 or not self.zone_mode,
            "zone_index": self.current_zone_index + 1 if self.current_zone_index >= 0 else 0,
        }
        self.state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))
        rospy.loginfo("Obstacle navigation state: %s | %s", state, detail)

    def enable_callback(self, message: Bool):
        with self.lock:
            self.enabled = bool(message.data)
            if not self.enabled:
                self.local_path = []
                self.dynamic_replan_events.clear()
                self.dynamic_clear_since = None
                self.publish_path(self.local_path_pub, [])
                self.publish_navigation_markers()
                self.set_state("DISABLED", "enable_topic_false")

    def odom_callback(self, message: Odometry):
        pose = message.pose.pose
        lidar_pose = Pose2D(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        base_pose = base_pose_from_lidar_odometry(lidar_pose, self.vehicle) if self.odom_pose_is_lidar else lidar_pose
        stamp = message.header.stamp if message.header.stamp != rospy.Time(0) else rospy.Time.now()
        with self.lock:
            self.pose = base_pose
            self.pose_history.append((stamp.to_sec(), base_pose))
            self.last_odom_receipt = rospy.Time.now()
        self.publish_vehicle_visualization(base_pose)

    def pose_at_stamp(self, stamp: rospy.Time) -> Optional[Pose2D]:
        with self.lock:
            if not self.pose_history:
                return self.pose
            if stamp == rospy.Time(0):
                return self.pose_history[-1][1]
            target = stamp.to_sec()
            nearest = min(self.pose_history, key=lambda item: abs(item[0] - target))
            if abs(nearest[0] - target) > 0.25:
                rospy.logwarn_throttle(2.0, "Cloud/odometry timestamp difference exceeds 0.25 s")
            return nearest[1]

    def cloud_callback(self, message: PointCloud2):
        base_pose = self.pose_at_stamp(message.header.stamp)
        if base_pose is None:
            return

        voxel_size = max(0.03, self.obstacle_voxel_size)
        frame_voxels: Dict[Tuple[int, int], List[float]] = {}
        raw_count = 0
        kept_count = 0
        for point in pc2.read_points(message, field_names=("x", "y", "z"), skip_nans=True):
            raw_count += 1
            if raw_count > self.max_cloud_points:
                break
            if raw_count % self.cloud_stride != 0:
                continue
            x_lidar, y_lidar, z_lidar = float(point[0]), float(point[1]), float(point[2])
            x_base = x_lidar + self.vehicle.lidar_x
            y_base = y_lidar + self.vehicle.lidar_y
            z_base = z_lidar + self.vehicle.lidar_z
            if not self.min_obstacle_height <= z_base <= self.max_obstacle_height:
                continue
            planar_range = math.hypot(x_lidar, y_lidar)
            if planar_range < self.cloud_min_range or planar_range > self.cloud_max_range:
                continue
            if x_base < self.cloud_forward_min or x_base > self.cloud_forward_max:
                continue
            if abs(y_base) > self.cloud_lateral_max:
                continue
            if (
                abs(x_base) <= self.vehicle.half_length + self.self_filter_padding
                and abs(y_base) <= self.vehicle.half_width + self.self_filter_padding
            ):
                continue

            x_world, y_world, _ = lidar_point_to_world((x_lidar, y_lidar, z_lidar), base_pose, self.vehicle)
            key = (int(math.floor(x_world / voxel_size)), int(math.floor(y_world / voxel_size)))
            value = frame_voxels.setdefault(key, [0.0, 0.0, 0.0])
            value[0] += x_world
            value[1] += y_world
            value[2] += 1.0
            kept_count += 1

        now_sec = rospy.Time.now().to_sec()
        accepted = 0
        with self.lock:
            for key, value in frame_voxels.items():
                count = int(value[2])
                if count < self.min_points_per_voxel:
                    continue
                self.obstacle_voxels[key] = (value[0] / count, value[1] / count, now_sec)
                accepted += 1
            self.last_cloud_receipt = rospy.Time.now()
            self.prune_obstacles_locked(now_sec, base_pose)
        rospy.loginfo_throttle(
            2.0,
            "Obstacle cloud: raw=%d height/range=%d accepted_voxels=%d memory=%d",
            raw_count,
            kept_count,
            accepted,
            len(self.obstacle_voxels),
        )

    def raw_livox_callback(self, message):
        sampled_points = []
        for index, point in enumerate(message.points):
            if index % self.raw_livox_stride != 0:
                continue
            sampled_points.append((float(point.x), float(point.y), float(point.z)))
        min_front, min_radial = raw_lidar_safety_distances(
            sampled_points,
            self.vehicle,
            self.min_obstacle_height,
            self.max_obstacle_height,
            self.raw_livox_max_range,
            self.safety_lateral_padding,
            self.self_filter_padding,
            self.raw_safety_min_points,
        )
        with self.lock:
            self.raw_min_front_clearance = min_front
            self.raw_min_radial = min_radial
            self.last_raw_livox_receipt = rospy.Time.now()

    def prune_obstacles_locked(self, now_sec: float, pose: Pose2D):
        radius_sq = self.obstacle_memory_radius * self.obstacle_memory_radius
        expired = []
        for key, value in self.obstacle_voxels.items():
            too_old = now_sec - value[2] > self.obstacle_memory_time
            too_far = (value[0] - pose.x) ** 2 + (value[1] - pose.y) ** 2 > radius_sq
            if too_old or too_far:
                expired.append(key)
        for key in expired:
            self.obstacle_voxels.pop(key, None)
        if len(self.obstacle_voxels) > self.max_obstacle_voxels:
            ordered = sorted(self.obstacle_voxels.items(), key=lambda item: item[1][2], reverse=True)
            self.obstacle_voxels = dict(ordered[: self.max_obstacle_voxels])

    def obstacle_snapshot(self) -> List[Point]:
        with self.lock:
            if self.pose is None:
                return []
            self.prune_obstacles_locked(rospy.Time.now().to_sec(), self.pose)
            return [(value[0], value[1]) for value in self.obstacle_voxels.values()]

    def publish_path(self, publisher, points: Sequence[Point]):
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.world_frame
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            if len(points) > 1:
                neighbor = points[min(index + 1, len(points) - 1)] if index < len(points) - 1 else points[index - 1]
                dx = neighbor[0] - point[0] if index < len(points) - 1 else point[0] - neighbor[0]
                dy = neighbor[1] - point[1] if index < len(points) - 1 else point[1] - neighbor[1]
                yaw = math.atan2(dy, dx)
                pose.pose.orientation.z = math.sin(0.5 * yaw)
                pose.pose.orientation.w = math.cos(0.5 * yaw)
            else:
                pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        publisher.publish(message)

    def make_pose_stamped(self, point: Point, yaw: float, stamp=None) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = stamp or rospy.Time.now()
        message.header.frame_id = self.world_frame
        message.pose.position.x = point[0]
        message.pose.position.y = point[1]
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        return message

    @staticmethod
    def sampled_marker_indices(points: Sequence[Point], spacing: float) -> List[int]:
        if not points:
            return []
        indices = [0]
        accumulated = 0.0
        for index in range(1, len(points)):
            accumulated += math.hypot(
                points[index][0] - points[index - 1][0],
                points[index][1] - points[index - 1][1],
            )
            if accumulated >= spacing:
                indices.append(index)
                accumulated = 0.0
        if indices[-1] != len(points) - 1:
            indices.append(len(points) - 1)
        return indices

    @staticmethod
    def set_marker_color(marker: Marker, red: float, green: float, blue: float, alpha: float = 1.0):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def append_path_markers(
        self,
        marker_array: MarkerArray,
        points: Sequence[Point],
        namespace: str,
        line_id: int,
        color: Tuple[float, float, float],
        line_width: float,
    ):
        if len(points) < 2:
            return
        stamp = rospy.Time.now()
        yaws = self.planner._path_yaws(points)
        marker_indices = self.sampled_marker_indices(points, self.marker_spacing)

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = self.world_frame
        line.ns = namespace + "_line"
        line.id = line_id
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = line_width
        self.set_marker_color(line, color[0], color[1], color[2], 0.95)
        line.points = [RosPoint(x=point[0], y=point[1], z=0.06) for point in points]
        marker_array.markers.append(line)

        waypoints = Marker()
        waypoints.header = line.header
        waypoints.ns = namespace + "_waypoints"
        waypoints.id = line_id + 1
        waypoints.type = Marker.SPHERE_LIST
        waypoints.action = Marker.ADD
        waypoints.pose.orientation.w = 1.0
        waypoints.scale.x = 0.09
        waypoints.scale.y = 0.09
        waypoints.scale.z = 0.09
        self.set_marker_color(waypoints, color[0], color[1], color[2], 0.95)
        waypoints.points = [RosPoint(x=points[index][0], y=points[index][1], z=0.10) for index in marker_indices]
        marker_array.markers.append(waypoints)

        for arrow_id, index in enumerate(marker_indices):
            yaw = yaws[index]
            start = RosPoint(x=points[index][0], y=points[index][1], z=0.13)
            end = RosPoint(
                x=start.x + self.marker_arrow_length * math.cos(yaw),
                y=start.y + self.marker_arrow_length * math.sin(yaw),
                z=start.z,
            )
            arrow = Marker()
            arrow.header = line.header
            arrow.ns = namespace + "_heading"
            arrow.id = arrow_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = 0.025
            arrow.scale.y = 0.065
            arrow.scale.z = 0.080
            self.set_marker_color(arrow, color[0], color[1], color[2], 1.0)
            arrow.points = [start, end]
            marker_array.markers.append(arrow)

    def publish_navigation_markers(self):
        marker_array = MarkerArray()
        clear = Marker()
        clear.header.stamp = rospy.Time.now()
        clear.header.frame_id = self.world_frame
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        self.append_path_markers(
            marker_array,
            self.reference.points,
            "reference",
            0,
            (0.15, 0.55, 1.00),
            0.045,
        )
        self.append_path_markers(
            marker_array,
            self.local_path,
            "detour",
            1000,
            (1.00, 0.35, 0.05),
            0.085,
        )
        self.append_zone_markers(marker_array)
        self.path_marker_pub.publish(marker_array)

    def append_zone_markers(self, marker_array: MarkerArray):
        if not self.zone_mode:
            return
        stamp = rospy.Time.now()
        for zone_index, (start_s, end_s) in enumerate(self.avoidance_zones):
            for boundary_index, (s_value, label, color) in enumerate(
                (
                    (start_s, "AVOID START {}".format(zone_index + 1), (0.10, 1.00, 0.25)),
                    (end_s, "AVOID END {}".format(zone_index + 1), (0.85, 0.15, 1.00)),
                )
            ):
                x, y, yaw = self.reference.sample(s_value)
                marker_id = zone_index * 10 + boundary_index

                boundary = Marker()
                boundary.header.stamp = stamp
                boundary.header.frame_id = self.world_frame
                boundary.ns = "avoid_zone_boundary"
                boundary.id = marker_id
                boundary.type = Marker.CUBE
                boundary.action = Marker.ADD
                boundary.pose.position.x = x
                boundary.pose.position.y = y
                boundary.pose.position.z = 0.04
                boundary.pose.orientation.z = math.sin(0.5 * yaw)
                boundary.pose.orientation.w = math.cos(0.5 * yaw)
                boundary.scale.x = 0.10
                boundary.scale.y = max(0.90, self.vehicle.width + 0.30)
                boundary.scale.z = 0.08
                self.set_marker_color(boundary, color[0], color[1], color[2], 0.80)
                marker_array.markers.append(boundary)

                text_marker = Marker()
                text_marker.header = boundary.header
                text_marker.ns = "avoid_zone_label"
                text_marker.id = marker_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = x
                text_marker.pose.position.y = y
                text_marker.pose.position.z = 0.65
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.25
                text_marker.text = label
                self.set_marker_color(text_marker, color[0], color[1], color[2], 1.0)
                marker_array.markers.append(text_marker)

    def publish_vehicle_visualization(self, pose: Pose2D):
        now = rospy.Time.now()
        if (now - self.last_pose_visualization).to_sec() < 1.0 / self.pose_visualization_rate:
            return
        self.last_pose_visualization = now
        self.vehicle_pose_pub.publish(self.make_pose_stamped((pose.x, pose.y), pose.yaw, now))

        footprint = Marker()
        footprint.header.stamp = now
        footprint.header.frame_id = self.world_frame
        footprint.ns = "bunker_mini"
        footprint.id = 0
        footprint.type = Marker.CUBE
        footprint.action = Marker.ADD
        footprint.pose.position.x = pose.x
        footprint.pose.position.y = pose.y
        footprint.pose.position.z = 0.04
        footprint.pose.orientation.z = math.sin(0.5 * pose.yaw)
        footprint.pose.orientation.w = math.cos(0.5 * pose.yaw)
        footprint.scale.x = self.vehicle.length
        footprint.scale.y = self.vehicle.width
        footprint.scale.z = 0.08
        self.set_marker_color(footprint, 0.10, 0.85, 0.25, 0.45)
        self.vehicle_footprint_pub.publish(footprint)

    def publish_tracking_target(self, points: Sequence[Point], target: Point):
        index = self.nearest_path_index(points, Pose2D(target[0], target[1], 0.0))
        if len(points) < 2:
            yaw = 0.0
        elif index < len(points) - 1:
            yaw = math.atan2(points[index + 1][1] - points[index][1], points[index + 1][0] - points[index][0])
        else:
            yaw = math.atan2(points[index][1] - points[index - 1][1], points[index][0] - points[index - 1][0])
        self.target_pose_pub.publish(self.make_pose_stamped(target, yaw))

    def publish_obstacles(self, obstacles: Sequence[Point]):
        # create_cloud_xyz32 requires a std_msgs/Header instance; importing it
        # lazily keeps the rest of the node's public interface compact.
        from std_msgs.msg import Header

        header = Header(stamp=rospy.Time.now(), frame_id=self.world_frame)
        self.obstacle_pub.publish(pc2.create_cloud_xyz32(header, [(x, y, 0.10) for x, y in obstacles]))

    def publish_obstacles_if_due(self, obstacles: Sequence[Point]):
        interval = 1.0 / max(0.1, self.obstacle_publish_rate)
        now = rospy.Time.now()
        if (now - self.last_obstacle_publish).to_sec() < interval:
            return
        self.publish_obstacles(obstacles)
        self.last_obstacle_publish = now

    def sensor_status(self) -> Optional[str]:
        now = rospy.Time.now()
        if self.pose is None or self.last_odom_receipt == rospy.Time(0):
            return "no_odometry"
        if (now - self.last_odom_receipt).to_sec() > self.odom_timeout:
            return "odometry_stale"
        if self.require_cloud:
            if self.last_cloud_receipt == rospy.Time(0):
                return "no_pointcloud"
            if (now - self.last_cloud_receipt).to_sec() > self.cloud_timeout:
                return "pointcloud_stale"
        if self.use_raw_livox_safety:
            if self.last_raw_livox_receipt == rospy.Time(0):
                return "no_raw_livox_safety"
            if (now - self.last_raw_livox_receipt).to_sec() > self.raw_livox_timeout:
                return "raw_livox_safety_stale"
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

    @staticmethod
    def nearest_path_index(points: Sequence[Point], pose: Pose2D) -> int:
        if not points:
            return 0
        return min(range(len(points)), key=lambda index: (points[index][0] - pose.x) ** 2 + (points[index][1] - pose.y) ** 2)

    def remaining_local_path_safe(self, pose: Pose2D, obstacles: Sequence[Point]) -> bool:
        if not self.local_path:
            return False
        start = self.nearest_path_index(self.local_path, pose)
        remaining = self.local_path[start:]
        if len(remaining) < 2:
            return True
        index = ObstacleIndex(obstacles)
        yaws = self.planner._path_yaws(remaining)
        stride = max(1, int(round(0.10 / max(self.planner_config.path_sample_step, 0.04))))
        for point_index in range(0, len(remaining), stride):
            point = remaining[point_index]
            if self.planner.footprint_collision(Pose2D(point[0], point[1], yaws[point_index]), index):
                return False
        return True

    def apply_plan_result(self, result: PlanResult):
        if result.status == "detour":
            self.local_path = list(result.points)
            self.local_rejoin_s = result.rejoin_s
            self.local_side = result.side
            self.publish_path(self.local_path_pub, self.local_path)
            self.publish_navigation_markers()
            detail = "side={} offset={:.2f} cost={:.2f} blocked=[{:.2f},{:.2f}]".format(
                result.side,
                result.lateral_offset,
                result.cost,
                result.blocked_start_s,
                result.blocked_end_s,
            )
            self.set_state("FOLLOW_DETOUR", detail)
        elif result.status == "clear":
            self.local_path = []
            self.local_rejoin_s = 0.0
            self.local_side = "none"
            self.publish_path(self.local_path_pub, [])
            self.publish_navigation_markers()
            self.set_state("FOLLOW_REFERENCE", result.reason)
        else:
            self.local_path = []
            self.local_rejoin_s = 0.0
            self.local_side = "none"
            self.publish_path(self.local_path_pub, [])
            self.publish_navigation_markers()
            self.set_state("BLOCKED", result.reason)

    def plan_now(self, pose: Pose2D, obstacles: Sequence[Point]):
        result = self.planner.plan(pose, self.progress_s, obstacles)
        self.last_plan_time = rospy.Time.now()
        self.apply_plan_result(result)

    def record_detour_invalidation(self) -> bool:
        now_sec = rospy.Time.now().to_sec()
        self.dynamic_replan_events.append(now_sec)
        while self.dynamic_replan_events and now_sec - self.dynamic_replan_events[0] > self.dynamic_replan_window:
            self.dynamic_replan_events.popleft()
        return len(self.dynamic_replan_events) >= self.dynamic_replan_limit

    def enter_dynamic_wait(self, detail: str):
        self.local_path = []
        self.local_rejoin_s = 0.0
        self.local_side = "none"
        self.dynamic_clear_since = None
        self.publish_path(self.local_path_pub, [])
        self.publish_navigation_markers()
        self.set_state("DYNAMIC_WAIT", detail)
        self.stop()

    def handle_dynamic_wait(self, pose: Pose2D, obstacles: Sequence[Point]) -> bool:
        result = self.planner.plan(pose, self.progress_s, obstacles)
        if result.status != "clear":
            self.dynamic_clear_since = None
            self.stop()
            return True
        now = rospy.Time.now()
        if self.dynamic_clear_since is None:
            self.dynamic_clear_since = now
        if (now - self.dynamic_clear_since).to_sec() < self.dynamic_wait_clear_hold:
            self.stop()
            return True
        self.dynamic_clear_since = None
        self.dynamic_replan_events.clear()
        self.set_state("FOLLOW_REFERENCE", "dynamic_obstacle_cleared")
        return False

    def detour_rejoined(self, pose: Pose2D, projection) -> bool:
        if not self.local_path:
            return True
        end_x, end_y = self.local_path[-1]
        distance_to_end = math.hypot(end_x - pose.x, end_y - pose.y)
        yaw_error = abs(wrap_angle(pose.yaw - projection.yaw))
        path_conditions = (
            projection.s >= self.local_rejoin_s - 0.12
            and abs(projection.d) <= self.rejoin_lateral_tolerance
            and yaw_error <= self.rejoin_yaw_tolerance
        )
        return distance_to_end <= self.goal_tolerance or path_conditions

    def active_tracking_path(self, pose: Pose2D) -> List[Point]:
        if self.local_path:
            return self.local_path
        end_s = min(self.reference.length, self.progress_s + max(4.0, self.planner_config.planning_horizon))
        return self.reference.segment(self.progress_s, end_s, self.planner_config.path_sample_step, pose)

    def lookahead_target(self, points: Sequence[Point], pose: Pose2D, lookahead: float) -> Optional[Point]:
        if not points:
            return None
        nearest = self.nearest_path_index(points, pose)
        accumulated = 0.0
        previous = (pose.x, pose.y)
        for index in range(nearest, len(points)):
            point = points[index]
            accumulated += math.hypot(point[0] - previous[0], point[1] - previous[1])
            if accumulated >= lookahead:
                return point
            previous = point
        return points[-1]

    def safety_distances(self, pose: Pose2D, obstacles: Sequence[Point]) -> Tuple[float, float]:
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        min_front = float("inf")
        min_radial = float("inf")
        lateral_limit = self.vehicle.half_width + self.safety_lateral_padding
        for ox, oy in obstacles:
            dx = ox - pose.x
            dy = oy - pose.y
            xb = cosine * dx + sine * dy
            yb = -sine * dx + cosine * dy
            min_radial = min(min_radial, math.hypot(dx, dy))
            front_clear = xb - self.vehicle.half_length
            if 0.0 <= front_clear <= self.safety_slow_distance and abs(yb) <= lateral_limit:
                min_front = min(min_front, front_clear)
        if self.use_raw_livox_safety:
            min_front = min(min_front, self.raw_min_front_clearance)
            min_radial = min(min_radial, self.raw_min_radial)
        return min_front, min_radial

    def publish_command(self, linear: float, angular: float, immediate_stop: bool = False):
        now = rospy.Time.now()
        dt = max(0.001, (now - self.last_cmd_time).to_sec())
        if immediate_stop:
            linear = 0.0
            angular = 0.0
        else:
            linear = clamp(
                linear,
                self.last_linear_cmd - self.max_linear_accel * dt,
                self.last_linear_cmd + self.max_linear_accel * dt,
            )
            angular = clamp(
                angular,
                self.last_angular_cmd - self.max_angular_accel * dt,
                self.last_angular_cmd + self.max_angular_accel * dt,
            )
        message = Twist()
        message.linear.x = clamp(linear, -self.max_linear, self.max_linear)
        message.angular.z = clamp(angular, -self.max_angular, self.max_angular)
        self.cmd_pub.publish(message)
        self.last_linear_cmd = message.linear.x
        self.last_angular_cmd = message.angular.z
        self.last_cmd_time = now

    def stop(self):
        self.publish_command(0.0, 0.0, immediate_stop=True)

    def track(self, pose: Pose2D, obstacles: Sequence[Point]):
        points = self.active_tracking_path(pose)
        speed = clamp(self.target_speed, 0.0, self.max_linear)
        lookahead = clamp(self.lookahead + 0.8 * speed, self.min_lookahead, self.max_lookahead)
        target = self.lookahead_target(points, pose, lookahead)
        if target is None:
            self.stop()
            return
        self.publish_tracking_target(points, target)

        dx = target[0] - pose.x
        dy = target[1] - pose.y
        distance = max(0.01, math.hypot(dx, dy))
        alpha = wrap_angle(math.atan2(dy, dx) - pose.yaw)
        curvature = 2.0 * math.sin(alpha) / distance
        angular = clamp(speed * curvature, -self.max_angular, self.max_angular)
        linear = speed / (1.0 + 0.9 * abs(curvature))

        min_front, min_radial = self.safety_distances(pose, obstacles)
        if min_front <= self.safety_stop_distance:
            self.safety_clear_since = None
            self.set_state("SAFETY_STOP", "front_clearance={:.3f}".format(min_front))
            self.stop()
            return
        if self.state == "SAFETY_STOP":
            if min_front < self.safety_slow_distance:
                self.safety_clear_since = None
                self.stop()
                return
            if self.safety_clear_since is None:
                self.safety_clear_since = rospy.Time.now()
            if (rospy.Time.now() - self.safety_clear_since).to_sec() < self.safety_clear_hold_time:
                self.stop()
                return
            self.safety_clear_since = None
            self.set_state(
                "FOLLOW_DETOUR" if self.local_path else "FOLLOW_REFERENCE",
                "safety_clear",
            )
        if min_front < self.safety_slow_distance:
            scale = (min_front - self.safety_stop_distance) / max(
                0.01, self.safety_slow_distance - self.safety_stop_distance
            )
            linear *= clamp(scale, 0.0, 1.0)

        if abs(alpha) > 1.0:
            if min_radial >= self.rotate_clearance:
                linear = 0.0
                angular = clamp(0.55 * alpha, -0.45, 0.45)
            else:
                self.set_state("SAFETY_STOP", "large_heading_error_and_rotation_not_safe")
                self.stop()
                return
        elif abs(alpha) > 0.65:
            linear *= 0.45

        self.publish_command(linear, angular)

    def control_step(self):
        with self.lock:
            if not self.enabled:
                self.set_state("DISABLED", "navigation_disabled")
                self.stop()
                return

            sensor_error = self.sensor_status()
            if sensor_error is not None:
                self.set_state("SENSOR_STOP", sensor_error)
                self.stop()
                return

            pose = self.pose
            if pose is None:
                self.stop()
                return
            projection = self.update_progress(pose)
            obstacles = self.obstacle_snapshot()
            self.publish_obstacles_if_due(obstacles)
            previous_zone_index = self.current_zone_index
            detected_zone_index = self.avoidance_zone_index(self.progress_s)
            if detected_zone_index >= 0:
                self.current_zone_index = detected_zone_index
            elif not self.local_path:
                self.current_zone_index = -1
            if previous_zone_index < 0 <= self.current_zone_index:
                self.set_state("FOLLOW_REFERENCE", "avoidance_zone_enter")
            elif previous_zone_index >= 0 and self.current_zone_index < 0:
                self.set_state("FOLLOW_REFERENCE", "avoidance_zone_exit")

            if self.progress_s >= self.reference.length - 0.05:
                goal_x, goal_y, _ = self.reference.sample(self.reference.length)
                if math.hypot(goal_x - pose.x, goal_y - pose.y) <= self.goal_tolerance:
                    self.set_state("FINISHED", "reference_path_complete")
                    self.stop()
                    return

            if self.state == "DYNAMIC_WAIT" and self.handle_dynamic_wait(pose, obstacles):
                return

            if self.local_path:
                if self.detour_rejoined(pose, projection):
                    self.progress_s = max(self.progress_s, self.local_rejoin_s)
                    self.local_path = []
                    self.local_rejoin_s = 0.0
                    self.local_side = "none"
                    self.dynamic_replan_events.clear()
                    self.publish_path(self.local_path_pub, [])
                    self.publish_navigation_markers()
                    self.set_state("FOLLOW_REFERENCE", "detour_rejoined")
                elif not self.remaining_local_path_safe(pose, obstacles):
                    rospy.logwarn("Current detour became occupied; stopping and replanning")
                    self.stop()
                    if self.record_detour_invalidation():
                        self.enter_dynamic_wait("repeated_detour_occupation")
                        return
                    self.plan_now(pose, obstacles)
                    if self.state == "BLOCKED":
                        return

            avoidance_active = (not self.zone_mode) or self.current_zone_index >= 0 or bool(self.local_path)
            if not avoidance_active:
                if self.state == "BLOCKED":
                    self.set_state("FOLLOW_REFERENCE", "outside_avoidance_zone")
                elif self.state not in ("FOLLOW_REFERENCE", "SAFETY_STOP"):
                    self.set_state("FOLLOW_REFERENCE", "outside_avoidance_zone")
                self.track(pose, obstacles)
                return

            now = rospy.Time.now()
            plan_interval = 1.0 / max(0.1, self.blocked_retry_rate if self.state in ("BLOCKED", "SAFETY_STOP") else self.plan_rate)
            plan_due = (now - self.last_plan_time).to_sec() >= plan_interval
            if not self.local_path and plan_due:
                self.plan_now(pose, obstacles)
                if self.state == "BLOCKED":
                    self.stop()
                    return

            if self.state in ("BLOCKED", "DYNAMIC_WAIT", "SENSOR_STOP", "DISABLED", "FINISHED", "ERROR_STOP"):
                self.stop()
                return

            self.track(pose, obstacles)

    def run(self):
        rate = rospy.Rate(max(1.0, self.control_rate))
        while not rospy.is_shutdown():
            try:
                self.control_step()
            except Exception as error:  # Safety boundary: an unexpected error must stop the vehicle.
                rospy.logerr_throttle(1.0, "Static obstacle control exception: %s", error)
                self.set_state("ERROR_STOP", str(error))
                self.stop()
            rate.sleep()

    def on_shutdown(self):
        self.stop()
        rospy.sleep(0.05)
        self.stop()


if __name__ == "__main__":
    try:
        StaticObstacleAvoidanceNode().run()
    except rospy.ROSInterruptException:
        pass
