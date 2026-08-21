#!/usr/bin/env python3
import math
import os
import sys
import threading
import unittest
from unittest import mock


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy

import pure_pursuit_astar_follower as follower_module
from avoidance_zone_astar_test import AvoidanceZoneAStarTest
from pure_pursuit_astar_follower import (
    PurePursuitAStarFollower,
    Waypoint,
    base_xy_from_lidar_pose,
    minimum_circular_inflation,
)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PurePursuitAStarSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Unit tests do not start a ROS master/node; wall-clock Time is enough
        # for lock and throttle semantics exercised here.
        rospy.rostime.set_rostime_initialized(True)

    def make_follower(self):
        node = PurePursuitAStarFollower.__new__(PurePursuitAStarFollower)
        node.current_x = 0.0
        node.current_y = 0.0
        node.current_yaw = 0.0
        node.footprint_front = 0.40
        node.footprint_rear = 0.40
        node.footprint_half_width = 0.35
        node.self_filter_front = 0.345
        node.self_filter_rear = 0.345
        node.self_filter_half_width = 0.285
        node.self_filter_padding = 0.0
        node.front_half_width = 0.42
        node.safety_slow_dist = 0.70
        node.side_safety_radius = 0.25
        node.side_stop_dist = 0.08
        node.rotation_stop_clearance = 0.08
        node.rotation_sweep_angle = 0.35
        node.rotation_sweep_step = 0.05
        node.escape_angular = 0.28
        node.detour_lock_time = 4.0
        node.prefix_backtrack_tolerance = 0.05
        node.plan_accept_translation_tolerance = 0.10
        node.plan_accept_yaw_tolerance = 0.10
        node.detour_side_lock = 0
        node.detour_lock_until = rospy.Time(0)
        node.blocked_since = None
        node.planner_enabled = True
        return node

    def test_lidar_pose_and_route_point_convert_to_body_centre(self):
        self.assertEqual(base_xy_from_lidar_pose(1.0, 2.0, 0.0, 0.4, 0.0), (0.6, 2.0))
        x, y = base_xy_from_lidar_pose(1.0, 2.0, math.pi / 2.0, 0.4, 0.0)
        self.assertAlmostEqual(x, 1.0, places=7)
        self.assertAlmostEqual(y, 1.6, places=7)

    def test_minimum_inflation_matches_centred_vehicle_and_stop_envelope(self):
        value = minimum_circular_inflation(
            0.40, 0.40, 0.35, 0.42, 0.12, 0.28, 0.25
        )
        self.assertAlmostEqual(value, math.hypot(0.68, 0.42), places=7)

    def test_astar_cannot_lower_inflation_below_safety_floor(self):
        node = self.make_follower()
        node.grid_resolution = 0.10
        node.xy_margin = 1.0
        node.max_grid_cells = 10000
        node.inflation_radius = 0.80
        node.minimum_inflation_radius = math.hypot(0.68, 0.42)
        node.goal_search_radius = 1.2
        node.local_waypoint_spacing = 0.30
        node.get_obstacles_snapshot = lambda: []
        captured_cells = []
        node.build_inflation_offsets = lambda cells: captured_cells.append(cells) or []
        plan = node.astar_plan(1.0, 0.0, inflation_radius=0.10)
        self.assertTrue(plan)
        # 0.80 m physical inflation plus half a grid diagonal protects against
        # rounding a measured obstacle to its nearest raster cell.
        self.assertEqual(captured_cells, [9])

    def test_continuous_path_clearance_catches_raster_shortcut(self):
        node = self.make_follower()
        self.assertFalse(
            node.path_has_obstacle_clearance(
                (0.0, 0.0), [(2.0, 0.0)], [(1.0, 0.79)], 0.80
            )
        )
        self.assertTrue(
            node.path_has_obstacle_clearance(
                (0.0, 0.0), [(2.0, 0.0)], [(1.0, 0.81)], 0.80
            )
        )

    def test_front_clearance_is_measured_from_real_front_bumper(self):
        node = self.make_follower()
        node.get_obstacles_snapshot = lambda: [(1.0, 0.0)]
        front, side = node.safety_distances()
        self.assertAlmostEqual(front, 0.60, places=7)
        self.assertTrue(math.isinf(side))

    def test_logged_false_stop_geometry_becomes_clear_after_frame_fix(self):
        node = self.make_follower()
        # In the old lidar-as-centre model this world point was reported as
        # 0.26 m clear (0.76 - bogus 0.50 front). From the real body centre it
        # is 1.16 m away and therefore 0.76 m beyond the 0.40 m front bumper.
        node.get_obstacles_snapshot = lambda: [(1.16, 0.0)]
        front, _ = node.safety_distances()
        # safety_distances intentionally reports infinity outside the slow zone.
        self.assertTrue(math.isinf(front))

    def test_active_left_lock_overrides_negative_alpha(self):
        node = self.make_follower()
        node.detour_side_lock = 1
        node.detour_lock_until = rospy.Time.now() + rospy.Duration(10.0)
        self.assertAlmostEqual(node.preferred_escape_turn(-1.0), 0.28, places=7)

    def test_rotation_sweep_blocks_near_body_but_allows_distant_obstacle(self):
        node = self.make_follower()
        node.get_obstacles_snapshot = lambda: [(0.30, 0.40)]
        self.assertFalse(node.rotation_sweep_is_clear(0.28))
        node.get_obstacles_snapshot = lambda: [(1.50, 0.0)]
        self.assertTrue(node.rotation_sweep_is_clear(0.28))

    def test_cloud_filter_removes_ground_and_singleton(self):
        node = self.make_follower()
        node.obstacle_source = "cloud"
        node.has_odom = True
        node.current_sensor_z = -0.13
        node.lidar_z = 0.50
        node.odom_ground_offset = 0.42
        node.min_obstacle_height = 0.06
        node.max_obstacle_height = 1.40
        node.cloud_keep_radius = 12.0
        node.cloud_stride = 1
        node.obstacle_voxel_size = 0.08
        node.obstacle_support_radius = 0.12
        node.min_points_per_voxel = 2
        node.max_obstacle_points = 100
        node.obstacle_lock = threading.Lock()
        node.obstacles_xy = []
        node.obstacle_stamp = rospy.Time(0)
        points = [
            (1.00, 0.00, -0.55),  # measured map ground peak
            (1.50, 0.00, -0.40),  # isolated noise above ground
            (2.01, 0.01, -0.35),
            (2.02, 0.02, -0.34),
        ]
        with mock.patch.object(follower_module.pc2, "read_points", return_value=points):
            node.cloud_callback(object())
        self.assertEqual(node.obstacles_xy, [(2.04, 0.04)])

    def test_cloud_support_crosses_voxel_boundary(self):
        node = self.make_follower()
        node.obstacle_source = "cloud"
        node.has_odom = True
        node.current_sensor_z = -0.13
        node.odom_ground_offset = 0.42
        node.min_obstacle_height = 0.06
        node.max_obstacle_height = 1.40
        node.cloud_keep_radius = 12.0
        node.cloud_stride = 1
        node.obstacle_voxel_size = 0.08
        node.obstacle_support_radius = 0.12
        node.min_points_per_voxel = 2
        node.max_obstacle_points = 100
        node.obstacle_lock = threading.Lock()
        node.obstacles_xy = []
        node.obstacle_stamp = rospy.Time(0)
        # 2.000 is an exact 8 cm grid boundary: these two returns land in
        # adjacent voxels even though they are only 2 mm apart physically.
        points = [(1.999, 0.01, -0.35), (2.001, 0.01, -0.35)]
        with mock.patch.object(follower_module.pc2, "read_points", return_value=points):
            node.cloud_callback(object())
        self.assertEqual(node.obstacles_xy, [(1.96, 0.04), (2.04, 0.04)])

    def test_cloud_voxel_uncertainty_cannot_overestimate_clearance(self):
        node = self.make_follower()
        node.obstacle_source = "cloud"
        node.obstacle_voxel_size = 0.08
        # The represented voxel extends 5.7 cm around its centre. Although the
        # centre is outside the 0.40 m footprint, the voxel reaches inside it.
        node.get_obstacles_snapshot = lambda: [(0.45, 0.0)]
        front, side = node.safety_distances()
        self.assertEqual(front, 0.0)
        self.assertEqual(side, 0.0)

    def test_dense_supported_voxel_is_compressed_to_one_point(self):
        node = self.make_follower()
        node.obstacle_source = "cloud"
        node.has_odom = True
        node.current_sensor_z = -0.13
        node.odom_ground_offset = 0.42
        node.min_obstacle_height = 0.06
        node.max_obstacle_height = 1.40
        node.cloud_keep_radius = 12.0
        node.cloud_stride = 1
        node.obstacle_voxel_size = 0.08
        node.obstacle_support_radius = 0.12
        node.min_points_per_voxel = 2
        node.max_obstacle_points = 100
        node.obstacle_lock = threading.Lock()
        node.obstacles_xy = []
        node.obstacle_stamp = rospy.Time(0)
        points = [
            (2.001 + 0.0001 * i, 0.01 + 0.0001 * i, -0.35)
            for i in range(100)
        ]
        with mock.patch.object(follower_module.pc2, "read_points", return_value=points):
            node.cloud_callback(object())
        self.assertEqual(node.obstacles_xy, [(2.04, 0.04)])

    def test_cloud_cap_keeps_nearest_voxels_regardless_of_input_order(self):
        node = self.make_follower()
        node.obstacle_source = "cloud"
        node.has_odom = True
        node.current_sensor_z = -0.13
        node.odom_ground_offset = 0.42
        node.min_obstacle_height = 0.06
        node.max_obstacle_height = 1.40
        node.cloud_keep_radius = 12.0
        node.cloud_stride = 1
        node.obstacle_voxel_size = 0.08
        node.obstacle_support_radius = 0.12
        node.min_points_per_voxel = 2
        node.max_obstacle_points = 2
        node.obstacle_lock = threading.Lock()
        node.obstacles_xy = []
        node.obstacle_stamp = rospy.Time(0)
        points = [
            (5.01, 0.01, -0.35), (5.02, 0.02, -0.35),
            (1.01, 0.01, -0.35), (1.02, 0.02, -0.35),
            (0.60, 0.01, -0.35), (0.61, 0.02, -0.35),
        ]
        with mock.patch.object(follower_module.pc2, "read_points", return_value=points):
            node.cloud_callback(object())
        xs = sorted(round(point[0], 2) for point in node.obstacles_xy)
        self.assertEqual(xs, [0.60, 1.00])

    def test_self_filter_does_not_hide_obstacle_inside_safety_margin(self):
        node = self.make_follower()
        self.assertFalse(node.inside_robot_self_filter(0.38, 0.0))
        self.assertTrue(node.inside_robot_footprint(0.38, 0.0))

    def test_intrusion_inside_conservative_footprint_has_zero_clearance(self):
        node = self.make_follower()
        for point in ((0.38, 0.0), (0.0, 0.31), (-0.38, 0.0)):
            with self.subTest(point=point):
                self.assertFalse(node.inside_robot_self_filter(*point))
                self.assertTrue(node.inside_robot_footprint(*point))
                node.get_obstacles_snapshot = lambda point=point: [point]
                front, side = node.safety_distances()
                self.assertEqual(front, 0.0)
                self.assertEqual(side, 0.0)
                self.assertEqual(
                    node.safety_hard_stop_reason(front, side),
                    "footprint_intrusion",
                )

    def test_one_centimetre_side_clearance_requires_hard_stop(self):
        node = self.make_follower()
        node.get_obstacles_snapshot = lambda: [(0.0, 0.36)]
        front, side = node.safety_distances()
        self.assertTrue(math.isinf(front))
        self.assertAlmostEqual(side, 0.01, places=7)
        self.assertEqual(
            node.safety_hard_stop_reason(front, side), "side_clearance"
        )

    def test_expired_detour_lock_does_not_flip_while_blocked(self):
        node = self.make_follower()
        node.detour_side_lock = 1
        node.detour_lock_until = rospy.Time.now() - rospy.Duration(1.0)
        node.blocked_since = rospy.Time.now() - rospy.Duration(5.0)
        node.obstacle_side_scores = lambda: (0.0, 100.0)
        self.assertEqual(node.preferred_detour_side(), 1)

        node.detour_side_lock = -1
        node.obstacle_side_scores = lambda: (100.0, 0.0)
        self.assertEqual(node.preferred_detour_side(), -1)

    def test_angular_reversal_emits_zero_cycle(self):
        node = AvoidanceZoneAStarTest.__new__(AvoidanceZoneAStarTest)
        node.control_rate = 20.0
        node.force_stop_cmd = False
        node.force_linear_stop_cmd = False
        node.last_cmd_linear = 0.0
        node.last_cmd_angular = 0.02
        node.last_cmd_time = rospy.Time.now() - rospy.Duration(0.05)
        node.max_linear_accel = 1.0
        node.max_linear_decel = 1.0
        node.heading_stop_decel = 1.5
        node.max_angular_accel = 0.6
        node.cmd_pub = DummyPublisher()

        node.publish_cmd(0.0, -0.28)
        self.assertEqual(node.cmd_pub.messages[-1].angular.z, 0.0)
        node.last_cmd_time = rospy.Time.now() - rospy.Duration(0.05)
        node.publish_cmd(0.0, -0.28)
        self.assertLess(node.cmd_pub.messages[-1].angular.z, 0.0)

    def test_front_escape_reversal_waits_one_control_cycle_at_zero(self):
        node = self.make_follower()
        node.last_cmd_angular = 0.02
        events = []
        node.preferred_escape_turn = lambda alpha: -0.28
        node.stop_robot = lambda: (
            events.append(("cmd", 0.0)),
            setattr(node, "last_cmd_angular", 0.0),
        )
        node.replan_while_blocked = lambda goal: node.stop_robot()
        node.rotation_sweep_is_clear = lambda angular: True
        node.publish_safety_cmd = lambda angular: events.append(("cmd", angular))

        self.assertFalse(node.handle_front_safety_stop(0.0, object()))
        self.assertEqual(events, [("cmd", 0.0)])
        self.assertTrue(node.handle_front_safety_stop(0.0, object()))
        self.assertEqual(events[-2:], [("cmd", 0.0), ("cmd", -0.28)])

    def test_obstacle_freshness_distinguishes_missing_stale_and_fresh_empty(self):
        node = self.make_follower()
        node.obstacle_source = "cloud"
        node.obstacle_timeout = 1.0
        node.obstacle_lock = threading.Lock()
        node.obstacles_xy = []
        node.obstacle_stamp = rospy.Time(0)
        self.assertFalse(node.obstacle_data_fresh())

        node.obstacle_stamp = rospy.Time.now() - rospy.Duration(2.0)
        self.assertFalse(node.obstacle_data_fresh())

        # A received cloud with no accepted obstacle voxels is valid clear data.
        node.obstacle_stamp = rospy.Time.now()
        self.assertTrue(node.obstacle_data_fresh())

    def test_failed_refresh_keeps_last_accepted_path(self):
        node = self.make_follower()
        node.planner_enabled = True
        node.xy_margin = 2.0
        node.inflation_radius = 0.70
        node.global_index = 0
        node.global_waypoints = [Waypoint(0, 2.0, 0.0, 0.0, 0.2), Waypoint(1, 3.0, 0.0, 0.0, 0.2)]
        node.local_path = [(0.5, 0.0), (1.0, 0.0)]
        node.local_index = 0
        node.last_full_plan = list(node.local_path)
        node.full_path_pub = DummyPublisher()
        node.exec_path_pub = DummyPublisher()
        node.local_goal_candidates = lambda waypoint: [(2.0, 0.0, "direct")]
        node.astar_plan = lambda *args, **kwargs: []
        old_path = list(node.local_path)
        self.assertFalse(node.replan_to_global_waypoint(node.global_waypoints[-1]))
        self.assertEqual(node.local_path, old_path)

    def test_replan_rejects_backward_execution_prefix_transactionally(self):
        node = self.make_follower()
        node.xy_margin = 2.0
        node.inflation_radius = 0.80
        node.grid_resolution = 0.10
        node.execute_points = 7
        node.min_valid_plan_points = 2
        node.min_valid_plan_dist = 0.60
        node.min_forward_target = -0.05
        node.global_index = 0
        node.global_waypoints = [
            Waypoint(0, 2.0, 0.0, 0.0, 0.2),
            Waypoint(1, 3.0, 0.0, 0.0, 0.2),
        ]
        node.local_path = [(0.5, 0.0), (1.0, 0.0)]
        node.local_index = 0
        node.last_full_plan = list(node.local_path)
        node.full_path_pub = DummyPublisher()
        node.exec_path_pub = DummyPublisher()
        node.local_goal_candidates = lambda waypoint: [(2.0, 0.0, "direct")]
        backward_prefix = [(-0.1 * i, 0.0) for i in range(1, 8)] + [(1.0, 0.0)]
        def backward_astar(*args, **kwargs):
            self.assertEqual(kwargs["start_xy"], (0.0, 0.0))
            return list(backward_prefix)

        node.astar_plan = backward_astar
        old_path = list(node.local_path)

        self.assertFalse(node.replan_to_global_waypoint(node.global_waypoints[0]))
        self.assertEqual(node.local_path, old_path)

    def test_replan_checks_prefix_against_latest_odom(self):
        node = self.make_follower()
        node.xy_margin = 2.0
        node.inflation_radius = 0.80
        node.grid_resolution = 0.10
        node.execute_points = 6
        node.min_valid_plan_points = 2
        node.min_valid_plan_dist = 0.60
        node.min_forward_target = -0.05
        node.global_index = 0
        node.global_waypoints = [
            Waypoint(0, 2.0, 0.0, 0.0, 0.2),
            Waypoint(1, 3.0, 0.0, 0.0, 0.2),
        ]
        node.local_path = [(0.5, 0.0)]
        node.last_full_plan = list(node.local_path)
        node.full_path_pub = DummyPublisher()
        node.exec_path_pub = DummyPublisher()
        node.local_goal_candidates = lambda waypoint: [(2.0, 0.0, "direct")]
        candidate = [(0.02, 0.0), (0.2, 0.0), (0.4, 0.0),
                     (0.6, 0.0), (0.8, 0.0), (1.0, 0.0)]

        def odom_moves_during_plan(*args, **kwargs):
            node.current_x = 0.08
            return list(candidate)

        node.astar_plan = odom_moves_during_plan
        self.assertFalse(node.replan_to_global_waypoint(node.global_waypoints[0]))
        self.assertEqual(node.local_path, [(0.5, 0.0)])

    def test_failed_blocked_refresh_latches_translation_pending(self):
        node = self.make_follower()
        node.blocked_replan_delay = 0.0
        node.blocked_since = rospy.Time.now() - rospy.Duration(1.0)
        node.blocked_replan_pending = False
        node.last_blocked_replan_stamp = rospy.Time(0)
        node.stop_robot = lambda: None
        node.replan_to_global_waypoint = lambda waypoint: False
        node.replan_while_blocked(object())
        self.assertTrue(node.blocked_replan_pending)

        node.blocked_since = rospy.Time.now() - rospy.Duration(1.0)
        node.replan_to_global_waypoint = lambda waypoint: True
        node.replan_while_blocked(object())
        self.assertFalse(node.blocked_replan_pending)

    def test_blocked_replan_publishes_stop_before_planning(self):
        node = self.make_follower()
        node.blocked_replan_delay = 0.0
        node.blocked_since = rospy.Time.now() - rospy.Duration(1.0)
        node.blocked_replan_pending = True
        node.last_blocked_replan_stamp = rospy.Time(0)
        events = []
        node.stop_robot = lambda: events.append("stop")
        node.replan_to_global_waypoint = lambda waypoint: events.append("replan") or False

        node.replan_while_blocked(object())
        self.assertEqual(events, ["stop", "replan"])


if __name__ == "__main__":
    unittest.main()
