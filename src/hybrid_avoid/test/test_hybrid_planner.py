#!/usr/bin/env python3

import math
import unittest

from hybrid_avoid.hybrid_planner import (
    CostmapConfig,
    HybridAStarPlanner,
    Pose2D,
    ReferencePath,
    VehicleGeometry,
    extract_avoidance_zones,
    raw_lidar_clearances,
)


def straight_path(length=12.0, step=0.15):
    count = int(round(length / step))
    return [(index * step, 0.0) for index in range(count + 1)]


def circle(cx, cy, radius=0.28):
    return [
        (cx + radius * math.cos(math.radians(degree)), cy + radius * math.sin(math.radians(degree)))
        for degree in range(0, 360, 8)
    ]


class HybridPlannerTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = VehicleGeometry()
        self.reference = ReferencePath(straight_path())
        self.config = CostmapConfig(
            size_x=10.0,
            size_y=10.0,
            resolution=0.10,
            collision_margin=0.12,
            soft_inflation=0.20,
            reference_weight=0.35,
        )
        self.planner = HybridAStarPlanner(self.vehicle, self.config)

    def assert_path_free(self, result):
        self.assertEqual(result.status, "path", result.reason)
        self.assertGreater(len(result.path), 5)
        for x, y in result.path:
            cell = result.costmap.world_to_cell(x, y)
            self.assertIsNotNone(cell)
            self.assertLess(result.costmap.cost(cell), result.costmap.LETHAL)

    def test_clear_path_stays_near_reference(self):
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, [], 4.5)
        self.assert_path_free(result)
        self.assertLess(max(abs(y) for _, y in result.path), 0.12)

    def test_single_obstacle_generates_detour(self):
        result = self.planner.plan(
            Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, circle(2.5, 0.0), 4.5
        )
        self.assert_path_free(result)
        self.assertGreater(max(abs(y) for _, y in result.path), 0.75)

    def test_unknown_count_alternating_obstacles(self):
        obstacles = circle(1.8, 0.35)
        obstacles += circle(3.1, -0.35)
        obstacles += circle(4.0, 0.40)
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, obstacles, 4.6)
        self.assert_path_free(result)

    def test_replan_from_offset_pose_keeps_safe_astar_fallback(self):
        result = self.planner.plan(
            Pose2D(1.674, 0.30, 0.48),
            1.674,
            self.reference,
            circle(3.2, 0.0, 0.30),
            6.174,
        )
        self.assert_path_free(result)

    def test_side_obstacle_does_not_force_reference_detour(self):
        result = self.planner.plan(
            Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, circle(2.5, 2.0), 4.5
        )
        self.assert_path_free(result)
        self.assertLess(max(abs(y) for _, y in result.path), 0.15)

    def test_parallel_indoor_walls_remain_obstacles_without_stopping(self):
        walls = []
        for index in range(-10, 61):
            x_value = 0.10 * index
            walls.extend(((x_value, -1.45), (x_value, 1.45)))
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, walls, 4.5)
        self.assert_path_free(result)
        self.assertLess(max(abs(y) for _, y in result.path), 0.15)

    def test_occupied_exact_goal_selects_free_reference_goal(self):
        result = self.planner.plan(
            Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, circle(3.7, 0.0), 4.5
        )
        self.assert_path_free(result)
        self.assertGreater(result.goal_s, 4.5)

    def test_vertical_route_goal_fits_square_rolling_costmap(self):
        vertical = ReferencePath([(0.0, value) for value in (0.0, 2.0, 4.0, 6.0, 8.0)])
        result = self.planner.plan(Pose2D(0.0, 0.0, math.pi / 2.0), 0.0, vertical, [], 4.5)
        self.assert_path_free(result)
        self.assertAlmostEqual(result.goal_s, 4.5, places=4)

    def test_previous_path_is_reused_only_while_complete_remainder_is_free(self):
        old_result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, [], 4.5)
        self.assert_path_free(old_result)
        clear, remaining, reason = self.planner.remaining_path_status(
            old_result.path, Pose2D(0.4, 0.0, 0.0), old_result.costmap
        )
        self.assertTrue(clear, reason)
        self.assertGreater(remaining, 3.5)

        occupied_result = self.planner.plan(
            Pose2D(0.4, 0.0, 0.0), 0.4, self.reference, circle(2.5, 0.0), 4.9
        )
        clear, _, reason = self.planner.remaining_path_status(
            old_result.path, Pose2D(0.4, 0.0, 0.0), occupied_result.costmap
        )
        self.assertFalse(clear)
        self.assertEqual(reason, "previous_path_occupied")

    def test_unknown_lane_width_uses_finite_fallback_corridor(self):
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, [], 4.5)
        outside = result.costmap.world_to_cell(0.0, self.config.fallback_center_limit + 0.2)
        self.assertIsNotNone(outside)
        self.assertEqual(result.costmap.cost(outside), result.costmap.LETHAL)

    def test_full_wall_reports_blocked(self):
        wall = [(2.2, -4.0 + 0.08 * index) for index in range(101)]
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, self.reference, wall, 4.5)
        self.assertEqual(result.status, "blocked")
        self.assertEqual(result.path, [])

    def test_zone_markers_are_paired(self):
        points = straight_path(8.0, 0.2)
        tasks = ["none"] * len(points)
        tasks[5] = "avoid_start"
        tasks[35] = "avoid_end"
        zones = extract_avoidance_zones(ReferencePath(points), points, tasks)
        self.assertEqual(len(zones), 1)
        self.assertAlmostEqual(zones[0][0], 1.0, places=4)
        self.assertAlmostEqual(zones[0][1], 7.0, places=4)

    def test_unpaired_zone_marker_rejected(self):
        points = straight_path(4.0, 0.2)
        tasks = ["none"] * len(points)
        tasks[4] = "avoid_start"
        with self.assertRaises(ValueError):
            extract_avoidance_zones(ReferencePath(points), points, tasks)

    def test_raw_safety_uses_front_bumper_and_noise_threshold(self):
        points = [(0.60, offset, -0.20) for offset in (-0.02, 0.0, 0.02)]
        front, radial = raw_lidar_clearances(points, self.vehicle, 0.06, 1.40, 3.0, 0.12, 3)
        self.assertAlmostEqual(front, 0.60, places=5)
        self.assertGreater(radial, 0.99)
        noise, _ = raw_lidar_clearances([(0.2, 0.0, -0.2)], self.vehicle, 0.06, 1.4, 3.0, 0.12, 3)
        self.assertTrue(math.isinf(noise))


if __name__ == "__main__":
    unittest.main()
