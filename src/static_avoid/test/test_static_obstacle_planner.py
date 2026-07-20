#!/usr/bin/env python3

import math
import unittest

from static_avoid.static_obstacle_planner import (
    ObstacleIndex,
    PlannerConfig,
    Pose2D,
    ReferencePath,
    StaticObstaclePlanner,
    VehicleGeometry,
    base_pose_from_lidar_odometry,
    extract_avoidance_zones,
    lidar_point_to_world,
    raw_lidar_safety_distances,
    smooth_path_points,
)


def straight_path(length=10.0, step=0.10):
    count = int(round(length / step))
    return [(index * step, 0.0) for index in range(count + 1)]


def circular_obstacle(cx, cy, radius=0.30):
    points = []
    for degree in range(0, 360, 6):
        angle = math.radians(degree)
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


class StaticObstaclePlannerTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = VehicleGeometry(length=0.8, width=0.7, lidar_x=0.4, lidar_z=0.5)
        self.reference = ReferencePath(straight_path())
        self.config = PlannerConfig(
            lateral_offsets=(0.70, 0.90, 1.10, 1.20),
            max_lateral_offset=1.20,
        )
        self.planner = StaticObstaclePlanner(self.reference, self.vehicle, self.config)

    def test_clear_reference_path(self):
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, [])
        self.assertEqual(result.status, "clear")
        self.assertGreater(len(result.points), 20)
        self.assertAlmostEqual(result.points[-1][0], 5.0, places=4)
        self.assertAlmostEqual(result.points[-1][1], 0.0, places=6)

    def test_central_obstacle_generates_safe_rejoining_detour(self):
        obstacles = circular_obstacle(3.0, 0.0, 0.30)
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, obstacles)
        self.assertEqual(result.status, "detour", result.reason)
        self.assertIn(result.side, ("left", "right"))
        self.assertGreater(max(abs(y) for _, y in result.points), 0.60)
        self.assertAlmostEqual(result.points[-1][1], 0.0, delta=0.03)

        index = ObstacleIndex(obstacles)
        yaws = self.planner._path_yaws(result.points)
        for point, yaw in zip(result.points, yaws):
            self.assertFalse(self.planner.footprint_collision(Pose2D(point[0], point[1], yaw), index))

    def test_obstacle_on_left_selects_right(self):
        obstacles = circular_obstacle(3.0, 0.0, 0.30)
        obstacles.extend(circular_obstacle(3.0, 0.95, 0.35))
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, obstacles)
        self.assertEqual(result.status, "detour", result.reason)
        self.assertEqual(result.side, "right")

    def test_alternating_multiple_obstacles_use_lattice_fallback(self):
        obstacles = circular_obstacle(2.5, 0.55, 0.28)
        obstacles.extend(circular_obstacle(5.0, -0.55, 0.28))
        result = self.planner.plan(Pose2D(0.0, 0.0, 0.0), 0.0, obstacles)
        self.assertEqual(result.status, "detour", result.reason)
        self.assertEqual(result.reason, "multi_obstacle_lattice")
        self.assertEqual(result.side, "mixed")
        self.assertGreater(result.rejoin_s, self.config.planning_horizon)
        index = ObstacleIndex(obstacles)
        yaws = self.planner._path_yaws(result.points)
        for point, yaw in zip(result.points, yaws):
            self.assertFalse(self.planner.footprint_collision(Pose2D(point[0], point[1], yaw), index))

    def test_narrow_corridor_stops_instead_of_forcing_an_unsafe_detour(self):
        narrow_config = PlannerConfig(
            lateral_offsets=(0.70, 0.90, 1.10, 1.20),
            max_lateral_offset=1.20,
            lane_half_width=0.80,
        )
        planner = StaticObstaclePlanner(self.reference, self.vehicle, narrow_config)
        result = planner.plan(
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            circular_obstacle(3.0, 0.0, 0.30),
        )
        self.assertEqual(result.status, "blocked")
        self.assertEqual(result.points, [])

    def test_lidar_offset_conversions(self):
        base = base_pose_from_lidar_odometry(Pose2D(0.4, 0.0, 0.0), self.vehicle)
        self.assertAlmostEqual(base.x, 0.0, places=6)
        self.assertAlmostEqual(base.y, 0.0, places=6)
        world = lidar_point_to_world((1.0, 0.0, -0.30), base, self.vehicle)
        self.assertAlmostEqual(world[0], 1.4, places=6)
        self.assertAlmostEqual(world[1], 0.0, places=6)
        self.assertAlmostEqual(world[2], 0.20, places=6)

    def test_raw_lidar_front_clearance_uses_front_bumper_and_rejects_single_noise(self):
        obstacle_points = [(0.60, offset, -0.20) for offset in (-0.02, 0.0, 0.02)]
        front, radial = raw_lidar_safety_distances(
            obstacle_points,
            self.vehicle,
            0.06,
            1.40,
            3.0,
            0.12,
            0.05,
            3,
        )
        self.assertAlmostEqual(front, 0.60, places=5)
        self.assertGreater(radial, 0.99)

        noise_front, _ = raw_lidar_safety_distances(
            [(0.20, 0.0, -0.20)],
            self.vehicle,
            0.06,
            1.40,
            3.0,
            0.12,
            0.05,
            3,
        )
        self.assertTrue(math.isinf(noise_front))

    def test_short_smoothing_window_reduces_recording_jitter(self):
        noisy = [(index * 0.1, 0.018 * (-1 if index % 2 else 1)) for index in range(80)]
        smoothed = smooth_path_points(noisy, 5)
        raw_variation = sum(abs(noisy[index][1] - noisy[index - 1][1]) for index in range(1, len(noisy)))
        smooth_variation = sum(abs(smoothed[index][1] - smoothed[index - 1][1]) for index in range(1, len(smoothed)))
        self.assertLess(smooth_variation, raw_variation * 0.5)
        self.assertEqual(smoothed[0], noisy[0])
        self.assertEqual(smoothed[-1], noisy[-1])

    def test_csv_zone_markers_create_forward_interval(self):
        points = straight_path(length=8.0, step=0.2)
        tasks = ["none"] * len(points)
        tasks[5] = "avoid_start"
        tasks[35] = "avoid_end"
        zones = extract_avoidance_zones(ReferencePath(points), points, tasks)
        self.assertEqual(len(zones), 1)
        self.assertAlmostEqual(zones[0][0], 1.0, places=4)
        self.assertAlmostEqual(zones[0][1], 7.0, places=4)

    def test_unpaired_zone_marker_is_rejected(self):
        points = straight_path(length=4.0, step=0.2)
        tasks = ["none"] * len(points)
        tasks[5] = "avoid_start"
        with self.assertRaises(ValueError):
            extract_avoidance_zones(ReferencePath(points), points, tasks)


if __name__ == "__main__":
    unittest.main()
