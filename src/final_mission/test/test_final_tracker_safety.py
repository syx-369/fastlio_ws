#!/usr/bin/env python3
import math
import os
import sys
import unittest
from unittest import mock


FINAL_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
WAYPOINT_SCRIPTS = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "waypoint_tools", "scripts")
)
for directory in (WAYPOINT_SCRIPTS, FINAL_SCRIPTS):
    if directory not in sys.path:
        sys.path.insert(0, directory)

import rospy
import rospkg
from geometry_msgs.msg import Twist

# Preload the sibling package from this source tree before final_tracker asks
# rospkg for an installed path. This keeps isolated and devel-space tests honest.
import avoidance_zone_astar_test  # noqa: F401
import pure_pursuit_astar_follower  # noqa: F401
with mock.patch.object(
    rospkg.RosPack,
    "get_path",
    autospec=True,
    side_effect=lambda _self, name: (
        os.path.dirname(WAYPOINT_SCRIPTS)
        if name == "waypoint_tools"
        else (_ for _ in ()).throw(rospkg.ResourceNotFound(name))
    ),
):
    from final_tracker import FinalTracker, PHASE_BACKTRACK
from pure_pursuit_astar_follower import base_xy_from_lidar_pose


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FinalTrackerSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.rostime.set_rostime_initialized(True)

    def make_tracker(self):
        node = FinalTracker.__new__(FinalTracker)
        node.current_x = 0.0
        node.current_y = 0.0
        node.current_yaw = 0.0
        node.footprint_front = 0.40
        node.footprint_rear = 0.40
        node.footprint_half_width = 0.35
        node.reverse_safety_half_width = 0.40
        node.obstacle_source = "scan"
        node.obstacle_voxel_size = 0.08
        return node

    def test_cached_rear_obstacle_is_used_outside_avoidance_zone(self):
        node = self.make_tracker()
        node.get_obstacles_snapshot = lambda: []  # zone-gated view is empty
        node.get_cached_obstacles_snapshot = lambda: [(-0.55, 0.0)]
        self.assertAlmostEqual(node.rear_clearance(), 0.15, places=7)

    def test_rear_intrusion_is_zero_clearance(self):
        node = self.make_tracker()
        node.get_cached_obstacles_snapshot = lambda: [(-0.38, 0.0)]
        self.assertEqual(node.rear_clearance(), 0.0)

    def test_cloud_voxel_radius_is_subtracted_from_rear_clearance(self):
        node = self.make_tracker()
        node.obstacle_source = "cloud"
        node.get_cached_obstacles_snapshot = lambda: [(-0.65, 0.0)]
        expected = 0.25 - 0.5 * math.sqrt(2.0) * 0.08
        self.assertAlmostEqual(node.rear_clearance(), expected, places=7)

    def test_backtrack_heading_reversal_and_blocked_sweep_hard_stop(self):
        node = self.make_tracker()
        node.backtrack_authorized = True
        node.task_phase = PHASE_BACKTRACK
        node.backtrack_moving = True
        node.reverse_max_angular = 0.12
        node.last_cmd_angular = 0.02
        node.cmd_pub = DummyPublisher()
        node.get_cached_obstacles_snapshot = lambda: []
        node.stop_robot = lambda: node.cmd_pub.publish(Twist())
        node.rotation_sweep_is_clear = lambda *args, **kwargs: True

        node.publish_failure_only_heading_correction(-0.10)
        self.assertEqual(node.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertEqual(node.cmd_pub.messages[-1].angular.z, 0.0)

        node.last_cmd_angular = 0.0
        node.rotation_sweep_is_clear = lambda *args, **kwargs: False
        node.publish_failure_only_heading_correction(0.10)
        self.assertEqual(node.cmd_pub.messages[-1].angular.z, 0.0)

        node.rotation_sweep_is_clear = lambda *args, **kwargs: True
        node.publish_failure_only_heading_correction(0.10)
        self.assertAlmostEqual(node.cmd_pub.messages[-1].angular.z, 0.10, places=7)

    def test_sensor_to_body_conversion_is_yaw_invariant(self):
        body_x, body_y = 3.2, -4.1
        for yaw in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0):
            sensor_x = body_x + 0.4 * math.cos(yaw)
            sensor_y = body_y + 0.4 * math.sin(yaw)
            converted = base_xy_from_lidar_pose(
                sensor_x, sensor_y, yaw, 0.4, 0.0
            )
            self.assertAlmostEqual(converted[0], body_x, places=7)
            self.assertAlmostEqual(converted[1], body_y, places=7)


if __name__ == "__main__":
    unittest.main()
