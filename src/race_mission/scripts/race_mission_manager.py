#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import signal
import subprocess

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class RaceMissionManager:
    """Top-level race state machine.

    The waypoint follower owns low-level base motion. This node only decides
    when to start navigation and how to answer external task points:

      ext:pick          -> send /arm_task_cmd "pick", wait /arm_task_result
      ext:traffic_light -> wait until the vision node reports green
      ext:place         -> send /arm_task_cmd "place", wait /arm_task_result
      ext:finish        -> mark the mission done after the final stop
    """

    def __init__(self):
        rospy.init_node("race_mission_manager", anonymous=False)

        # ---------------- Parameters ----------------
        self.csv_path = rospy.get_param("~csv_path", "")
        self.target_speed = str(rospy.get_param("~target_speed", "0.45"))
        self.cmd_topic = rospy.get_param("~cmd_topic", "/smoother_cmd_vel")
        self.vehicle_model = rospy.get_param("~vehicle_model", "diff")
        self.wheelbase = str(rospy.get_param("~wheelbase", "0.65"))
        self.max_steer_angle = str(rospy.get_param("~max_steer_angle", "0.461"))
        self.enable_final_yaw_align = rospy.get_param("~enable_final_yaw_align", True)
        self.wait_for_start = bool(rospy.get_param("~wait_for_start", True))
        self.auto_start = bool(rospy.get_param("~auto_start", False))

        self.task_event_topic = rospy.get_param("~task_event_topic", "/waypoint_task_event")
        self.task_done_topic = rospy.get_param("~task_done_topic", "/waypoint_task_done")
        self.external_task_timeout = float(rospy.get_param("~external_task_timeout", 180.0))

        self.arm_task_cmd_topic = rospy.get_param("~arm_task_cmd_topic", "/arm_task_cmd")
        self.arm_task_result_topic = rospy.get_param("~arm_task_result_topic", "/arm_task_result")
        self.traffic_light_topic = rospy.get_param("~traffic_light_topic", "/race/traffic_light")
        self.start_signal_topic = rospy.get_param("~start_signal_topic", "/race/start_signal")
        self.manual_command_topic = rospy.get_param("~manual_command_topic", "/race/mission_command")
        self.mission_state_topic = rospy.get_param("~mission_state_topic", "/race/mission_state")
        self.vision_control_topic = rospy.get_param("~vision_control_topic", "/race/vision_control")

        self.arm_tasks = self._param_list("~arm_tasks", ["pick", "place"])
        self.traffic_light_tasks = self._param_list("~traffic_light_tasks", ["traffic_light"])
        self.finish_task_name = rospy.get_param("~finish_task_name", "finish")
        self.green_states = self._param_list("~green_states", ["green"])
        self.traffic_timeout = float(rospy.get_param("~traffic_timeout", 120.0))
        self.traffic_timeout_policy = rospy.get_param("~traffic_timeout_policy", "hold").strip().lower()
        self.finish_done_delay = float(rospy.get_param("~finish_done_delay", 8.0))

        # ---------------- State ----------------
        self.state = "BOOT"
        self.start_received = self.auto_start
        self.traffic_light_state = "none"
        self.pending_task = None
        self.pending_task_started_at = None
        self.pursuit_process = None
        self.finish_seen_at = None

        # ---------------- ROS IO ----------------
        self.state_pub = rospy.Publisher(self.mission_state_topic, String, queue_size=1, latch=True)
        self.task_done_pub = rospy.Publisher(self.task_done_topic, String, queue_size=10)
        self.arm_task_pub = rospy.Publisher(self.arm_task_cmd_topic, String, queue_size=10)
        self.vision_control_pub = rospy.Publisher(self.vision_control_topic, String, queue_size=5)
        self.stop_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)

        rospy.Subscriber(self.start_signal_topic, Bool, self.start_signal_callback, queue_size=5)
        rospy.Subscriber("/flag_detected", Bool, self.start_signal_callback, queue_size=5)
        rospy.Subscriber(self.manual_command_topic, String, self.manual_command_callback, queue_size=5)
        rospy.Subscriber(self.task_event_topic, String, self.task_event_callback, queue_size=20)
        rospy.Subscriber(self.arm_task_result_topic, String, self.arm_result_callback, queue_size=10)
        rospy.Subscriber(self.traffic_light_topic, String, self.traffic_light_callback, queue_size=10)
        rospy.Subscriber("/traffic_light_state", String, self.traffic_light_callback, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)
        self.set_state("WAIT_START" if self.wait_for_start and not self.auto_start else "READY")
        if self.wait_for_start and not self.auto_start:
            self.vision_control_pub.publish(String(data="flag"))
        else:
            self.vision_control_pub.publish(String(data="idle"))

    def _param_list(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return list(value)

    def set_state(self, state):
        if self.state != state:
            rospy.loginfo("Mission state: %s -> %s", self.state, state)
        self.state = state
        self.state_pub.publish(String(data=state))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def start_signal_callback(self, msg):
        if msg.data:
            self.start_received = True
            self.vision_control_pub.publish(String(data="idle"))
            rospy.loginfo("Start signal received.")

    def manual_command_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "start":
            self.start_received = True
            rospy.loginfo("Manual start command received.")
        elif command == "stop":
            rospy.logwarn("Manual stop command received.")
            self.stop_robot()
            self.stop_pursuit()
            self.set_state("STOPPED")
        elif command == "finish":
            rospy.logwarn("Manual finish command received.")
            self.finish_seen_at = rospy.Time.now()
            self.set_state("DONE")
        elif command:
            rospy.logwarn("Unknown mission command: %s", command)

    def traffic_light_callback(self, msg):
        self.traffic_light_state = msg.data.strip().lower()

    def task_event_callback(self, msg):
        parts = msg.data.strip().split(":")
        if len(parts) < 2:
            return

        phase = parts[0].strip()
        task_name = parts[1].strip()
        if phase != "start":
            return

        rospy.loginfo("Waypoint task started: %s", msg.data)

        if task_name in self.arm_tasks:
            self.start_arm_task(task_name)
        elif task_name in self.traffic_light_tasks:
            self.start_traffic_light_task(task_name)
        elif task_name == self.finish_task_name:
            self.complete_waypoint_task(task_name)
            self.finish_seen_at = rospy.Time.now()
            self.set_state("FINISHING")
        else:
            rospy.logwarn("No handler for task '%s'. Marking it done.", task_name)
            self.complete_waypoint_task(task_name)

    def arm_result_callback(self, msg):
        result = msg.data.strip()
        if not result:
            return

        if self.state != "ARM_TASK" or not self.pending_task:
            rospy.loginfo("Ignoring arm result outside ARM_TASK: %s", result)
            return

        expected_done = "done:%s" % self.pending_task
        expected_fail = "fail:%s" % self.pending_task

        if result in ("done", "all_done", self.pending_task, expected_done):
            rospy.loginfo("Arm task '%s' completed.", self.pending_task)
            self.complete_waypoint_task(self.pending_task)
            self.pending_task = None
            self.pending_task_started_at = None
            self.set_state("NAVIGATING")
        elif result in ("fail", expected_fail):
            rospy.logerr("Arm task '%s' failed: %s", self.pending_task, result)
            self.stop_robot()
            self.set_state("ERROR")
        else:
            rospy.loginfo("Arm result does not match pending task '%s': %s", self.pending_task, result)

    # ------------------------------------------------------------------
    # Task handlers
    # ------------------------------------------------------------------
    def start_arm_task(self, task_name):
        self.stop_robot()
        self.pending_task = task_name
        self.pending_task_started_at = rospy.Time.now()
        self.set_state("ARM_TASK")

        if self.arm_task_pub.get_num_connections() == 0:
            rospy.logwarn("No subscribers on %s yet. Publishing task anyway.", self.arm_task_cmd_topic)

        rospy.loginfo("Sending arm task: %s", task_name)
        self.vision_control_pub.publish(String(data="idle"))
        self.arm_task_pub.publish(String(data=task_name))

    def start_traffic_light_task(self, task_name):
        self.stop_robot()
        self.pending_task = task_name
        self.pending_task_started_at = rospy.Time.now()

        if self.traffic_light_state in self.green_states:
            rospy.loginfo("Traffic light already green; continue.")
            self.vision_control_pub.publish(String(data="idle"))
            self.complete_waypoint_task(task_name)
            self.pending_task = None
            self.pending_task_started_at = None
            self.set_state("NAVIGATING")
            return

        rospy.loginfo("Waiting for green traffic light. Current state: %s", self.traffic_light_state)
        self.vision_control_pub.publish(String(data="light"))
        self.set_state("WAIT_TRAFFIC_LIGHT")

    def complete_waypoint_task(self, task_name):
        done_msg = "done:%s" % task_name
        rospy.loginfo("Publishing waypoint task done: %s", done_msg)
        self.task_done_pub.publish(String(data=done_msg))

    # ------------------------------------------------------------------
    # Navigation process
    # ------------------------------------------------------------------
    def start_pursuit(self):
        if self.pursuit_process and self.pursuit_process.poll() is None:
            rospy.logwarn("pure_pursuit_follower is already running.")
            return

        cmd = [
            "rosrun", "waypoint_tools", "pure_pursuit_follower.py",
            "_target_speed:=%s" % self.target_speed,
            "_cmd_topic:=%s" % self.cmd_topic,
            "_vehicle_model:=%s" % self.vehicle_model,
            "_wheelbase:=%s" % self.wheelbase,
            "_max_steer_angle:=%s" % self.max_steer_angle,
            "_enable_final_yaw_align:=%s" % str(self.enable_final_yaw_align).lower(),
            "_task_event_topic:=%s" % self.task_event_topic,
            "_task_done_topic:=%s" % self.task_done_topic,
            "_external_task_timeout:=%.1f" % self.external_task_timeout,
        ]
        if self.csv_path.strip():
            cmd.append("_csv_path:=%s" % self.csv_path)

        rospy.loginfo("Starting waypoint follower: %s", " ".join(cmd))
        self.pursuit_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.set_state("NAVIGATING")

    def stop_pursuit(self):
        if self.pursuit_process and self.pursuit_process.poll() is None:
            rospy.loginfo("Stopping waypoint follower.")
            try:
                os.killpg(os.getpgid(self.pursuit_process.pid), signal.SIGTERM)
                self.pursuit_process.wait(timeout=5.0)
            except Exception as exc:
                rospy.logwarn("Failed to stop follower gracefully: %s", exc)
                try:
                    os.killpg(os.getpgid(self.pursuit_process.pid), signal.SIGKILL)
                except Exception:
                    pass

    def stop_robot(self):
        zero = Twist()
        for _ in range(3):
            self.stop_pub.publish(zero)
            rospy.sleep(0.03)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state in ("WAIT_START", "READY"):
                if (not self.wait_for_start) or self.start_received:
                    self.start_pursuit()

            elif self.state == "WAIT_TRAFFIC_LIGHT":
                if self.traffic_light_state in self.green_states:
                    rospy.loginfo("Traffic light is green; continue.")
                    self.vision_control_pub.publish(String(data="idle"))
                    self.complete_waypoint_task(self.pending_task)
                    self.pending_task = None
                    self.pending_task_started_at = None
                    self.set_state("NAVIGATING")
                elif self.traffic_timeout > 0.0 and self.pending_task_started_at is not None:
                    waited = (rospy.Time.now() - self.pending_task_started_at).to_sec()
                    if waited >= self.traffic_timeout:
                        if self.traffic_timeout_policy == "pass":
                            rospy.logwarn("Traffic light timeout; passing by policy.")
                            self.vision_control_pub.publish(String(data="idle"))
                            self.complete_waypoint_task(self.pending_task)
                            self.pending_task = None
                            self.pending_task_started_at = None
                            self.set_state("NAVIGATING")
                        else:
                            rospy.logerr("Traffic light timeout; holding in ERROR.")
                            self.vision_control_pub.publish(String(data="idle"))
                            self.stop_robot()
                            self.set_state("ERROR")

            elif self.state == "FINISHING":
                self.stop_robot()
                if self.finish_seen_at is not None:
                    elapsed = (rospy.Time.now() - self.finish_seen_at).to_sec()
                    if elapsed >= self.finish_done_delay:
                        self.stop_pursuit()
                        self.set_state("DONE")

            elif self.state == "DONE":
                self.stop_robot()

            elif self.state == "ERROR":
                self.stop_robot()

            if self.pursuit_process and self.pursuit_process.poll() is not None:
                if self.state not in ("DONE", "ERROR", "STOPPED"):
                    rospy.logwarn("Waypoint follower exited with code %s.", self.pursuit_process.returncode)
                    self.set_state("DONE")

            rate.sleep()

    def on_shutdown(self):
        self.stop_robot()
        self.stop_pursuit()
        self.stop_robot()


if __name__ == "__main__":
    try:
        RaceMissionManager().spin()
    except rospy.ROSInterruptException:
        pass
