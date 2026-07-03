#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import signal
import rospy
from enum import Enum
from std_msgs.msg import Bool, String


class State(Enum):
    WAIT_FLAG = 0
    NAVIGATING = 1
    DONE = 2


class MissionController:
    def __init__(self):
        rospy.init_node('mission_controller')
        self.state = State.WAIT_FLAG
        self.flag_detected = False
        self.pursuit_process = None

        self.csv_path = rospy.get_param("~csv_path", "")
        self.target_speed = rospy.get_param("~target_speed", "0.5")

        # 红绿灯状态
        self.traffic_light_state = "none"
        self.waiting_traffic_light = False

        # 订阅红旗检测
        rospy.Subscriber('/flag_detected', Bool, self.flag_callback)

        # 订阅 pure_pursuit 的任务事件
        rospy.Subscriber('/waypoint_task_event', String, self.task_event_callback)

        # 订阅红绿灯检测结果
        rospy.Subscriber('/traffic_light_state', String, self.traffic_light_callback)

        # 发布 task_done
        self.task_done_pub = rospy.Publisher('/waypoint_task_done', String, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("MissionController initialized, state=WAIT_FLAG")

    def flag_callback(self, msg):
        if msg.data and self.state == State.WAIT_FLAG:
            self.flag_detected = True

    def traffic_light_callback(self, msg):
        self.traffic_light_state = msg.data.strip()

    def task_event_callback(self, msg):
        # 格式: "start:task_name:idxN" 或 "done:task_name:idxN"
        parts = msg.data.split(":")
        if len(parts) < 2:
            return
        phase = parts[0]
        task_name = parts[1]

        if phase == "start":
            rospy.loginfo(f"Task event: {msg.data}")

            if task_name == "traffic_light":
                self.handle_traffic_light()
            # 后续扩展:
            # elif task_name == "pick":
            #     self.handle_pick()
            # elif task_name == "place":
            #     self.handle_place()

    def handle_traffic_light(self):
        # 检查当前灯态，如果是绿灯直接放行
        if self.traffic_light_state == "green":
            rospy.loginfo("Traffic light is GREEN, passing through.")
            self.task_done_pub.publish(String(data="done"))
            return

        # 红灯或未知，等待变绿
        rospy.loginfo(f"Traffic light is {self.traffic_light_state}, waiting for GREEN...")
        self.waiting_traffic_light = True

    def start_pure_pursuit(self):
        cmd = [
            "rosrun", "waypoint_tools", "pure_pursuit_follower.py",
            f"_target_speed:={self.target_speed}",
        ]
        if self.csv_path:
            cmd.append(f"_csv_path:={self.csv_path}")

        rospy.loginfo(f"Starting pure_pursuit_follower: {' '.join(cmd)}")
        self.pursuit_process = subprocess.Popen(cmd, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))

    def stop_pure_pursuit(self):
        if self.pursuit_process and self.pursuit_process.poll() is None:
            self.pursuit_process.terminate()
            self.pursuit_process.wait(timeout=5)
            rospy.loginfo("pure_pursuit_follower stopped.")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == State.WAIT_FLAG:
                if self.flag_detected:
                    rospy.loginfo("Flag detected! Starting navigation.")
                    self.start_pure_pursuit()
                    self.state = State.NAVIGATING

            elif self.state == State.NAVIGATING:
                # 等红绿灯变绿
                if self.waiting_traffic_light:
                    if self.traffic_light_state == "green":
                        rospy.loginfo("Traffic light turned GREEN! Resuming.")
                        self.waiting_traffic_light = False
                        self.task_done_pub.publish(String(data="done"))

                # 检查 pure_pursuit 是否已结束
                if self.pursuit_process and self.pursuit_process.poll() is not None:
                    rospy.loginfo("Navigation finished.")
                    self.state = State.DONE

            elif self.state == State.DONE:
                rospy.loginfo("Mission complete!")
                break

            rate.sleep()

    def on_shutdown(self):
        self.stop_pure_pursuit()


if __name__ == '__main__':
    try:
        mc = MissionController()
        mc.run()
    except rospy.ROSInterruptException:
        pass
