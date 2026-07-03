cmake_minimum_required(VERSION 3.0.2)
project(delivery_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_controller.py
  scripts/red_flag_detector.py
  scripts/traffic_light_detector.py
  scripts/vision_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
