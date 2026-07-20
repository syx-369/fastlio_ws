cmake_minimum_required(VERSION 3.0.2)
project(hybrid_avoid)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/hybrid_avoidance_node.py
  scripts/hybrid_obstacle_simulator.py
  scripts/validate_hybrid_route.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch paths
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
