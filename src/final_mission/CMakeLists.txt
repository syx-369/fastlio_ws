cmake_minimum_required(VERSION 3.0.2)
project(final_mission)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/final_tracker.py
  scripts/final_manager.py
  scripts/final_vision.py
  scripts/arm_bridge.py
  scripts/validate_route.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_final_tracker_safety.py)
endif()
