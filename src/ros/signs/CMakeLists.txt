cmake_minimum_required(VERSION 3.0.2)
project(hts_ros_pkg)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package(
)
catkin_install_python(PROGRAMS
  scripts/sign_string_node.py
  scripts/sign_classifier_cv.py
  scripts/sign_actions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
