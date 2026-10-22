cmake_minimum_required(VERSION 3.0.2)
project(yihan_ros_pkg)
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS ackermann_msgs cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/camera_yihan_web.py
  scripts/rear_bev_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY calibration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
