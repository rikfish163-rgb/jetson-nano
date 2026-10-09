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
  scripts/ros_flask_viewer.py
  scripts/camera_yihan_web.py
  scripts/rear_camera_lane_web.py
  scripts/blue_marker_metric.py
  scripts/rear_bev_calibrate.py
  scripts/rear_bev_node.py
  scripts/parking_observation.py
  scripts/parking_observation_node.py
  scripts/parking_front_metric.py
  scripts/parking_lane_pose.py
  scripts/parking_rear_metric.py
  scripts/parking_exit_metric.py
  scripts/parking_lidar_adapter.py
  scripts/parking_target_selector.py
  scripts/parking_session_logger.py
  scripts/parking_field_capture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config calibration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
