cmake_minimum_required(VERSION 3.0.2)
project(main)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ackermann_msgs
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/legacy_controller.py
  scripts/parking_controller.py
  scripts/parking_replay.py
  scripts/parking_autotune.py
  scripts/parking_acceptance.py
  scripts/parking_stack_check.py
  scripts/parking_calibration_check.py
  scripts/reverse_parking.py
  scripts/return_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
catkin_add_nosetests(test)
