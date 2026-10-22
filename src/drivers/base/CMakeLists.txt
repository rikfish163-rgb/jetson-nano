cmake_minimum_required(VERSION 3.0.2)
project(base_controller)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ackermann_msgs
  serial
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)
add_executable(base_controller src/base_controller.cpp
                               )
target_link_libraries(base_controller
  ${catkin_LIBRARIES}
)
