cmake_minimum_required(VERSION 3.0.2)
project(vehicle_control)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nav_msgs rospy std_msgs
)
