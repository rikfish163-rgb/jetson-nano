cmake_minimum_required(VERSION 3.0.2)
project(robocup_competition)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs sensor_msgs geometry_msgs cv_bridge)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS master/main.py master/controller_node.py camera/perception_node.py signs/sign_node.py master/diagnose.py motion/bench_command.py parking/parking_inspect.py parking/timed_vision_node.py lidar/lidar_preview_node.py turn/left_lock_test.py turn/left_blue_test.py turn/left_two_stage_test.py turn/blue_stop_test.py lane/record_run.py parallel_parking/reference_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config master camera lidar signs motion lane obstacle turn uturn parking parallel_parking common
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.json" PATTERN "*.launch" PATTERN "README.md")
