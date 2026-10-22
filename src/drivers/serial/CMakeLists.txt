cmake_minimum_required(VERSION 2.8.3)
project(serial)
find_package(catkin REQUIRED)
set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
catkin_package(LIBRARIES ${PROJECT_NAME} INCLUDE_DIRS include DEPENDS rt pthread)
add_library(${PROJECT_NAME} src/serial.cc src/impl/unix.cc src/impl/list_ports/list_ports_linux.cc)
include_directories(include)
target_link_libraries(${PROJECT_NAME} rt pthread)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(FILES include/serial/serial.h include/serial/v8stdint.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)
