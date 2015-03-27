cmake_minimum_required(VERSION 2.4.6)

project(uos_rotunit_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  message_runtime
  genmsg
)

add_service_files(
  FILES
  RotVelSrv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  message_runtime
  genmsg
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(uos_rotunit_driver src/rotunit.cpp src/can.cpp)
target_link_libraries(uos_rotunit_driver ${catkin_LIBRARIES})
add_dependencies(uos_rotunit_driver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS
  uos_rotunit_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
