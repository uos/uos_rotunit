cmake_minimum_required(VERSION 2.4.6)

project(uos_rotunit_snapshotter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_assembler
  genmsg
  actionlib_msgs
  actionlib
  std_msgs
  message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_message_files(DIRECTORY msg FILES RotunitSnapshot.msg)
add_action_files(DIRECTORY action FILES RotunitSnapshot.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS
  roscpp
  sensor_msgs
  laser_assembler
  message_runtime
  actionlib_msgs
  std_msgs
)

add_executable(uos_rotunit_snapshotter src/uos_rotunit_snapshotter.cpp)
target_link_libraries(uos_rotunit_snapshotter ${catkin_LIBRARIES})
add_dependencies(uos_rotunit_snapshotter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS
  uos_rotunit_snapshotter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
