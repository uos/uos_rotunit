cmake_minimum_required(VERSION 2.4.6)

project(uos_rotunit_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  uos_rotunit_snapshotter
  uos_rotunit_driver
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  uos_rotunit_snapshotter
  uos_rotunit_driver
)

add_executable(uos_rotunit_teleop_ps3joy src/uos_rotunit_teleop_ps3joy.cpp)
target_link_libraries(uos_rotunit_teleop_ps3joy ${catkin_LIBRARIES})
add_dependencies(uos_rotunit_teleop_ps3joy ${catkin_EXPORTED_TARGETS})

install(TARGETS
  uos_rotunit_teleop_ps3joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
