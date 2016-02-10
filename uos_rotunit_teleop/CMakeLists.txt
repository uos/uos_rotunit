cmake_minimum_required(VERSION 2.4.6)
project(uos_rotunit_teleop)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  uos_rotunit_snapshotter
  uos_rotunit_driver
  rviz
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}  
)

catkin_package(
CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  uos_rotunit_snapshotter
  uos_rotunit_driver
  rviz
DEPENDS
  Qt4
)

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  include/rviz_scan_tool.h
)

add_library(uos_rotunit_teleop_rviz_scan_tool src/rviz_scan_tool.cpp ${MOC_FILES})
target_link_libraries(uos_rotunit_teleop_rviz_scan_tool ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(uos_rotunit_teleop_ps3joy src/uos_rotunit_teleop_ps3joy.cpp)
target_link_libraries(uos_rotunit_teleop_ps3joy ${catkin_LIBRARIES})
add_dependencies(uos_rotunit_teleop_ps3joy ${catkin_EXPORTED_TARGETS})

install(TARGETS
  uos_rotunit_teleop_ps3joy
  uos_rotunit_teleop_rviz_scan_tool
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
