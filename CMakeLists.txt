cmake_minimum_required(VERSION 2.8.3)
project(riegl_remote_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  riegl_vline_msgs
)

catkin_package(
  CATKIN_DEPENDS riegl_vline_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  riegl_remote_control_node
  ${riegl_vline_msgs_EXPORTED_TARGETS}
)
