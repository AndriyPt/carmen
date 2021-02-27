cmake_minimum_required(VERSION 2.8.3)
project(carmen_2dnav)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS move_base gmapping amcl map_server rviz
)

install(FILES
  launch/gmapping.launch
  launch/rviz.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)