cmake_minimum_required(VERSION 2.8.3)
project(carmen_2dnav)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS move_base gmapping amcl map_server rviz rqt
)

install(FILES
  launch/gmapping.launch
  launch/amcl.launch
  launch/move_base.launch
  launch/rviz.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)