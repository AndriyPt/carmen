cmake_minimum_required(VERSION 2.8.3)
project(carmen_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS carmen_gazebo carmen_description robot_state_publisher
)

install(FILES
  launch/rviz.launch
  launch/simulation.launch
  launch/hardware.launch
  launch/teleop.launch
  launch/view_urdf.launch
  launch/gzweb.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
