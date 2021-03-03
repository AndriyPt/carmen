cmake_minimum_required(VERSION 2.8.3)
project(carmen_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    xacro
    joint_state_publisher_gui
)

catkin_package()

install(FILES
  launch/display.launch
  urdf/common.urdf.xacro
  urdf/materials.urdf.xacro
  urdf/wheels.urdf.xacro
  urdf/sensors/camera.urdf.xacro
  urdf/sensors/sonar.urdf.xacro
  robot/robot.urdf.xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
