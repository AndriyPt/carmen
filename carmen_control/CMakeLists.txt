cmake_minimum_required(VERSION 2.8.3)
project(carmen_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs  
    joint_state_controller
    robot_localization
    controller_manager
    teleop_twist_keyboard
    diff_drive_controller
    imu_sensor_controller
    velocity_controllers
    effort_controllers
    imu_tools
    INCLUDE_DIRS include
    LIBRARIES rage_sensor_controller
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(rage_sensor_controller src/range_sensor_controller.cpp)
target_link_libraries(rage_sensor_controller ${catkin_LIBRARIES})

install(DIRECTORY include/carmen_control/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS rage_sensor_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  config/common/joystick.yml
  config/common/keyboard.yml
  config/common/state_controller.yml
  config/common/twist_mux.yml
  config/common/velocity_controllers.yml
  config/gazebo/diff_drive_controller.yml
  config/gazebo/gazebo_controller.yml
  launch/control.launch
  launch/teleop.launch
  range_sensor_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
