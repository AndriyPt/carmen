cmake_minimum_required(VERSION 2.8.3)
project(carmen_hardware)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager
    roscpp
    dynamic_reconfigure
    sensor_msgs
    carmen_msgs
)

catkin_package(
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    dynamic_reconfigure
    sensor_msgs
    carmen_msgs
    rosserial_mbed
)

install(FILES
  launch/hardware.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
