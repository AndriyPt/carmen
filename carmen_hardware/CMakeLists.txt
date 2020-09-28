cmake_minimum_required(VERSION 2.8.3)
project(carmen_hardware)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager
    roscpp
    dynamic_reconfigure
    orion_protocol
)

catkin_package(
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    dynamic_reconfigure
    orion_protocol
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/robot_hardware.cpp
  src/hardware_node.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  orion_protocol
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  launch/hardware.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


if (CATKIN_ENABLE_TESTING)

  include_directories(
    include
    firmware/App/public
    firmware/test/include
    ${catkin_INCLUDE_DIRS}
  )

#  catkin_add_gmock(${PROJECT_NAME}_test_fw_communication
#    firmware/test/test_communication.cpp
#    firmware/App/communication/communication.cpp
#  )
#  add_dependencies(${PROJECT_NAME}_test_fw_communication ${catkin_EXPORTED_TARGETS})
#  target_link_libraries(${PROJECT_NAME}_test_fw_communication)

  catkin_add_gmock(${PROJECT_NAME}_test_fw_control_loop
    firmware/test/test_control_loop.cpp
    firmware/App/control_loop/control_loop.c
  )
  add_dependencies(${PROJECT_NAME}_test_fw_control_loop ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_test_fw_control_loop)

endif ()
