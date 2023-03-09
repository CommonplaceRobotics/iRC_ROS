cmake_minimum_required(VERSION 3.5)
project(irc_ros_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(irc_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(INCLUDE_DEPENDS
  ament_cmake
  control_msgs
  controller_manager
  controller_manager_msgs
  hardware_interface
  irc_ros_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

include_directories(include)

add_library( ${PROJECT_NAME} SHARED
  src/irc_ros_cri.cpp
  src/CRI/cri_socket.cpp
  src/CRI/cri_messages.cpp
  src/irc_ros_can.cpp
  src/CAN/module.cpp
  src/CAN/joint.cpp
  src/CAN/digital_io.cpp
  src/CAN/can_message.cpp
  src/CAN/can_interface.cpp
  src/CAN/can_socket.cpp
)

target_include_directories( ${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies( ${PROJECT_NAME}
  ${INCLUDE_DEPENDS}
)

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface irc_ros_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  #DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_dependencies(
  ${INCLUDE_DEPENDS}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

# this must happen before the invocation of ament_package()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
