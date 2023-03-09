cmake_minimum_required(VERSION 3.5)
project(irc_ros_bringup)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)


install(DIRECTORY config launch params
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
