cmake_minimum_required(VERSION 2.8.3)
project(uavcan_ros_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sam_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  message_runtime
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  UavcanNodeStatus.msg
  UavcanNodeStatusNamed.msg
  UavcanNodeStatusNamedArray.msg
  ESCStatus.msg
  CircuitStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UavcanGetNodeInfo.srv
  UavcanGetTransportStats.srv
  UavcanRestartNode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS
    #  LIBRARIES 
    #  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
    #  CFG_EXTRAS
)
