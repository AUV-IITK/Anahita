cmake_minimum_required(VERSION 2.8.3)
project(hardware_arduino)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rospy
  rosserial_arduino
  rosserial_client
  anahita_msgs
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
  # INCLUDE_DIRS include
  # LIBRARIES hardware_arduino
  CATKIN_DEPENDS roscpp rospy rosserial_arduino rosserial_client std_msgs anahita_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${ROS_LIB_DIR}
  ${catkin_INCLUDE_DIRS}
)

## For building arduino firmware
rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

add_dependencies(${PROJECT_NAME}_ros_lib
  anahita_msgs_generate_messages
  anahita_msgs_generate_messages_cpp
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware arduino_node ALL)
rosserial_add_client_target(firmware arduino_node-upload)
