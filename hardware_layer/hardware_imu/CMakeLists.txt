cmake_minimum_required(VERSION 2.8.3)
project(hardware_imu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES IMU
  CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(px4 src/px4.cpp)
target_link_libraries(px4 ${catkin_LIBRARIES})

add_executable(sparton src/sparton.cpp)
target_link_libraries(sparton ${catkin_LIBRARIES})

add_executable(gazebo_imu src/gazebo_imu.cpp)
target_link_libraries(gazebo_imu ${catkin_LIBRARIES})
