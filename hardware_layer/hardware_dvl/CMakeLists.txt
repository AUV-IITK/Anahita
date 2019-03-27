cmake_minimum_required(VERSION 2.8.3)
project(hardware_dvl)
set(hardware_dvl_SRC_DIR "src")
set(hardware_dvl_INCLUDE_DIR "include")

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

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES 
  CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
  DEPENDS system_lib
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

# Include all the directories where source can be found.
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_library(dvl_library SHARED src/driver/dvl_ethernet.cpp include/dvl_data.h include/dvl_ethernet.h)
target_link_libraries(dvl_library ${catkin_LIBRARIES})

add_executable(dvl_node src/hardware_dvl/dvl_node.cpp src/hardware_dvl/dvl_connection.cpp)
target_link_libraries(dvl_node dvl_library ${catkin_LIBRARIES})
# add_dependencies(dvl_node dvl_generate_messages_cpp)
