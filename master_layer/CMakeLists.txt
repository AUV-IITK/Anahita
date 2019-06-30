cmake_minimum_required(VERSION 2.8.3)
project(master_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
#   anahita_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  darknet_ros_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package (Eigen3 REQUIRED NO_MODULE)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Waypoint.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CurrentTask.srv
  ChangeOdom.srv
  GoToIncremental.srv
  GoTo.srv
  GoToPose.srv
  VerifyObject.srv
  InitCircularTrajectory.srv
  Hold.srv
  TrajectoryComplete.srv
  PoseReach.srv
  ChangeTorpedoHole.srv
  RequestMarkerAngle.srv
  PingerBottomTarget.srv
  PingerFrontTarget.srv
  TargetNormal.srv
  GetMaxDepth.srv
  GetBlackoutTime.srv
  ResetIMU.srv
  StallVehicle.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs anahita_msgs geometry_msgs darknet_ros_msgs 
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
#  INCLUDE_DIRS include
#  LIBRARIES action_servers
CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs geometry_msgs # task_handler_layer motion_layer anahita_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#   ${task_handler_layer_INCLUDE_DIRS}
#   ${motion_layer_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(local_vision_odom src/local_vision_odom.cpp)
target_link_libraries(local_vision_odom ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
