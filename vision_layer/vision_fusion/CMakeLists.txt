cmake_minimum_required(VERSION 2.8.3)
project(vision_fusion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion
CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs std_msgs
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
)

## Declare a C++ library
add_library(color_constancy
  src/color_constancy.cpp
)
target_link_libraries(color_constancy 
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_library(laplacian_blend
  src/laplacianBlend.cpp
)
target_link_libraries(laplacian_blend 
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(fusion_node src/fusion_framework.cpp)
target_link_libraries(fusion_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  laplacian_blend
  color_constancy
)

add_executable(color_debug src/color_constancy_debug.cpp)
target_link_libraries(color_debug
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  laplacian_blend
  color_constancy
)