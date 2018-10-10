cmake_minimum_required(VERSION 2.8.3)
project(vision_commons)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  dynamic_reconfigure
)

roslint_cpp()

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/blueFilterDemoRange.cfg)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES vision_commons)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(vision_commons src/filter.cpp src/contour.cpp src/geometry.cpp src/morph.cpp src/threshold.cpp)
target_link_libraries(vision_commons ${OpenCV_LIBRARIES} )

add_executable(blue_filter_demo src/blue_filter_demo.cpp)
add_dependencies(blue_filter_demo ${vision_commons_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blue_filter_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

install(DIRECTORY include/vision_commons/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
