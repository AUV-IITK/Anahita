cmake_minimum_required(VERSION 2.8.3)
project(vision_commons)

add_compile_options(-std=c++11)

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
	INCLUDE_DIRS include helper_nodes
	LIBRARIES vision_commons)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(vision_commons src/filter.cpp src/contour.cpp src/geometry.cpp src/morph.cpp src/threshold.cpp)
target_link_libraries(vision_commons ${OpenCV_LIBRARIES} )

add_executable(blue_filter_demo helper_nodes/blue_filter_demo.cpp)
add_dependencies(blue_filter_demo ${vision_commons_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blue_filter_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(image_pub helper_nodes/image_pub.cpp)
target_link_libraries(image_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(image_save helper_nodes/image_save.cpp)
target_link_libraries(image_save ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(depth_image_pub helper_nodes/depth_image_pub.cpp)
target_link_libraries(depth_image_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

install(DIRECTORY include/vision_commons/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
