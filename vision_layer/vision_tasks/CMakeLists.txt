cmake_minimum_required(VERSION 2.8.3)
project(vision_tasks)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(vision_commons REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/buoyRange.cfg
  cfg/gateBottomRange.cfg
  cfg/gateFrontRange.cfg
  cfg/lineRange.cfg
  cfg/torpedoRange.cfg
  cfg/markerDropperRange.cfg
)

catkin_package(LIBRARIES vision_commons)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake 
  ${vision_commons_INCLUDE_DIRS}
  include/
)

add_library(buoy_task SHARED src/buoy.cpp include/buoy.h)
add_dependencies(buoy_task ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(buoy_task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(line_task SHARED src/line.cpp include/line.h)
add_dependencies(line_task ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(torpedo_task SHARED src/torpedo.cpp include/torpedo.h)
add_dependencies(torpedo_task ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(torpedo_task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(markerDropper_task SHARED src/markerDropper.cpp include/markerDropper.h)
add_dependencies(markerDropper_task ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(markerDropper_task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(vision_node src/vision_node.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons buoy_task line_task torpedo_task markerDropper_task)
