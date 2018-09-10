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
)

catkin_package(LIBRARIES vision_commons)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake 
  ${vision_commons_INCLUDE_DIRS}
  include/
)

add_library(buoy_task SHARED src/buoy.cpp include/buoy.h)
target_link_libraries(buoy_task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(vision_node src/vision_node.cpp)
add_executable(gate_task_bottom src/gate_bottom.cpp)
add_executable(gate_task_front src/gate_front.cpp)
add_executable(line_task src/line.cpp)
add_executable(torpedo_task src/torpedo.cpp)

target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons buoy_task)
target_link_libraries(gate_task_bottom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )
target_link_libraries(gate_task_front ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )
target_link_libraries(line_task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )
target_link_libraries(torpedo_task ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )
