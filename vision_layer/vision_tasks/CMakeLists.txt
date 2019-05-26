cmake_minimum_required(VERSION 2.8.3)
project(vision_tasks)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  image_transport
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(vision_commons REQUIRED)

catkin_package(LIBRARIES vision_commons)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake 
  ${vision_commons_INCLUDE_DIRS}
  include/
)

add_library(base_class SHARED src/base_class.cpp include/base_class.h)
add_dependencies(base_class ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_class ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(torpedo_task SHARED src/torpedo.cpp include/torpedo.h include/base_class.h)
add_dependencies(torpedo_task base_class ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(torpedo_task base_class ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(gate_task SHARED src/gate.cpp include/gate.h include/base_class.h)
add_dependencies(gate_task base_class  ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gate_task base_class  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(path_marker SHARED src/path_marker.cpp include/path_marker.h include/base_class.h)
add_dependencies(path_marker base_class  ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_marker base_class  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(marker SHARED src/marker.cpp include/marker.h include/base_class.h)
add_dependencies(marker base_class  ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker base_class  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_library(start_gate SHARED src/start_gate.cpp include/start_gate.h include/base_class.h)
add_dependencies(start_gate base_class  ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(start_gate base_class  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(vision_node src/vision_node.cpp)
add_dependencies(vision_node ${vision_tasks_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  vision_commons 
  gate_task 
  path_marker
  marker
  torpedo_task
)
