cmake_minimum_required(VERSION 2.8.3)
project(odom_dvl_imu)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(SRC_DIR "src")
set(INCLUDE_DIR "include")


# C A T K I N   C O N F I G U R A T I O N

find_package(PkgConfig)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    geometry_msgs
    sensor_msgs
    tf
)

add_service_files(
        DIRECTORY srv
        FILES
        SetDepthOffset.srv
        SetWorldXYOffset.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs  # Or other packages containing msgs
)

catkin_package(
    INCLUDE_DIRS ${SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS roscpp message_runtime std_msgs
)

file(GLOB_RECURSE FILES
        "${SRC_DIR}/*.cpp"
        "${SRC_DIR}/*.h"
        "${INCLUDE_DIR}/*.h"
)

list(REMOVE_ITEM FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/nav_node.cpp)

# Include all the directories where source can be found.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${SRC_DIR}
        ${INCLUDE_DIR}
)


add_executable(${PROJECT_NAME}_node ${SRC_DIR}/nav_node.cpp ${FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)

