cmake_minimum_required(VERSION 2.8.3)

include_directories(
    ${ROS_LIB_DIR}
    ../include/
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(arduino_node
  SRCS arduino_node.cpp ${ROS_LIB_DIR}/time.cpp ../src/MS5837.cpp
  HDRS ../include/MS5837.h
  BOARD uno
  PORT /dev/arduino
)

