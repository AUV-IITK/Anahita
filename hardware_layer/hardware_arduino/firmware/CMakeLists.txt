cmake_minimum_required(VERSION 2.8.3)

include_directories(
    ${ROS_LIB_DIR}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

#====================================================================#
# Advanced static library
#====================================================================#
generate_arduino_library(thruster_lib
    SRCS ../src/Thruster.cpp
    HDRS ../include/Thruster.h
    BOARD uno
)

generate_arduino_library(esc_lib
    SRCS ../src/ESC.cpp
    HDRS ../include/ESC.h
    BOARD uno
)

generate_arduino_library(pressure_sensor_lib
    SRCS ../src/MS5837.cpp
    HDRS ../include/MS5837.h
    BOARD uno
)

#====================================================================#
# Advanced firwmare
#====================================================================#
generate_arduino_firmware(arduino_node
  SRCS arduino_node.cpp ${ROS_LIB_DIR}/time.cpp
  HDRS ../include/ArduinoConfig.h
  LIBS thruster_lib pressure_sensor_lib esc_lib
  BOARD uno
  PORT /dev/arduino
)
