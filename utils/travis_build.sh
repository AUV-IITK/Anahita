#!/bin/bash

catkin_make --pkg vision_commons
catkin_make --pkg vision_fusion
catkin_make --pkg odom_dvl_imu
catkin_make --pkg hardware_imu
catkin_make --pkg hardware_dvl_ethernet
catkin_make --pkg teledyne_navigator
catkin_make --pkg hardware_camera
catkin_make --pkg hardware_pressure
catkin_make --pkg pid_calibration
catkin_make --pkg master_layer
catkin_make --pkg vision_tasks
