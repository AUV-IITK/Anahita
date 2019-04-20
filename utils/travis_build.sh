#!/bin/bash

catkin_make --pkg anahita_msgs
catkin_make --pkg vision_commons
catkin_make --pkg vision_fusion
catkin_make --pkg odom_dvl_imu
catkin_make --pkg vision_tasks
catkin_make --pkg motion_layer
catkin_make --pkg task_handler_layer
catkin_make --pkg hardware_imu
catkin_make --pkg hardware_dvl
catkin_make --pkg hardware_camera
catkin_make --pkg hardware_pressure
catkin_make --pkg pid_calibration
catkin_make --pkg viso2_ros
catkin_make --pkg master_layer