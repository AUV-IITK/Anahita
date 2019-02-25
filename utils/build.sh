#!/bin/bash

catkin_make --pkg anahita_msgs
catkin_make --pkg vision_commons
catkin_make --pkg vision_tasks
catkin_make --pkg motion_layer
catkin_make --pkg task_handler_layer
catkin_make --pkg hardware_imu
catkin_make --pkg hardware_dvl
catkin_make --pkg hardware_camera
catkin_make --pkg master_layer