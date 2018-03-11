# hardware_camera

## Overview

This is a ROS package for interacting arduino through the [usb_cam](http://wiki.ros.org/usb_cam) package. The package is meant to activate cameras connected to the system, and publish the measurements taken from the front and bottom camera onto separate topics.

The `hardware_camera` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](LICENSE.md).

The hardware used are as follows:
* [Logitech C930e](https://www.logitech.com/en-in/product/c930e-webcam)

## Setting up Arduino

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following ROS Packages: [usb_cam](http://wiki.ros.org/usb_cam)

### Preparing the Serial Port
Camera will likely connect to computer as port `/dev/front_cam` and `/dev/bottom_cam`.
```
ls /dev*
```
Next we make sure that we have permissions to activate
```
sudo chmod o+x /dev/front_cam
sudo chmod o+x /dev/bottom_cam
```
### Building Arduino code

Run the following command:
```
cd ~/catkin_ws
catkin_make --pkg hardware_camera
```

## Usage

To connect to the arduino, run:
```
roslaunch hardware_camera camera_config.launch
```

## Nodes

#### Subscribed Topics
All topic subscribed by usb_cam package

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AUV-IITK/auv2017/issues).

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html

