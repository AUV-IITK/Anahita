# hardware_camera

## Overview

This is a ROS package for getting feed from the camera using [usb_cam](http://wiki.ros.org/usb_cam) and publishing it as a [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) to the other layers.

The `hardware_camera` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](LICENSE.md).

The hardware used are as follows:
* Front-facing Camera: [Logitech C930e](https://www.logitech.com/en-in/product/c930e-webcam)
* Bottom-facing Camera: [Logitech C930e](https://www.logitech.com/en-in/product/c930e-webcam)

## Setting up hardware_camera

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following ROS Packages: [usb_cam](http://wiki.ros.org/usb_cam)

### Preparing the Serial Port via udev rules

The udev rules for the cameras connected to the robot are present in the [/utils/udev] directory. To copy them to your system, run:
```
cd ~/catkin_ws/src/auv2018/utils
sudo bash clone_udev.sh
```

Camera will then be connected to the ports with id `/dev/front_cam` and `/dev/bottom_cam`.

### Building Camera code

Run the following command:
```
cd ~/catkin_ws
catkin_make --pkg hardware_camera
```

## Usage

To initialize the cmaera nodes, run:
```
roslaunch hardware_camera camera_logitech.launch
```

## Nodes

### front_camera

The `usb_cam_node` interfaces with standard USB cameras (e.g. the Logitech Quickcam) using libusb_cam and publishes images as `sensor_msgs::Image`. The node belongs to the [usb_cam](http://wiki.ros.org/usb_cam) package.

#### Published Topics
* **`hardware_camera/front_cam/image_raw`** ([sensor_msgs/Image])

#### Parameters
* `~image_width` (integer, default: 1280)
        Image width
* `~image_height` (integer, default: 720)
        Image height
* `~framerate` (integer, default: 30)
        The required framerate
* `~camera_info_url` (string, default: )
        An url to the camera calibration file that will be read by the [CameraInfoManager](http://wiki.ros.org/CameraInfoManager) class

### bottom_camera

The `usb_cam_node` interfaces with standard USB cameras (e.g. the Logitech Quickcam) using libusb_cam and publishes images as `sensor_msgs::Image`. The node belongs to the [usb_cam](http://wiki.ros.org/usb_cam) package.

#### Published Topics
* **`bottom_camera/image_raw`** ([sensor_msgs/Image])

#### Parameters
* `~image_width` (integer, default: 1280)
        Image width
* `~image_height` (integer, default: 720)
        Image height
* `~framerate` (integer, default: 30)
        The required framerate
* `~camera_info_url` (string, default: )
        An url to the camera calibration file that will be read by the [CameraInfoManager](http://wiki.ros.org/CameraInfoManager) class.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AUV-IITK/auv2018/issues).

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
