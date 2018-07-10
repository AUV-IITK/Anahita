# vision_tasks

## Overview

This is a ROS package for processing video feed provided by [`hardware_camera`](https://github.com/AUV-IITK/Hyperion-Software/tree/master/hardware_layer/hardware_camera) and publishing the coordinates of the object of interest in the camera's frame as a [geometry_msgs/PointStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PointStamped.html) or [geometry_msgs/Pose2D](http://docs.ros.org/api/geometry_msgs/html/msg/Pose2D.html) to the other layers.

The `vision_tasks` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](LICENSE.md).

The hardware used are as follows:
* Front-facing Camera: [Logitech C930e](https://www.logitech.com/en-in/product/c930e-webcam)
* Bottom-facing Camera: [Logitech C930e](https://www.logitech.com/en-in/product/c930e-webcam)

## Setting up vision_tasks

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following ROS Packages: [cv_bridge](http://wiki.ros.org/cv_bridge)
                          [sensor_msgs](http://wiki.ros.org/sensor_msgs)
                          [std_msgs](http://wiki.ros.org/std_msgs)
                          [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)
                          [image_transport](http://wiki.ros.org/image_transport)
### Building

Run the following command:
```
cd ~/catkin_ws
catkin_make --pkg vision_tasks
```

## Usage

To run any vision task named `<name>`, run:
```
rosrun vision_tasks <name>
```

## Nodes

### buoy_task

This task pre-processes (blue-filters) the raw image, applies thresholding on the image, draws a minimum enclosing circle on the main blob and returns the coordinates of the buoy in the camera's reference frame. The distance of the buoy from the camera is calculated using an exponential mapping of the radius of the minimum enclosing circle and the actual distance.

#### Published Topics
* **`/buoy_task/blue_filtered`** ([sensor_msgs/Image])
* **`/buoy_task/thresholded`** ([sensor_msgs/Image])
* **`/buoy_task/marked`** ([sensor_msgs/Image])
* **`/buoy_task/buoy_coordinates`** ([geometry_msgs/PointStamped])

#### Parameters
* `~clahe_clip` (double, default: 4.0, range: 0.0 - 40.0)
        Clip limit for CLAHE
* `~clahe_grid_size` (integer, default: 3, range: 1 - 16)
        CLAHE grid size
* `~clahe_bilateral_iter` (integer, default: 4, range: 0 - 16)
        Number of iterations of bilateral filter after CLAHE is applied
* `~balanced_bilateral_iter` (integer, default: 2, range: 0 - 8)
        Number of iterations of bilateral filter after white balancing is applied
* `~denoise_h` (double, default: 10.0, range: 0.0 - 20.0)
        h value for non-local means denoising applied to the final blue filtered picture
* `~low_b` (int, default: 0, range: 0.0 - 20.0)
        h value for non-local means denoising applied to the final blue filtered picture

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
