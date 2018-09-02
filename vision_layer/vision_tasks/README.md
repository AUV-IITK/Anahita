# vision_tasks

## Overview

This is a ROS package for processing video feed provided by [`hardware_camera`](https://github.com/AUV-IITK/Hyperion-Software/tree/master/hardware_layer/hardware_camera) and publishing the coordinates of the object of interest in the camera's frame as a [geometry_msgs/PointStamped] or [geometry_msgs/Pose2D] to the other layers.

The `vision_tasks` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](../../LICENSE).

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
* **`/buoy_task/buoy_coordinates`** ([geometry_msgs/PointStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PointStamped.html))

#### Parameters
* `~clahe_clip` (double, default: 0.15, range: 0.0 - 40.0)
        Clip limit for CLAHE
* `~clahe_grid_size` (integer, default: 3, range: 1 - 16)
        Grid size of the CLAHE operator
* `~clahe_bilateral_iter` (integer, default: 2, range: 0 - 16)
        Number of iterations of bilateral filter after CLAHE is applied
* `~balanced_bilateral_iter` (integer, default: 2, range: 0 - 8)
        Number of iterations of bilateral filter after white balancing is applied
* `~denoise_h` (double, default: 10.0, range: 0.0 - 20.0)
        h value for fast non-local means denoising applied on the final blue-filtered image
* `~low_h` (int, default: 0, range: 0 - 255)
        Lower Bound of H
* `~high_h` (int, default: 10, range: 0 - 255)
        Higher Bound of H
* `~low_s` (int, default: 251, range: 0 - 255)
        Lower Bound of S
* `~high_s` (int, default: 255, range: 0 - 255)
        Higher Bound of S
* `~low_v` (int, default: 160, range: 0 - 255)
        Lower Bound of V
* `~high_v` (int, default: 255, range: 0 - 255)
        Higher Bound of V
* `~opening_mat_point` (int, default: 1, range: 1 - 7)
        Center of the matrix for the opening operation (size extrapolated 2x+1)
* `~opening_iter` (int, default: 0, range: 0 - 10)
        Iterations of opening applied on the thresholded image
* `~closing_mat_point` (int, default: 1, range: 1 - 7)
        Center of the matrix for the closing operation (size extrapolated 2x+1)
* `~closing_iter` (int, default: 0, range: 0 - 10)
        Iterations of closing applied on the opened thresholded image

### gate_task_bottom

This task applies pre-processes (blue-filters) the raw image, applies thresholding on the image, draws a minimum enclosing rectangle on the main blob and publishes the coordinates of the gate's horizontal arm in the camera's reference frame and also publishes whether the task is done or not. 

#### Published Topics
* **`/gate_task/bottom/thresholded`** ([sensor_msgs/Image])
* **`/gate_task/bottom/marked`** ([sensor_msgs/Image])
* **`/gate_task/bottom/pipe_coordinates`** ([geometry_msgs/PointStamped])
* **`/gate_task/done`** ([std_msgs/Bool](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Bool.html))

#### Parameters
* `~clahe_clip` (double, default: 4.0, range: 0.0 - 40.0)
        Clip limit for CLAHE
* `~clahe_grid_size` (integer, default: 8, range: 1 - 16)
        Grid size of the CLAHE operator
* `~clahe_bilateral_iter` (integer, default: 4, range: 0 - 16)
        Number of iterations of bilateral filter after CLAHE is applied
* `~balanced_bilateral_iter` (integer, default: 2, range: 0 - 8)
        Number of iterations of bilateral filter after white balancing is applied
* `~denoise_h` (double, default: 10.0, range: 0.0 - 20.0)
        h value for fast non-local means denoising applied on the final blue-filtered image
* `~low_h` (int, default: 0, range: 0 - 255)
        Lower Bound of H
* `~high_h` (int, default: 130, range: 0 - 255)
        Higher Bound of H
* `~low_s` (int, default: 0, range: 0 - 255)
        Lower Bound of S
* `~high_s` (int, default: 123, range: 0 - 255)
        Higher Bound of S
* `~low_v` (int, default: 95, range: 0 - 255)
        Lower Bound of V
* `~high_v` (int, default: 255, range: 0 - 255)
        Higher Bound of V
* `~closing_mat_point` (int, default: 2, range: 1 - 7)
        Center of the matrix for the closing operation (size extrapolated 2x+1)
* `~closing_iter` (int, default: 2, range: 0 - 10)
        Iterations of closing applied on the thresholded image

### gate_task_front

This task applies pre-processes (blue-filters) the raw image, applies thresholding on the image, detects edges using Canny edge detection,detects straight lines using Probabilistic Hough Line Transform, filters all the vertical and horizontal lines and detects which pair of these lines are perpendicular and close enough to be a gate. If multiple such pairs are found, the pair with longer lines is taken into considerations. This returns the coordinates of the gate in the camera's frame. The distance of the gate from the camera is calculated using an exponential mapping of the diagonal of the rectangle formed by the two arms and the actual distance. If both arms of the gate are not detected and only one of the arms is detected, the rectangle is extrapolated and the distance of the gate from the camera is also calculated based on this extrapolated rectangle.

#### Published Topics
* **`/gate_task/front/blue_filtered`** ([sensor_msgs/Image])
* **`/gate_task/front/thresholded`** ([sensor_msgs/Image])
* **`/gate_task/front/canny`** ([sensor_msgs/Image])
* **`/gate_task/front/lines`** ([sensor_msgs/Image])
* **`/gate_task/front/marked`** ([sensor_msgs/Image])
* **`/gate_task/front/gate_coordinates`** ([geometry_msgs/PointStamped])

#### Parameters
* `~clahe_clip` (double, default: 0.15, range: 0.0 - 40.0)
        Clip limit for CLAHE
* `~clahe_grid_size` (integer, default: 3, range: 1 - 16)
        Grid size of the CLAHE operator
* `~clahe_bilateral_iter` (integer, default: 2, range: 0 - 16)
        Number of iterations of bilateral filter after CLAHE is applied
* `~balanced_bilateral_iter` (integer, default: 4, range: 0 - 8)
        Number of iterations of bilateral filter after white balancing is applied
* `~denoise_h` (double, default: 10.0, range: 0.0 - 20.0)
        h value for fast non-local means denoising applied on the final blue-filtered image
* `~low_h` (int, default: 0, range: 0 - 255)
        Lower Bound of H
* `~high_h` (int, default: 10, range: 0 - 255)
        Higher Bound of H
* `~low_s` (int, default: 156, range: 0 - 255)
        Lower Bound of S
* `~high_s` (int, default: 255, range: 0 - 255)
        Higher Bound of S
* `~low_v` (int, default: 88, range: 0 - 255)
        Lower Bound of V
* `~high_v` (int, default: 255, range: 0 - 255)
        Higher Bound of V
* `~closing_mat_point` (int, default: 1, range: 1 - 7)
        Center of the matrix for the closing operation (size extrapolated 2x+1)
* `~closing_iter` (int, default: 0, range: 0 - 10)
        Iterations of closing applied on the thresholded image
* `~canny_threshold_low` (int, default: 0, range: 0 - 1000)
        Lower threshold for the pixel intensity gradient
* `~canny_threshold_high` (int, default: 1000, range: 0 - 1000)
        Higher threshold for the pixel intensity gradient        
* `~canny_kernel_size` (int, default: 3, range: 3 - 7)
        Kernel size of the operator
* `~hough_minline` (int, default: 200, range: 0 - 500)
        Minimum length of the detected lines
* `~hough_threshold` (int, default: 105, range: 0 - 500)
        Minimum number of intersections to be considered as part of one line
* `~hough_maxgap` (int, default: 61, range: 0 - 500)
        Maximum gap between two points to be considered the same line
* `~hough_angle_tolerance` (double, default: 20.0, range: 0.0 - 45.0)
        Tolerance angle with respect to 0, 90 and 180 degrees to be considered as a probable line
* `~gate_distance_tolerance` (double, default: 50.0, range: 0.0 - 500.0)
        Maximum distance between atleast one pair of points to be considered close to each other
* `~gate_angle_tolerance` (double, default: 20.0, range: 0.0 - 45.0)
        Tolerance angle with respect to 90 to be considered as perpendicular

### line_task

This task applies thresholding on the raw image, draws contours around the blob, finds straight lines using Probabilistic Hough Line Transform and draws a tilted bounding rectangle around the contour. This returns the coordinates in the camera's reference frame using the bounding rectangle and returns the vehicle's angle with this line using the lines detected.

#### Published Topics
* **`/line_task/thresholded`** ([sensor_msgs/Image])
* **`/line_task/marked`** ([sensor_msgs/Image])
* **`/line_task/gate_coordinates`** ([geometry_msgs/Pose2D])

#### Parameters
* `~low_h` (int, default: 31, range: 0 - 255)
        Lower Bound of H
* `~high_h` (int, default: 47, range: 0 - 255)
        Higher Bound of H
* `~low_s` (int, default: 0, range: 0 - 255)
        Lower Bound of S
* `~high_s` (int, default: 255, range: 0 - 255)
        Higher Bound of S
* `~low_v` (int, default: 0, range: 0 - 255)
        Lower Bound of V
* `~high_v` (int, default: 255, range: 0 - 255)
        Higher Bound of V
* `~opening_mat_point` (int, default: 1, range: 1 - 7)
        Center of the matrix for the opening operation (size extrapolated 2x+1)
* `~opening_iter` (int, default: 0, range: 0 - 10)
        Iterations of opening applied on the thresholded image
* `~closing_mat_point` (int, default: 2, range: 1 - 7)
        Center of the matrix for the closing operation (size extrapolated 2x+1)
* `~closing_iter` (int, default: 1, range: 0 - 10)
        Iterations of closing applied on the opened thresholded image

### torpedo_task

This task applies pre-processes (blue-filters) the raw image, applies thresholding on the image, draws a minimum enclosing rectangle on all the detected contours. Based on the number of contours and their areas, an algorithm decides which of the contours is actually the heart and publishes the coordinates of the target in the camera's reference frame. The distance of the target from the camera is calculated using an exponential mapping of the width of the bounding rectangle and the actual distance.

#### Published Topics
* **`/torpedo_task/thresholded`** ([sensor_msgs/Image])
* **`/torpedo_task/marked`** ([sensor_msgs/Image])
* **`/torpedo_task/gate_coordinates`** ([geometry_msgs/Pose2D])

#### Parameters
* `~clahe_clip` (double, default: 0.15, range: 0.0 - 40.0)
        Clip limit for CLAHE
* `~clahe_grid_size` (integer, default: 3, range: 1 - 16)
        Grid size of the CLAHE operator
* `~clahe_bilateral_iter` (integer, default: 2, range: 0 - 16)
        Number of iterations of bilateral filter after CLAHE is applied
* `~balanced_bilateral_iter` (integer, default: 4, range: 0 - 8)
        Number of iterations of bilateral filter after white balancing is applied
* `~denoise_h` (double, default: 5.6, range: 0.0 - 20.0)
        h value for fast non-local means denoising applied on the final blue-filtered image
* `~low_h` (int, default: 53, range: 0 - 255)
        Lower Bound of H
* `~high_h` (int, default: 86, range: 0 - 255)
        Higher Bound of H
* `~low_s` (int, default: 128, range: 0 - 255)
        Lower Bound of S
* `~high_s` (int, default: 255, range: 0 - 255)
        Higher Bound of S
* `~low_v` (int, default: 104, range: 0 - 255)
        Lower Bound of V
* `~high_v` (int, default: 202, range: 0 - 255)
        Higher Bound of V
* `~opening_mat_point` (int, default: 2, range: 1 - 7)
        Center of the matrix for the opening operation (size extrapolated 2x+1)
* `~opening_iter` (int, default: 1, range: 0 - 10)
        Iterations of opening applied on the thresholded image
* `~closing_mat_point` (int, default: 2, range: 1 - 7)
        Center of the matrix for the closing operation (size extrapolated 2x+1)
* `~closing_iter` (int, default: 3, range: 0 - 10)
        Iterations of closing applied on the opened thresholded image

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AUV-IITK/auv2018/issues).

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[geometry_msgs/PointStamped]: http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PointStamped.html
[geometry_msgs/Pose2D]: http://docs.ros.org/api/geometry_msgs/html/msg/Pose2D.html
