# vision_commons

## Overview

This is a ROS package that abstracts common image processing operations and serves as a library for [`vision_tasks`](https://github.com/AUV-IITK/Hyperion-Software/tree/master/vision_layer/vision_tasks).

The `vision_commons` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](../../LICENSE).

## Setting up vision_commons

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
catkin_make --pkg vision_commons
```

## Usage

To run the blue filter demo, run:
```
rosrun vision_commons blue_filter_demo
```
All other files are libraries and hence cannot be executed.

To use the filtering library, add this line to your C++ source file:
```
#include <vision_commons/filter.h>
```
To use the geometry library, add this line to your C++ source file:
```
#include <vision_commons/geometry.h>
```
To use the morphing library, add this line to your C++ source file:
```
#include <vision_commons/morph.h>
```
To use the thresholding library, add this line to your C++ source file:
```
#include <vision_commons/threshold.h>
```

## Libraries

### contour

This module contains all the functions related to forming a contour around a thresholded region.

#### Functions
* **`std::vector<std::vector<cv::Point>> getBestX(cv::Mat& raw, int x)`**
        Returns best x (passed as int) contours from the passed binary image in the descending order of area
* **`std::vector<std::vector<cv::Point>> getBestXConvexHulled(cv::Mat& raw, int x)`**
        Returns best x (passed as int) contours using convex hull from the passed binary image in the descending order of area

### filter

This module contains all the functions related to filtering the image to enhance it.

#### Functions
* **`cv::Mat clahe(cv::Mat& image, double clahe_clip, int clahe_grid_size)`**
        Applies CLAHE to `image` with clip limit `clahe_clip` and the grid size of the operator `clahe_grid_size` and returns the resultant image
* **`cv::Mat balance_white(cv::Mat& image)`**
        Applies white balance to `image` with Gray World assumption and returns the resultant image
* **`cv::Mat blue_filter(cv::Mat& image, double clahe_clip, int clahe_grid_size, int clahe_bilateral_iter, int balanced_bilateral_iter, double denoise_h)`**
        "Blue filters" `image`. Applies CLAHE on `image` followed by `clahe_bilateral_iter` number of bilateral filter iterations followed by white balancing followed by `balanced_bilateral_iter` number of bilateral filter iterations followed by fast non-local means denoising with h value of `denoise_h`

### geometry

This module contains all the functions related to geometrical operations on points.

#### Functions
* **`double distance(cv::Point &p1, cv::Point &p2)`**
        Calculates distance between `p1` and `p2` using Pythagoras' Theorem
* **`double angleWrtY(cv::Point &p1, cv::Point &p2)`**
        Calculates angle of the line joining `p1` and `p2` with respect to the y-axis (vertical).

### morph

This module contains all the functions related to morphological operations.

#### Functions
* **`cv::Mat open(cv::Mat& raw, int element_size, int element_centerX, int element_centerY, int iterations)`**
        Applies "opening" operation on the image using a matrix of size `element_size`, center ar (`element_centerX`, `element_centerY`) `iterations` times
* **`cv::Mat close(cv::Mat &raw, int element_size, int element_centerX, int element_centerY, int iterations)`**
        Applies "closing" operation on the image using a matrix of size `element_size`, center ar (`element_centerX`, `element_centerY`) `iterations` times

### threshold

This module contains all the functions related to thresholding operations.

#### Functions
* **`cv::Mat threshold(cv::Mat &raw, int low_a, int high_a, int low_b, int high_b, int low_c, int high_c)`**
        Thresholds `raw` between (`low_a`, `low_b`, `low_c`) and (`high_a`, `high_b`, `high_c`) and returns the resultant binary image

## Nodes

### blue_filter_demo

This node serves for demonstration purposes of our pre-processing pipeline.

#### Published Topics
* **`/blue_filter_demo/clahe`** ([sensor_msgs/Image])
* **`/blue_filter_demo/white_balanced`** ([sensor_msgs/Image])
* **`/blue_filter_demo/blue_filtered`** ([sensor_msgs/Image])

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

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AUV-IITK/auv2018/issues).

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
