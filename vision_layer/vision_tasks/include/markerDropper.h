#ifndef MARKERDROPPER_TASK_H
#define MARKERDROPPER_TASK_H

// #include "ros/ros.h"
// #include "sensor_msgs/Image.h"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/core/core.hpp"
// #include "opencv2/imgproc/imgproc_c.h"
// #include "opencv2/highgui/highgui.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <dynamic_reconfigure/server.h>
// #include <geometry_msgs/Potamped.h>
// #include <sensor_msgs/image_encodings.h>
// #include <bits/stdc++.h>
// #include <stdlib.h>
// #include <string>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
// #include <boost/thread.hpp> 

#include <vision_tasks/markerDropperFrontRangeConfig.h>
#include <vision_tasks/markerDropperBottomRangeConfig.h>
// #include <vision_commons/filter.h>
// #include <vision_commons/contour.h>
// #include <vision_commons/morph.h>
// #include <vision_commons/threshold.h>

#include "base_class.h"

class MarkerDropper: public Base_class
{
protected:

    front_clahe_clip_ = 4.0;
    front_clahe_grid_size_ = 8;
    front_clahe_bilateral_iter_ = 8;
    front_balanced_bilateral_iter_ = 4;
    front_denoise_h_ = 10.0;
    front_low_h_ = 0;
    front_high_h_ = 255;
    front_low_s_ = 0;
    front_high_s_ = 255;
    front_low_v_ = 0;
    front_high_v_ = 255;
    front_closing_mat_point_ = 1;
    front_closing_iter_ = 1;
    front_opening_mat_point_ = 1;	
	front_opening_iter_ = 1;
    front_canny_threshold_low_ = 0;
    front_canny_threshold_high_ = 1000;
    front_canny_kernel_size_ = 3;
    front_hough_threshold_ = 0;
    front_hough_minline_ = 0;
    front_hough_maxgap_ = 0;
    front_hough_angle_tolerance_ = 0.0;

    bottom_clahe_clip_ = 4.0;
    bottom_clahe_grid_size_ = 8;
    bottom_clahe_bilateral_iter_ = 8;
    bottom_balanced_bilateral_iter_ = 4;
    bottom_denoise_h_ = 10.0;
    bottom_low_h_ = 10;
    bottom_low_s_ = 0;
    bottom_low_v_ = 0;
    bottom_high_h_ = 90;
    bottom_high_s_ = 255;
    bottom_high_v_ = 255;
    bottom_closing_mat_point_ = 1;
    bottom_closing_iter_ = 1;
    bottom_opening_iter_ = 1;    
    bottom_opening_mat_point_ = 1;

	// void frontCallback(vision_tasks::markerDropperFrontRangeConfig &config, double level);
	// void bottomCallback(vision_tasks::markerDropperBottomRangeConfig &config, double level);
    
public:
    MarkerDropper();
};
#endif // MARKERDROPPER_TASK_H

