#ifndef MARKERDROPPER_TASK_H
#define MARKERDROPPER_TASK_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <boost/thread.hpp> 

#include <vision_tasks/markerDropperFrontRangeConfig.h>
#include <vision_tasks/markerDropperBottomRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

class MarkerDropper
{
protected:
	ros::NodeHandle nh;
    
    image_transport::Publisher bottom_blue_filtered_pub;
	image_transport::Publisher bottom_thresholded_pub;
	image_transport::Publisher bottom_marked_pub;
	ros::Publisher bottom_coordinates_pub;
	
	image_transport::Publisher front_blue_filtered_pub;
	image_transport::Publisher front_thresholded_pub;
	image_transport::Publisher front_marked_pub;
	ros::Publisher front_coordinates_pub;

	image_transport::Subscriber front_image_raw_sub;
	image_transport::Subscriber bottom_image_raw_sub;
  	ros::Publisher detection_pub;

	std::string camera_frame_;

    double front_clahe_clip_ = 4.0;
    int front_clahe_grid_size_ = 8;
    int front_clahe_bilateral_iter_ = 8;
    int front_balanced_bilateral_iter_ = 4;
    double front_denoise_h_ = 10.0;
    int front_low_h_ = 0;
    int front_high_h_ = 255;
    int front_low_s_ = 0;
    int front_high_s_ = 255;
    int front_low_v_ = 0;
    int front_high_v_ = 255;
    int front_closing_mat_point_ = 1;
    int front_closing_iter_ = 1;
    int front_opening_mat_point_ = 1;	
	int front_opening_iter_ = 1;
    int front_canny_threshold_low_ = 0;
    int front_canny_threshold_high_ = 1000;
    int front_canny_kernel_size_ = 3;
    int front_hough_threshold_ = 0;
    int front_hough_minline_ = 0;
    int front_hough_maxgap_ = 0;
    double front_hough_angle_tolerance_ = 0.0;
    double front_gate_distance_tolerance_ = 50.0;
    double front_gate_angle_tolerance_ = 0.0;

    double bottom_clahe_clip_ = 4.0;
    int bottom_clahe_grid_size_ = 8;
    int bottom_clahe_bilateral_iter_ = 8;
    int bottom_balanced_bilateral_iter_ = 4;
    double bottom_denoise_h_ = 10.0;
    int bottom_low_h_ = 10;
    int bottom_low_s_ = 0;
    int bottom_low_v_ = 0;
    int bottom_high_h_ = 90;
    int bottom_high_s_ = 255;
    int bottom_high_v_ = 255;
    int bottom_closing_mat_point_ = 1;
    int bottom_closing_iter_ = 1;
    int bottom_opening_iter_ = 1;    
    int bottom_opening_mat_point_ = 1;

	void frontCallback(vision_tasks::markerDropperFrontRangeConfig &config, double level);
	void bottomCallback(vision_tasks::markerDropperBottomRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
    MarkerDropper();
	image_transport::ImageTransport it();
	cv::Mat image_;
    boost::thread* spin_thread_bottom; 
    boost::thread* spin_thread_front; 

	cv::Mat image_marked;
	void TaskHandling(bool status);
	void BottomTaskHandling();
	void FrontTaskHandling();  
};
#endif // MARKERDROPPER_TASK_H

