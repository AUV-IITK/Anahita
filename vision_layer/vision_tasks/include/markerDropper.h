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
#include <filter.h>
#include <contour.h>
#include <morph.h>
#include <threshold.h>

class MarkerDropper
{
protected:
	ros::NodeHandle nh;
    
    image_transport::Publisher bottom_blue_filtered_pub;
	image_transport::Publisher bottom_thresholded_pub;
	image_transport::Publisher bottom_thresholded_blue_pub;    
	image_transport::Publisher bottom_marked_pub;
	
	image_transport::Publisher front_blue_filtered_pub;
	image_transport::Publisher front_thresholded_pub;
	image_transport::Publisher front_marked_pub;

	image_transport::Subscriber front_image_raw_sub;
	image_transport::Subscriber bottom_image_raw_sub;
  	ros::Publisher detection_pub;

    ros::Publisher x_coordinates_pub;
	ros::Publisher y_coordinates_pub;
	ros::Publisher z_coordinates_pub;
	
	std::string camera_frame_;

    double front_clahe_clip_;
    int front_clahe_grid_size_;
    int front_clahe_bilateral_iter_;
    int front_balanced_bilateral_iter_;
    double front_denoise_h_;
    int front_low_h_;
    int front_high_h_;
    int front_low_s_;
    int front_high_s_;
    int front_low_v_;
    int front_high_v_;
    int front_closing_mat_point_;
    int front_closing_iter_;
    int front_opening_mat_point_;	
	int front_opening_iter_;
    int front_canny_threshold_low_;
    int front_canny_threshold_high_;
    int front_canny_kernel_size_;
    int front_hough_threshold_;
    int front_hough_minline_;
    int front_hough_maxgap_;
    double front_hough_angle_tolerance_;
    double front_gate_distance_tolerance_;
    double front_gate_angle_tolerance_;

    double bottom_clahe_clip_;
    int bottom_clahe_grid_size_;
    int bottom_clahe_bilateral_iter_;
    int bottom_balanced_bilateral_iter_;
    double bottom_denoise_h_;
    int bottom_low_h_;
    int bottom_low_s_;
    int bottom_low_v_;
    int bottom_high_h_;
    int bottom_high_s_;
    int bottom_high_v_;
    int bottom_closing_mat_point_;
    int bottom_closing_iter_;
    int bottom_opening_iter_;    
    int bottom_opening_mat_point_;

    int bottom_low_h_blue;
    int bottom_low_s_blue;
    int bottom_low_v_blue;
    int bottom_high_h_blue;
    int bottom_high_s_blue;
    int bottom_high_v_blue;
    int bottom_closing_mat_point_blue;
    int bottom_closing_iter_blue;
    int bottom_opening_iter_blue;    
    int bottom_opening_mat_point_blue;

    bool close_task = false;

	void frontCallback(vision_tasks::markerDropperFrontRangeConfig &config, double level);
	void bottomCallback(vision_tasks::markerDropperBottomRangeConfig &config, double level);
	void imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg);
    void imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
    MarkerDropper();
    ~MarkerDropper();
	image_transport::ImageTransport it;
	cv::Mat image_front;
    cv::Mat image_bottom;
    boost::thread* spin_thread_bottom; 
    boost::thread* spin_thread_front; 

	cv::VideoCapture cap;
    
	cv::Mat image_marked;
	// void TaskHandling(bool status);
	void bottomTaskHandling(bool status);
	void frontTaskHandling(bool status);
    void spinThreadBottom();
    void spinThreadFront();

    std_msgs::Float32 x_coordinate;
	std_msgs::Float32 y_coordinate;
	std_msgs::Float32 z_coordinate;
};
#endif // MARKERDROPPER_TASK_H

