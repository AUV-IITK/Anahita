#ifndef GATE_TASK_H
#define GATE_TASK_H

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

#include <vision_tasks/gateFrontRangeConfig.h>
#include <vision_tasks/gateBottomRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>
#include <vision_commons/geometry.h>

class Gate
{
protected:
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

    bool task_done = false;

    image_transport::Subscriber front_image_sub;
    image_transport::Subscriber bottom_image_sub;

	image_transport::Publisher blue_filtered_pub_front;
	image_transport::Publisher thresholded_pub_front; 
	image_transport::Publisher canny_pub_front;
	image_transport::Publisher lines_pub_front;
	image_transport::Publisher marked_pub_front;
	ros::Publisher x_coordinates_pub;
	ros::Publisher y_coordinates_pub;
	ros::Publisher z_coordinates_pub;

    image_transport::Publisher blue_filtered_pub_bottom;
    image_transport::Publisher thresholded_pub_bottom;
    image_transport::Publisher marked_pub_bottom;
    ros::Publisher coordinates_pub_bottom;
    ros::Publisher task_done_pub;
   	ros::Publisher detection_pub;

    std::string camera_frame_;
	void frontCallback(vision_tasks::gateFrontRangeConfig &config, double level);
	void bottomCallback(vision_tasks::gateBottomRangeConfig &config, double level);    
	void imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg);
    void imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv::Point2i rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle);

public:
    Gate();
    ~Gate();
    ros::NodeHandle nh;
	image_transport::ImageTransport it;
	cv::Mat image_front;
    cv::Mat image_bottom;
	cv::Mat image_marked;
    boost::thread* spin_thread_bottom; 
    boost::thread* spin_thread_front; 
    // void TaskHandling(bool status);
	void bottomTaskHandling(bool status);
    void frontTaskHandling(bool status);
    void spinThreadFront();
    void spinThreadBottom();
    std_msgs::Float32 x_coordinate;
	std_msgs::Float32 y_coordinate;
	std_msgs::Float32 z_coordinate;
};
#endif // GATE_TASK_H
