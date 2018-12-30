#ifndef TORPEDO_TASK_H
#define TORPEDO_TASK_H

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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>
#include <boost/thread.hpp> 

#include <vision_tasks/torpedoRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

class Torpedo
{
protected:

    image_transport::Publisher blue_filtered_pub;
	image_transport::Publisher thresholded_pub;
	image_transport::Publisher marked_pub;
	ros::Publisher x_coordinates_pub;
	ros::Publisher y_coordinates_pub;
	ros::Publisher z_coordinates_pub;
	ros::Publisher detection_pub;		
	
	image_transport::Subscriber image_raw_sub;
	std::string camera_frame_;

    double clahe_clip_;
	int clahe_grid_size_;
	int clahe_bilateral_iter_;
	int balanced_bilateral_iter_;
	double denoise_h_;
	int low_h_;
	int high_h_;
	int low_s_;
	int high_s_;
	int low_v_;
	int high_v_;
	int opening_mat_point_;
	int opening_iter_;
	int closing_mat_point_;
	int closing_iter_;

	int data_low_h[2] = {7, 7};
	int data_high_h[2] = {157, 17};
	int data_low_s[2] = {0, 0};
	int data_high_s[2] = {173, 255};
	int data_low_v[2] = {0, 21};
	int data_high_v[2] = {255, 117};

	bool close_task = false;

	void callback(vision_tasks::torpedoRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
    Torpedo();
    ros::NodeHandle nh;
	image_transport::ImageTransport it();
	cv::Mat image_;
	cv::Mat image_marked;
	boost::thread* spin_thread; 
	void TaskHandling(bool);
	void switchColor(int);
	void spinThread();
	int current_color;
	std_msgs::Float32 x_coordinate;
	std_msgs::Float32 y_coordinate;
	std_msgs::Float32 z_coordinate;
};
#endif // TORPEDO_TASK_H

