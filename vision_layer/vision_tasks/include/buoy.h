#ifndef BUOY_TASK_H
#define BUOY_TASK_H

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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <string>
#include <boost/thread.hpp> 

#include <vision_tasks/buoyRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

#define SHELLSCRIPT_DUMP_RED_BUOY "\
#/bin/bash \n\
echo -e \"Parameters dumped!!\" \n\
rosparam dump -v ~/Projects/anahita/src/Anahita/vision_layer/vision_tasks/thresholding/red_buoy.yaml /vision_node\
"
#define SHELLSCRIPT_LOAD_RED_BUOY "\
#/bin/bash \n\
echo -e \"Parameters loaded!!\" \n\
rosparam load -v ~/Projects/anahita/src/Anahita/vision_layer/vision_tasks/thresholding/red_buoy.yaml /vision_node\
"

class Buoy
{
protected:

    ros::NodeHandle nh;
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

	int data_low_h[3] = {0, 0, 0};
	int data_high_h[3] = {127, 8, 74};
	int data_low_s[3] = {0, 49, 65};
	int data_high_s[3] = {105, 243, 155};
	int data_low_v[3] = {248, 36, 0};
	int data_high_v[3] = {255, 255, 183};

	void setParams(ros::NodeHandle &nh);

	void callback(vision_tasks::buoyRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
	Buoy();
	~Buoy();
	std_msgs::Float32 x_coordinate;
	std_msgs::Float32 y_coordinate;
	std_msgs::Float32 z_coordinate;
	cv::Mat image_;
	cv::Mat image_marked;
	boost::thread* spin_thread; 
	image_transport::ImageTransport it;
	void TaskHandling(bool);
	void switchColor(int);
	void spinThread();
	int current_color;
	bool close_task = false;
	bool task_on = false;
};
#endif // BUOY_TASK_H
