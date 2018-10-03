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
#include <string>

#include <vision_tasks/buoyRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

class Buoy
{
protected:
    image_transport::Publisher blue_filtered_pub;
	image_transport::Publisher thresholded_pub;
	image_transport::Publisher marked_pub;
	ros::Publisher coordinates_pub;
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
	void callback(vision_tasks::buoyRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
    Buoy();
    ros::NodeHandle nh;
	image_transport::ImageTransport it();
	cv::Mat image_;
	cv::Mat image_marked;
	void TaskHandling();
};
#endif // BUOY_TASK_H
