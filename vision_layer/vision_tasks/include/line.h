#ifndef LINE_TASK_H
#define LINE_TASK_H

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
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>

#include <vision_tasks/lineRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

class Line
{
protected:

	image_transport::Publisher thresholded_pub;
	image_transport::Publisher marked_pub;
    image_transport::Subscriber image_raw_sub;

   	ros::Publisher detection_pub;
	ros::Publisher coordinates_pub;
	
	std::string camera_frame_;

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

	bool task_done = false;
	
	ros::Publisher x_coordinates_pub;
	ros::Publisher y_coordinates_pub;
	ros::Publisher z_coordinates_pub;
	
	void callback(vision_tasks::lineRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
    double computeMean(std::vector<double> &newAngles);

public:
    Line();
    ros::NodeHandle nh;
	std_msgs::Float32 x_coordinate;
	std_msgs::Float32 y_coordinate;
	std_msgs::Float32 z_coordinate;	
	image_transport::ImageTransport it();
	cv::Mat image_;
	cv::Mat image_marked;
	void TaskHandling();
};
#endif // LINE_TASK_H

