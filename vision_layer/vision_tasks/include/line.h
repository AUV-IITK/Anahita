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
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>

#include <boost/thread.hpp> 

#include <vision_tasks/lineRangeConfig.h>
#include <filter.h>
#include <contour.h>
#include <morph.h>
#include <threshold.h>

class Line
{
protected:

	image_transport::Publisher thresholded_pub;
	image_transport::Publisher marked_pub;
    image_transport::Subscriber image_raw_sub;

   	ros::Publisher detection_pub;
	ros::Publisher coordinates_pub;
	
	std::string camera_frame_;

    int low_h_ = 0;
	int high_h_ = 80;
	int low_s_ = 0;
	int high_s_ = 255;
	int low_v_ = 0;
	int high_v_ = 255;
	int opening_mat_point_=1;
	int opening_iter_=3;
	int closing_mat_point_=1;
	int closing_iter_=0;

	bool task_done = false;
	
	ros::Publisher x_coordinates_pub;
	ros::Publisher y_coordinates_pub;
	ros::Publisher z_coordinates_pub;

	ros::Publisher ang_pub;
	
	void callback(vision_tasks::lineRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
    double computeMean(std::vector<double> &newAngles);

public:
    Line();
	~Line();
    ros::NodeHandle nh;
	std_msgs::Float32 x_coordinate;
	std_msgs::Float32 y_coordinate;
	std_msgs::Float32 z_coordinate;	
	image_transport::ImageTransport it;
	cv::Mat image_;
	cv::Mat image_marked;
	boost::thread* spin_thread; 
	void TaskHandling(bool status);
	void spinThread();
};
#endif // LINE_TASK_H

