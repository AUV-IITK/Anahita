#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

int low_h = 0;
int low_s = 1;
int low_v = 2;
int high_h = 255;
int high_s = 255;
int high_v = 255;

image_transport::Publisher thresholded_HSV_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	cv_bridge::CvImagePtr cv_img_ptr;
	try{
		cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_img_ptr->image;
		if(!image.empty()){

			cv::Mat image_hsv;
			cv::Mat image_thresholded;
			cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);
			inRange(image_hsv, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), image_thresholded);
			cv_bridge::CvImage thresholded_ptr;
			thresholded_ptr.header = msg->header;
			thresholded_ptr.encoding = sensor_msgs::image_encodings::MONO8;
			thresholded_ptr.image = image_thresholded;

			thresholded_HSV_pub.publish(thresholded_ptr.toImageMsg());
		}
	}
	catch(cv_bridge::Exception &e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch(cv::Exception &e){
		ROS_ERROR("cv exception: %s", e.what());
	}
}

int main(int argc, char **argv){
	cv::Mat image;
	ros::init(argc, argv, "thresholded");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	thresholded_HSV_pub = it.advertise("/thresholded", 1);
	image_transport::Subscriber image_raw_sub = it.subscribe("/kraken/front_camera", 1, imageCallback);
	ros::spin();
	return 0;
}