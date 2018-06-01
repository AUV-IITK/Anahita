#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/xphoto/white_balance.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

image_transport::Publisher blue_filtered_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_img_ptr;
	try {
		cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_img_ptr->image;
		if(!image.empty()){
			cv::Mat blue_filtered;
			cv::xphoto::createGrayworldWB()->balanceWhite(image, blue_filtered);
			cv_bridge::CvImage image_ptr;
			image_ptr.header = msg->header;
			image_ptr.encoding = msg->encoding;
			image_ptr.image = blue_filtered;
			blue_filtered_pub.publish(image_ptr.toImageMsg());
		}
	}
	catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

int main(int argc, char **argv) {
	cv::Mat image;
	image = cv::imread("/home/heisenberg/Pictures/Screenshot from 2018-06-01 10-27-20.png", CV_LOAD_IMAGE_COLOR);
	ros::init(argc, argv, "blue_filter");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	blue_filtered_pub = it.advertise("/blue_filtered", 1);
	image_transport::Subscriber image_raw_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}
