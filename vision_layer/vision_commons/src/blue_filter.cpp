#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/xphoto/white_balance.hpp"
#include "opencv2/photo/photo.hpp"
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

image_transport::Publisher white_balanced_pub;
image_transport::Publisher noise_reduced_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_img_ptr;
	try {
		cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_img_ptr->image;
		if(!image.empty()){

			cv::Mat white_balanced;
			cv::xphoto::createGrayworldWB()->balanceWhite(image, white_balanced);
			cv_bridge::CvImage white_balanced_ptr;
			white_balanced_ptr.header = msg->header;
			white_balanced_ptr.encoding = msg->encoding;
			white_balanced_ptr.image = white_balanced;

			cv::Mat lab_image;
			cv::cvtColor(image, lab_image, CV_BGR2Lab);
			std::vector<cv::Mat> lab_planes(3);
			cv::split(lab_image, lab_planes);
			cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(16, cv::Size(5, 5));
			clahe->setClipLimit(8);
			cv::Mat l0dst;
			clahe->apply(lab_planes[0], l0dst);
			cv::Mat l1dst;
			clahe->apply(lab_planes[1], l1dst);
			cv::Mat l2dst;
			clahe->apply(lab_planes[2], l2dst);
			l0dst.copyTo(lab_planes[0]);
			l1dst.copyTo(lab_planes[1]);
			l2dst.copyTo(lab_planes[2]);
			cv::merge(lab_planes, lab_image);
			cv::Mat noisy;
			cv::cvtColor(lab_image, noisy, CV_Lab2BGR);
			cv::Mat noise_reduced;
			cv::fastNlMeansDenoisingColored(noisy, noise_reduced, 10.0, 10.0, 5, 11);
			cv_bridge::CvImage noise_reduced_ptr;
			noise_reduced_ptr.header = msg->header;
			noise_reduced_ptr.encoding = msg->encoding;
			noise_reduced_ptr.image = noise_reduced;

			white_balanced_pub.publish(white_balanced_ptr.toImageMsg());
			noise_reduced_pub.publish(noise_reduced_ptr.toImageMsg());
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
	white_balanced_pub = it.advertise("/white_balanced", 1);
	noise_reduced_pub = it.advertise("/noise_reduced", 1);
	image_transport::Subscriber image_raw_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}
