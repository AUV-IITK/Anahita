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
#include <vision_commons/rangeConfig.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>

int low_h = 0;
int low_s = 1;
int low_v = 2;
int high_h = 255;
int high_s = 255;
int high_v = 255;
float focal_length = 10;
float known_width = 25;
std::string camera_frame = "front_cam_link";

image_transport::Publisher thresholded_HSV_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_pub;


void callback(vision_commons::rangeConfig &config, double level){
	ROS_INFO("Reconfigure Request: (%d %d %d) - (%d %d %d): ", config.low_h, config.low_s, config.low_v, config.high_h, config.high_s, config.high_v);
	low_h = config.low_h;
	low_s = config.low_s;
	low_v = config.low_v;
	high_h = config.high_h;
	high_s = config.high_s;
	high_v = config.high_v;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	cv_bridge::CvImagePtr cv_img_ptr;
	try{
		cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_img_ptr->image;
		cv::Mat image_marked = cv_img_ptr->image;
		if(!image.empty()){
			cv::Mat image_hsv;
			cv::Mat image_thresholded;
			cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);
			ROS_INFO("Thresholding Values: (%d %d %d) - (%d %d %d): ", low_h, low_s, low_v, high_h, high_s, high_v);
			if(!(high_h<=low_h || high_s<=low_s || high_v<=low_v)) {
				inRange(image_hsv, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), image_thresholded);
				cv::Mat opening_closing_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));
				cv::morphologyEx(image_thresholded, image_thresholded, cv::MORPH_OPEN, opening_closing_kernel, cv::Point(-1, -1), 1);
				cv::morphologyEx(image_thresholded, image_thresholded, cv::MORPH_CLOSE, opening_closing_kernel, cv::Point(-1, -1), 1);
				cv_bridge::CvImage thresholded_ptr;
				thresholded_ptr.header = msg->header;
				thresholded_ptr.encoding = sensor_msgs::image_encodings::MONO8;
				thresholded_ptr.image = image_thresholded;
				thresholded_HSV_pub.publish(thresholded_ptr.toImageMsg());
				std::vector<std::vector<cv::Point> > contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(image_thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
				ROS_INFO("contours size = %d", contours.size());
				if (contours.size() != 0){
					int index = -1;
					float max_area = 0.0;
					float area = 0.0;
					for( int i = 0; i< contours.size(); i++ ){
						area = cv::contourArea(contours[i]);
						if(area >= max_area){
							index = i;
							max_area =area;
						}
					}
					// calculating center of mass of contour using moments
					std::vector<cv::Moments> mu(1);
					mu[0] = moments(contours[index], false);
					std::vector<cv::Point2f> mc(1);
					mc[0] = cv::Point2f( mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00 );
					cv::Rect bounding_rectangle = cv::boundingRect(cv::Mat(contours[index]));

					// publish coordinates message
					geometry_msgs::PointStamped buoy_point_message;
					buoy_point_message.header.stamp = ros::Time();
					buoy_point_message.header.frame_id = camera_frame.c_str();
					buoy_point_message.point.x = (bounding_rectangle.br().x + bounding_rectangle.tl().x)/2 - (image.size().width)/2;
					buoy_point_message.point.y = ((float)image.size().height)/2 - (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2;
					buoy_point_message.point.z = (known_width * focal_length) / (bounding_rectangle.br().x + bounding_rectangle.tl().x);
					ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", buoy_point_message.point.x, buoy_point_message.point.y, buoy_point_message.point.z);
					coordinates_pub.publish(buoy_point_message);
					cv::RotatedRect minEllipse;
					cv::cvtColor(image_marked, image_marked, cv::COLOR_BGR2HSV);
					if(contours[index].size()>=5){
						minEllipse = cv::fitEllipse(cv::Mat(contours[index]));
						cv::ellipse(image_marked, minEllipse, cv::Scalar(255,255,0), 2, 8 );
					}
					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x)/2, (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2), 10, cv::Scalar(255,100,100), 8, 0);
					cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 10, cv::Scalar(255,100, 50), 2, 8, 0);
					cv_bridge::CvImage marked_ptr;
					marked_ptr.header = msg->header;
					marked_ptr.encoding = sensor_msgs::image_encodings::RGB8;
					marked_ptr.image = image_marked;
					marked_pub.publish(marked_ptr.toImageMsg());
				}
			}
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
	ros::init(argc, argv, "thresholded");
	ros::NodeHandle nh;
	dynamic_reconfigure::Server<vision_commons::rangeConfig> server;
	dynamic_reconfigure::Server<vision_commons::rangeConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	image_transport::ImageTransport it(nh);
	thresholded_HSV_pub = it.advertise("/thresholded", 1);
	marked_pub = it.advertise("/marked",1);
	coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/buoy_processing/buoy_coordinates", 1000);
	image_transport::Subscriber image_raw_sub = it.subscribe("/blue_filtered", 1, imageCallback);
	ros::spin();
	return 0;
}
