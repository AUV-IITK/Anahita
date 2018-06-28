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

double clahe_clip = 4.0;
int clahe_grid_size = 8;
int clahe_bilateral_iter = 8;
int balanced_bilateral_iter = 4;
double denoise_h = 10.0;
int low_b = 0;
int high_b = 130;
int low_g = 0;
int high_g = 123;
int low_r = 95;
int high_r = 255;
int opening_closing_mat_point, opening_iter, closing_iter;
image_transport::Publisher clahe_pub;
image_transport::Publisher white_balanced_pub;
image_transport::Publisher blue_filtered_pub;
image_transport::Publisher thresholded_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_pub;
std::string camera_frame = "auv-iitk";
geometry_msgs::PointStamped buoy_point_message;

void callback(vision_tasks::buoyRangeConfig &config, double level){
	clahe_clip = config.clahe_clip;
	clahe_grid_size = config.clahe_grid_size;
	clahe_bilateral_iter = config.clahe_bilateral_iter;
	balanced_bilateral_iter = config.balanced_bilateral_iter;
	denoise_h = config.denoise_h;
	low_b = config.low_b;
	low_g = config.low_g;
	low_r = config.low_r;
	high_b = config.high_b;
	high_g = config.high_g;
	high_r = config.high_r;
	opening_closing_mat_point = config.opening_closing_mat_point;
	opening_iter = config.opening_iter;
	closing_iter = config.closing_iter;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	cv_bridge::CvImagePtr cv_img_ptr;
	try{
		cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_img_ptr->image;
		cv::Mat image_marked = cv_img_ptr->image;
		if(!image.empty()){
			cv::Mat blue_filtered = vision_commons::Filter::blue_filter(image, clahe_clip, clahe_grid_size, clahe_bilateral_iter, balanced_bilateral_iter, denoise_h);
			blue_filtered_pub.publish(cv_bridge::CvImage(msg->header, "bgr8", blue_filtered).toImageMsg());
			if(!(high_b<=low_b || high_g<=low_g || high_r<=low_r)) {
				cv::Mat image_thresholded = vision_commons::Threshold::threshold(blue_filtered, low_b, low_g, low_r, high_b, high_g, high_r);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2*opening_closing_mat_point+1, opening_closing_mat_point, opening_closing_mat_point, opening_iter);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2*opening_closing_mat_point+1, opening_closing_mat_point, opening_closing_mat_point, closing_iter);
				thresholded_pub.publish(cv_bridge::CvImage(msg->header, "mono8", image_thresholded).toImageMsg());
				std::vector<std::vector<cv::Point> > contours = vision_commons::Contour::getBestX(image_thresholded, 2);
				if (contours.size() != 0){
					std::vector<cv::Moments> mu(1);
					mu[0] = moments(contours[0], false);
					std::vector<cv::Point2f> mc(1);
					mc[0] = cv::Point2f( mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00 );
					int index = 0;
					cv::Rect bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
					if(contours.size() >= 2) {
						cv::Rect bounding_rectangle2 = cv::boundingRect(cv::Mat(contours[1]));
						if((bounding_rectangle2.br().y + bounding_rectangle2.tl().y) > (bounding_rectangle.br().y + bounding_rectangle.tl().y)) {
							index = 1;
							bounding_rectangle = bounding_rectangle2;
						}
					}
					std::vector<cv::Point2f>center(1);
					std::vector<float>radius(1);
					cv::minEnclosingCircle(contours[index], center[0], radius[0]);
					cv::circle(image_marked, center[0], (int)radius[0], cv::Scalar(180,180,180), 2, 8, 0);
					buoy_point_message.header.stamp = ros::Time();
					buoy_point_message.header.frame_id = camera_frame.c_str();
					buoy_point_message.point.x = pow(radius[0]/7526.5,-.92678);
					buoy_point_message.point.y = (bounding_rectangle.br().x + bounding_rectangle.tl().x)/2 - (image.size().width)/2;
					buoy_point_message.point.z = ((float)image.size().height)/2 - (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2;
					ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", buoy_point_message.point.x, buoy_point_message.point.y, buoy_point_message.point.z);
					coordinates_pub.publish(buoy_point_message);
					cv::RotatedRect minEllipse;
					cv::cvtColor(image_marked, image_marked, cv::COLOR_BGR2HSV);
					if(contours[index].size()>=5){
						minEllipse = cv::fitEllipse(cv::Mat(contours[index]));
						cv::ellipse(image_marked, minEllipse, cv::Scalar(255,255,0), 2, 8 );
					}
					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x)/2, (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2), 1, cv::Scalar(255,100,100), 8, 0);
					cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(255,100, 50), 8, 0);
					for( int i = 0; i< contours.size(); i++ ) {
						cv::Scalar color = cv::Scalar(255,255,100);
						cv::drawContours( image_marked, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
					}
					marked_pub.publish(cv_bridge::CvImage(msg->header, "rgb8", image_marked).toImageMsg());
				}
				else{
					if(buoy_point_message.header.frame_id=="auv-iitk"){
						ROS_INFO("Could not find buoy, publishing last known information...");
						ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", buoy_point_message.point.x, buoy_point_message.point.y, buoy_point_message.point.z);
						coordinates_pub.publish(buoy_point_message);
					}
					else{
						ROS_INFO("No buoy found yet, move the bot in the expected direction...");
					}
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
	ros::init(argc, argv, "buoy_task");
	ros::NodeHandle nh;
	dynamic_reconfigure::Server<vision_tasks::buoyRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::buoyRangeConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	image_transport::ImageTransport it(nh);
	thresholded_pub = it.advertise("/buoy_task/thresholded", 1);
	blue_filtered_pub = it.advertise("/buoy_task/blue_filtered", 1);
	marked_pub = it.advertise("/buoy_task/marked",1);
	coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/buoy_task/buoy_coordinates", 1000);
	image_transport::Subscriber image_raw_sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}
