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

#include <vision_tasks/lineRangeConfig.h>
#include <vision_commons/Pose2DStamped.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

int low_h, low_s, low_v, high_h, high_s, high_v;
image_transport::Publisher thresholded_HSV_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_pub;
std::string camera_frame = "auv-iitk";

void callback(vision_tasks::lineRangeConfig &config, double level){
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
				image_thresholded = vision_commons::Threshold::threshold(image, low_h, low_s, low_v, high_h, high_s, high_v);
				cv_bridge::CvImage thresholded_ptr;
				thresholded_ptr.header = msg->header;
				thresholded_ptr.encoding = sensor_msgs::image_encodings::MONO8;
				thresholded_ptr.image = image_thresholded;
				thresholded_HSV_pub.publish(thresholded_ptr.toImageMsg());
				std::vector<std::vector<cv::Point> > contours = vision_commons::Contour::getBestX(image_thresholded, 1);
				ROS_INFO("contours size = %d", contours.size());
				if (contours.size() != 0){
					// calculating center of mass of contour using moments
					std::vector<cv::Moments> mu(1);
					mu[0] = moments(contours[0], false);
					std::vector<cv::Point2f> mc(1);
					mc[0] = cv::Point2f( mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00 );
					cv::RotatedRect bounding_rectangle = cv::minAreaRect(cv::Mat(contours[0]));
					// publish coordinates message
					vision_commons::Pose2DStamped line_point_message;
					line_point_message.header.stamp = ros::Time();
					line_point_message.header.frame_id = camera_frame.c_str();
					line_point_message.pose.x = (image.size().height)/2 - bounding_rectangle.center.y;
					line_point_message.pose.y = bounding_rectangle.center.x - (image.size().width)/2;
					line_point_message.pose.theta = bounding_rectangle.angle;
					ROS_INFO("Line (x, y, theta) = (%.2f, %.2f, %.2f)", line_point_message.pose.x, line_point_message.pose.y, line_point_message.pose.theta);
					coordinates_pub.publish(line_point_message);
//					cv::cvtColor(image_marked, image_marked, cv::COLOR_BGR2HSV);
					cv::circle(image_marked, cv::Point(bounding_rectangle.center.x, bounding_rectangle.center.y), 1, cv::Scalar(0, 255, 255), 8, 0);
					cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(0, 255, 255), 8, 0);
					for( int i = 0; i< contours.size(); i++ ) {
						cv::Scalar color = cv::Scalar(255,255,100);
						cv::drawContours( image_marked, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
						// cv::drawContours( image_marked, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
					}
					cv::Point2f rect_points[4];
					bounding_rectangle.points(rect_points);
					for( int i = 0; i < 4; i++  )
						line( image_marked, rect_points[i], rect_points[(i+1)%4], cv::Scalar(0, 255, 255), 1, 8  );
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
	ros::init(argc, argv, "line_task");
	ros::NodeHandle nh;
	dynamic_reconfigure::Server<vision_tasks::lineRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::lineRangeConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	image_transport::ImageTransport it(nh);
	thresholded_HSV_pub = it.advertise("/line_task/thresholded", 1);
	marked_pub = it.advertise("/line_task/marked", 1);
	coordinates_pub = nh.advertise<vision_commons::Pose2DStamped>("/line_task/line_coordinates", 1000);
	image_transport::Subscriber image_raw_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}
