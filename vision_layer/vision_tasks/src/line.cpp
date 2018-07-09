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
#include <geometry_msgs/Pose2D.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#include <vision_tasks/lineRangeConfig.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

int low_h = 0;
int low_s = 0;
int low_v = 0;
int high_h = 255;
int high_s = 255;
int high_v = 255;
int opening_closing_mat_point = 1;
int opening_iter = 1;
int closing_iter = 1;
cv::Mat image;

std::string camera_frame = "auv-iitk";

void callback(vision_tasks::lineRangeConfig &config, double level){
	ROS_INFO("Reconfigure Request: (%d %d %d) - (%d %d %d): ", config.low_h, config.low_s, config.low_v, config.high_h, config.high_s, config.high_v);
	low_h = config.low_h;
	low_s = config.low_s;
	low_v = config.low_v;
	high_h = config.high_h;
	high_s = config.high_s;
	high_v = config.high_v;
	opening_closing_mat_point = config.opening_closing_mat_point;
	opening_iter = config.opening_iter;
	closing_iter = config.closing_iter;
}

double computeMode(std::vector<double> &newAngles) {
	double minDeviation = 5.0;
	double mode = newAngles[0];
	int freq = 1;
	int tempFreq;
	double diff;
	for (int i = 0; i < newAngles.size(); i++) {
		tempFreq = 1;
		for (int j = i + 1; j < newAngles.size(); j++) {
			diff = newAngles[j] - newAngles[i] > 0.0 ? newAngles[j] - newAngles[i] : newAngles[i] - newAngles[j];
			if (diff <= minDeviation) {
				tempFreq++;
				newAngles.erase(newAngles.begin() + j);
				j = j - 1;
			}
		}
		if (tempFreq >= freq){
			mode = newAngles[i];
			freq = tempFreq;
		}
	}
	return mode;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	try { image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image; }
	catch(cv_bridge::Exception &e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }
	catch(cv::Exception &e) { ROS_ERROR("cv exception: %s", e.what()); }
}

int main(int argc, char **argv){
	ros::init(argc, argv, "line_task");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<vision_tasks::lineRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::lineRangeConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher thresholded_pub = it.advertise("/line_task/thresholded", 1);
	image_transport::Publisher marked_pub = it.advertise("/line_task/marked", 1);
	ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::Pose2D>("/line_task/line_coordinates", 1000);

	image_transport::Subscriber image_raw_sub = it.subscribe("/bottom_camera/image_raw", 1, imageCallback);

	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::RotatedRect bounding_rectangle;
	std::vector<cv::Vec4i> lines;
	std::vector<double> angles;
	geometry_msgs::Pose2D line_point_message;
	cv::Point2f rect_points[4];
	cv::Mat image_marked;

	while(ros::ok()) {
		if(!image.empty()){
			image.copyTo(image_marked);
			cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);
			if(!(high_h<=low_h || high_s<=low_s || high_v<=low_v)) {
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, low_h, low_s, low_v, high_h, high_s, high_v);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2*opening_closing_mat_point+1, opening_closing_mat_point, opening_closing_mat_point, opening_iter);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2*opening_closing_mat_point+1, opening_closing_mat_point, opening_closing_mat_point, closing_iter);
				std::vector<std::vector<cv::Point> > contours = vision_commons::Contour::getBestX(image_thresholded, 1);
				cv::Mat edges(image_thresholded.rows, image_thresholded.cols, CV_8UC1, cv::Scalar::all(0));
				cv::drawContours(edges, contours, 0, cv::Scalar(255, 255, 255), 2, 8, std::vector<cv::Vec4i>());
				if (contours.size() != 0){
					bounding_rectangle = cv::minAreaRect(cv::Mat(contours[0]));
					cv::HoughLinesP(edges, lines, 1, CV_PI/180, 60, 70, 10 );
					for(int i = 0; i < lines.size(); i++) {
						if ((lines[i][2] == lines[i][0]) || (lines[i][1] == lines[i][3]))
							continue;
						angles.push_back(atan(static_cast<double>(lines[i][2] - lines[i][0]) / (lines[i][1] - lines[i][3]))*180.0/3.14159);
						cv::line(image_marked, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 1, CV_AA );
					}
					line_point_message.x = (image.size().height)/2 - bounding_rectangle.center.y;
					line_point_message.y = bounding_rectangle.center.x - (image.size().width)/2;
					if(angles.size() > 0) {
						float angle = computeMode(angles);
						if(angle > 90.0) line_point_message.theta = angle - 90.0;
						else line_point_message.theta = angle;
					}
					else line_point_message.theta = 0.0;
					ROS_INFO("Line (x, y, theta) = (%.2f, %.2f, %.2f)", line_point_message.x, line_point_message.y, line_point_message.theta);
					cv::circle(image_marked, cv::Point(bounding_rectangle.center.x, bounding_rectangle.center.y), 1, cv::Scalar(0, 255, 255), 8, 0);
					cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(0, 255, 255), 8, 0);
					for( int i = 0; i< contours.size(); i++ ) {
						cv::Scalar color = cv::Scalar(255, 255, 100);
						cv::drawContours(image_marked, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
					}
					bounding_rectangle.points(rect_points);
					for( int i = 0; i < 4; i++  )
						line(image_marked, rect_points[i], rect_points[(i+1)%4], cv::Scalar(0, 255, 255), 1, 8);
				}
			}
			thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());
			coordinates_pub.publish(line_point_message);
			marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_marked).toImageMsg());
		}
		else ROS_INFO("Image empty");
		ros::spinOnce();
	}
	return 0;
}
