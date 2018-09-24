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

double clahe_clip_ = 0.15;
int clahe_grid_size_ = 3;
int clahe_bilateral_iter_ = 2;
int balanced_bilateral_iter_ = 4;
double denoise_h_ = 10.0;
int low_h_ = 0;
int high_h_ = 10;
int low_s_ = 251;
int high_s_ = 255;
int low_v_ = 160;
int high_v_ = 255;
int opening_mat_point_ = 1;
int opening_iter_ = 0;
int closing_mat_point_ = 1;
int closing_iter_ = 0;

cv::Mat image_;

std::string camera_frame_ = "auv-iitk";

void callback(vision_tasks::buoyRangeConfig &config, double level)
{
	clahe_clip_ = config.clahe_clip;
	clahe_grid_size_ = config.clahe_grid_size;
	clahe_bilateral_iter_ = config.clahe_bilateral_iter;
	balanced_bilateral_iter_ = config.balanced_bilateral_iter;
	denoise_h_ = config.denoise_h;
	low_h_ = config.low_h;
	high_h_ = config.high_h;
	low_s_ = config.low_s;
	high_s_ = config.high_s;
	low_v_ = config.low_v;
	high_v_ = config.high_v;
	opening_mat_point_ = config.opening_mat_point;
	opening_iter_ = config.opening_iter;
	closing_mat_point_ = config.closing_mat_point;
	closing_iter_ = config.closing_iter;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_img_ptr;
	try
	{
		image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e)
	{
		ROS_ERROR("cv exception: %s", e.what());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "buoy_task");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<vision_tasks::buoyRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::buoyRangeConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher blue_filtered_pub = it.advertise("/buoy_task/blue_filtered", 1);
	image_transport::Publisher thresholded_pub = it.advertise("/buoy_task/thresholded", 1);
	image_transport::Publisher marked_pub = it.advertise("/buoy_task/marked", 1);
	ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/buoy_task/buoy_coordinates", 1000);

	image_transport::Subscriber image_raw_sub = it.subscribe("/bottom_camera/image_raw", 1, imageCallback);

	cv::Scalar buoy_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar enclosing_circle_color(149, 255, 23);
	cv::Scalar contour_color(255, 0, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_marked;
	std::vector<std::vector<cv::Point> > contours;
	cv::Rect bounding_rectangle;
	std::vector<cv::Point2f> center(1);
	std::vector<float> radius(1);
	geometry_msgs::PointStamped buoy_point_message;
	buoy_point_message.header.frame_id = camera_frame_.c_str();
	cv::RotatedRect min_ellipse;

	while (ros::ok())
	{
		if (!image_.empty())
		{
			image_.copyTo(image_marked);
			blue_filtered = vision_commons::Filter::blue_filter(image_, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
			if (high_h_ > low_h_ && high_s_ > low_s_ && high_v_ > low_v_)
			{
				cv::cvtColor(blue_filtered, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, low_h_, high_h_, low_s_, high_s_, low_v_, high_v_);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * opening_mat_point_ + 1, opening_mat_point_, opening_mat_point_, opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * closing_mat_point_ + 1, closing_mat_point_, closing_mat_point_, closing_iter_);
				contours = vision_commons::Contour::getBestX(image_thresholded, 2);
				if (contours.size() != 0)
				{
					int index = 0;
					bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
					if (contours.size() >= 2)
					{
						cv::Rect bounding_rectangle2 = cv::boundingRect(cv::Mat(contours[1]));
						if ((bounding_rectangle2.br().y + bounding_rectangle2.tl().y) > (bounding_rectangle.br().y + bounding_rectangle.tl().y))
						{
							index = 1;
							bounding_rectangle = bounding_rectangle2;
						}
					}
					cv::minEnclosingCircle(contours[index], center[0], radius[0]);
					buoy_point_message.header.stamp = ros::Time();
					buoy_point_message.point.x = pow(radius[0] / 7526.5, -.92678);
					buoy_point_message.point.y = center[0].x - ((float)image_.size().width) / 2;
					buoy_point_message.point.z = ((float)image_.size().height) / 2 - center[0].y;
					ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", buoy_point_message.point.x, buoy_point_message.point.y, buoy_point_message.point.z);
					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2, (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2), 1, buoy_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_.size().width / 2, image_.size().height / 2), 1, image_center_color, 8, 0);
					cv::circle(image_marked, center[0], (int)radius[0], enclosing_circle_color, 2, 8, 0);
					for (int i = 0; i < contours.size(); i++)
					{
						cv::drawContours(image_marked, contours, i, contour_color, 1);
					}
				}
			}
			blue_filtered_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "bgr8", blue_filtered).toImageMsg());
			thresholded_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "mono8", image_thresholded).toImageMsg());
			coordinates_pub.publish(buoy_point_message);
			ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", buoy_point_message.point.x, buoy_point_message.point.y, buoy_point_message.point.z);
			marked_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty");
		ros::spinOnce();
	}
	return 0;
}
