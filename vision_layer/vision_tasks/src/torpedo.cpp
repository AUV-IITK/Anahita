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

#include <vision_tasks/torpedoRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>
#include <vision_commons/geometry.h>
#include <torpedo.h>

Torpedo::Torpedo(){
	this->clahe_clip_ = 0.15;
	this->clahe_grid_size_ = 3;
	this->clahe_bilateral_iter_ = 2;
	this->balanced_bilateral_iter_ = 4;
	this->denoise_h_ = 5.6;
	this->low_h_ = 53;
	this->high_h_ = 86;
	this->low_s_ = 128;
	this->high_s_ = 255;
	this->low_v_ = 104;
	this->high_v_ = 202;
	this->opening_mat_point_ = 2;
	this->opening_iter_ = 1;
	this->closing_mat_point_ = 2;
	this->closing_iter_ = 3;
	this->camera_frame_ = "auv-iitk";
}

void Torpedo::callback(vision_tasks::torpedoRangeConfig &config, double level)
{
	Torpedo::clahe_clip_ = config.clahe_clip;
	Torpedo::clahe_grid_size_ = config.clahe_grid_size;
	Torpedo::clahe_bilateral_iter_ = config.clahe_bilateral_iter;
	Torpedo::balanced_bilateral_iter_ = config.balanced_bilateral_iter;
	Torpedo::denoise_h_ = config.denoise_h;
	Torpedo::low_h_ = config.low_h;
	Torpedo::high_h_ = config.high_h;
	Torpedo::low_s_ = config.low_s;
	Torpedo::high_s_ = config.high_s;
	Torpedo::low_v_ = config.low_v;
	Torpedo::high_v_ = config.high_v;
	Torpedo::opening_mat_point_ = config.opening_mat_point;
	Torpedo::opening_iter_ = config.opening_iter;
	Torpedo::closing_mat_point_ = config.closing_mat_point;
	Torpedo::closing_iter_ = config.closing_iter;
};

void Torpedo::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try
	{
		image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
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

void Torpedo::TaskHandling()
{
	image_transport::ImageTransport it(nh);
	image_transport::Publisher blue_filtered_pub = it.advertise("/torpedo_task/blue_filtered", 1);
	image_transport::Publisher thresholded_pub = it.advertise("/torpedo_task/thresholded", 1);
	image_transport::Publisher marked_pub = it.advertise("/torpedo_task/marked", 1);
	ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/torpedo_task/torpedo_coordinates", 1000);

	image_transport::Subscriber image_raw_sub = it.subscribe("/bottom_camera/image_raw", 1, &Torpedo::imageCallback, this);

	cv::Scalar torpedo_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar enclosing_rectangle_color(149, 255, 23);
	cv::Scalar contour_color(255, 0, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_marked;
	std::vector<std::vector<cv::Point> > contours;
	cv::Rect bounding_rectangle;
	geometry_msgs::PointStamped torpedo_point_message;
	torpedo_point_message.header.frame_id = camera_frame_.c_str();

	while (1)
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
				contours = vision_commons::Contour::getBestX(image_thresholded, 4);
				if (contours.size() != 0)
				{
					bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
					if (contours.size() == 2)
					{
						bounding_rectangle = cv::boundingRect(cv::Mat(contours[1]));
					}
					else if (contours.size() == 3)
					{
						cv::Rect bounding_rectangle1 = boundingRect(cv::Mat(contours[0]));
						cv::Rect bounding_rectangle2 = boundingRect(cv::Mat(contours[1]));
						cv::Rect bounding_rectangle3 = boundingRect(cv::Mat(contours[2]));
						if (bounding_rectangle1.contains(bounding_rectangle2.br()) && bounding_rectangle2.contains(bounding_rectangle2.tl()))
						{
							bounding_rectangle = bounding_rectangle2;
						}
						else if ((bounding_rectangle2.contains(bounding_rectangle3.br()) && bounding_rectangle2.contains(bounding_rectangle3.tl())) || (bounding_rectangle1.contains(bounding_rectangle3.br()) && bounding_rectangle1.contains(bounding_rectangle3.tl())))
						{
							bounding_rectangle = bounding_rectangle3;
						}
						else
						{
							if (bounding_rectangle3.br().y + bounding_rectangle3.tl().y > bounding_rectangle2.br().y + bounding_rectangle2.tl().y)
							{
								bounding_rectangle = bounding_rectangle3;
							}
							else
								bounding_rectangle = bounding_rectangle2;
						}
					}
					else if (contours.size() == 4)
					{
						cv::Rect bounding_rectangle1 = boundingRect(cv::Mat(contours[2]));
						cv::Rect bounding_rectangle2 = boundingRect(cv::Mat(contours[3]));
						if (bounding_rectangle1.br().y + bounding_rectangle1.tl().y > bounding_rectangle2.br().y + bounding_rectangle2.tl().y)
							bounding_rectangle = bounding_rectangle1;
						else
							bounding_rectangle = bounding_rectangle2;
					}
					torpedo_point_message.header.stamp = ros::Time();
					torpedo_point_message.point.x = pow((bounding_rectangle.br().x - bounding_rectangle.tl().x) / 7526.5, -.92678);
					torpedo_point_message.point.y = (bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2 - (image_.size().width) / 2;
					torpedo_point_message.point.z = ((float)image_.size().height) / 2 - (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2;
					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2, (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2), 1, torpedo_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_.size().width / 2, image_.size().height / 2), 1, image_center_color, 8, 0);
					cv::rectangle(image_marked, bounding_rectangle.tl(), bounding_rectangle.br(), enclosing_rectangle_color);
					for (int i = 0; i < contours.size(); i++)
						cv::drawContours(image_marked, contours, i, contour_color, 1, 8);
				}
			}
			blue_filtered_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
			thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());
			coordinates_pub.publish(torpedo_point_message);
			ROS_INFO("Torpedo Centre Location (x, y, z) = (%.2f, %.2f, %.2f)", torpedo_point_message.point.x, torpedo_point_message.point.y, torpedo_point_message.point.z);
			marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty");
		ros::spinOnce();
	}
}
