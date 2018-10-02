#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>
#include <cmath>
#include <std_msgs/Bool.h>
#include <vision_tasks/gateFrontRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/filter.h>
#include <vision_commons/geometry.h>

double clahe_clip_ = 4.0;
int clahe_grid_size_ = 8;
int clahe_bilateral_iter_ = 8;
int balanced_bilateral_iter_ = 4;
double denoise_h_ = 10.0;
int low_h_ = 0;
int high_h_ = 255;
int low_s_ = 0;
int high_s_ = 255;
int low_v_ = 0;
int high_v_ = 255;
int closing_mat_point_ = 1;
int closing_iter_ = 1;
int canny_threshold_low_ = 0;
int canny_threshold_high_ = 1000;
int canny_kernel_size_ = 3;
int hough_threshold_ = 0;
int hough_minline_ = 0;
int hough_maxgap_ = 0;
double hough_angle_tolerance_ = 0.0;
double gate_distance_tolerance_ = 50.0;
double gate_angle_tolerance_ = 0.0;

cv::Mat image_;

std::string camera_frame_ = "auv-iitk";

cv::Point2i rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle)
{
	if (v1.x > v2.x)
	{
		cv::Point2i v3 = v1 - v2;
		cv::Point2i finalVertex;
		finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
		finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
		finalVertex = finalVertex + v2;
		return finalVertex;
	}
	else
	{
		cv::Point2i v3 = v2 - v1;
		cv::Point2i finalVertex;
		finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
		finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
		finalVertex = finalVertex + v1;
		return finalVertex;
	}
}

void callback(vision_tasks::gateFrontRangeConfig &config, double level)
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
	closing_mat_point_ = config.closing_mat_point;
	closing_iter_ = config.closing_iter;
	canny_threshold_low_ = config.canny_threshold_low;
	canny_threshold_high_ = config.canny_threshold_high;
	canny_kernel_size_ = config.canny_kernel_size;
	hough_threshold_ = config.hough_threshold;
	hough_minline_ = config.hough_minline;
	hough_maxgap_ = config.hough_maxgap;
	hough_angle_tolerance_ = config.hough_angle_tolerance;
	gate_distance_tolerance_ = config.gate_distance_tolerance;
	gate_angle_tolerance_ = config.gate_angle_tolerance;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
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
	ros::init(argc, argv, "gate_task_front");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher blue_filtered_pub = it.advertise("/gate_task/front/blue_filtered", 1);
	image_transport::Publisher thresholded_pub = it.advertise("/gate_task/front/thresholded", 1);
	image_transport::Publisher canny_pub = it.advertise("/gate_task/front/canny", 1);
	image_transport::Publisher lines_pub = it.advertise("/gate_task/front/lines", 1);
	image_transport::Publisher marked_pub = it.advertise("/gate_task/front/marked", 1);
	ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/front/gate_coordinates", 1000);

//	image_transport::Subscriber front_image_sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
	image_transport::Subscriber front_image_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);

	cv::Scalar gate_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar hough_line_color(0, 164, 253);
	cv::Scalar horizontal_line_color(253, 0, 127);
	cv::Scalar vertical_line_color(0, 253, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_gray;
	cv::Mat image_canny;
	cv::Mat image_lines;
	cv::Mat image_marked;
	std::vector<cv::Vec4i> lines;
	std::vector<cv::Vec4i> lines_filtered;
	std::vector<double> angles;
	int found = 0;
	cv::Point2i pi1;
	cv::Point2i pi2;
	cv::Point2i pj1;
	cv::Point2i pj2;
	cv::Point2i horizontal1(0, 0);
	cv::Point2i horizontal2(0, 0);
	cv::Point2i vertical1(0, 0);
	cv::Point2i vertical2(0, 0);
	cv::Point2i longest1(0, 0);
	cv::Point2i longest2(0, 0);
	geometry_msgs::PointStamped gate_point_message;
	gate_point_message.header.frame_id = camera_frame_.c_str();

	while (ros::ok())
	{
		if (!image_.empty())
		{
			image_.copyTo(image_marked);
			//blue_filtered = vision_commons::Filter::blue_filter(image_, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
			if (high_h_ > low_h_ && high_s_ > low_s_ && high_v_ > low_v_)
			{
				cv::cvtColor(image_, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, low_h_, high_h_, low_s_, high_s_, low_v_, high_v_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * closing_mat_point_ + 1, closing_mat_point_, closing_mat_point_, closing_iter_);
				cv::cvtColor(image_thresholded, image_gray, CV_GRAY2BGR);
				cv::Canny(image_gray, image_canny, canny_threshold_low_, canny_threshold_high_, canny_kernel_size_);
				image_lines = blue_filtered;
				cv::HoughLinesP(image_thresholded, lines, 1, CV_PI / 180, hough_threshold_, hough_minline_, hough_maxgap_);
				double angle = 0.0;
				for (int i = 0; i < lines.size(); i++)
				{
					cv::Point p1(lines[i][0], lines[i][1]);
					cv::Point p2(lines[i][2], lines[i][3]);
					angle = vision_commons::Geometry::angleWrtY(p1, p2);
					if (angle < hough_angle_tolerance_ || abs(angle - 90.0) < hough_angle_tolerance_ || abs(180.0 - angle) < hough_angle_tolerance_)
					{
						cv::line(image_lines, p1, p2, hough_line_color, 3, CV_AA);
						lines_filtered.push_back(lines[i]);
						angles.push_back(angle);
					}
				}
				for (int i = 0; i < lines_filtered.size(); i++)
				{
					pi1.x = lines_filtered[i][0];
					pi1.y = lines_filtered[i][1];
					pi2.x = lines_filtered[i][2];
					pi2.y = lines_filtered[i][3];
					if (vision_commons::Geometry::distance(pi1, pi2) > vision_commons::Geometry::distance(longest1, longest2))
					{
						longest1.x = pi1.x;
						longest1.y = pi1.y;
						longest2.x = pi2.x;
						longest2.y = pi2.y;
					}
					for (int j = i + 1; j < lines_filtered.size(); j++)
					{
						pj1.x = lines_filtered[j][0];
						pj1.y = lines_filtered[j][1];
						pj2.x = lines_filtered[j][2];
						pj2.y = lines_filtered[j][3];
						if (abs(angles[i] - angles[j] - 90.0) < gate_angle_tolerance_)
						{
							double distance1 = vision_commons::Geometry::distance(pi1, pj1);
							double distance2 = vision_commons::Geometry::distance(pi1, pj2);
							double distance3 = vision_commons::Geometry::distance(pi2, pj1);
							double distance4 = vision_commons::Geometry::distance(pi2, pj2);
							if (distance1 < gate_distance_tolerance_ || distance2 < gate_distance_tolerance_ || distance3 < gate_distance_tolerance_ || distance4 < gate_distance_tolerance_)
							{
								if (abs(angles[j] - 90.0) < abs(angles[i] - 90.0) && (vision_commons::Geometry::distance(pj1, pj2) > vision_commons::Geometry::distance(horizontal1, horizontal2) || vision_commons::Geometry::distance(pi1, pi2) > vision_commons::Geometry::distance(vertical1, vertical2)))
								{
									horizontal1.x = pj1.x;
									horizontal1.y = pj1.y;
									horizontal2.x = pj2.x;
									horizontal2.y = pj2.y;
									vertical1.x = pi1.x;
									vertical1.y = pi1.y;
									vertical2.x = pi2.x;
									vertical2.y = pi2.y;
									found = 1;
								}
								else if (abs(angles[j] - 90.0) > abs(angles[i] - 90.0) && (vision_commons::Geometry::distance(pi1, pi2) > vision_commons::Geometry::distance(horizontal1, horizontal2) || vision_commons::Geometry::distance(pj1, pj2) > vision_commons::Geometry::distance(vertical1, vertical2)))
								{
									horizontal1.x = pi1.x;
									horizontal1.y = pi1.y;
									horizontal2.x = pi2.x;
									horizontal2.y = pi2.y;
									vertical1.x = pj1.x;
									vertical1.y = pj1.y;
									vertical2.x = pj2.x;
									vertical2.y = pj2.y;
									found = 1;
								}
							}
						}
					}
				}
				gate_point_message.header.stamp = ros::Time();
				if (found)
				{
					gate_point_message.point.x = pow(sqrt(pow(vision_commons::Geometry::distance(horizontal1, horizontal2), 2) + pow(vision_commons::Geometry::distance(vertical1, vertical2), 2)) / 7526.5, -.92678);
					gate_point_message.point.y = (horizontal1.x + horizontal2.x) / 2 - image_.size().width / 2;
					gate_point_message.point.z = image_.size().height / 2 - (vertical1.y + vertical2.y) / 2;
					ROS_INFO("Gate Center (x, y, z) = (%.2f, %.2f, %.2f)", gate_point_message.point.x, gate_point_message.point.y, gate_point_message.point.z);
					cv::line(image_marked, horizontal1, horizontal2, horizontal_line_color, 3, CV_AA);
					cv::line(image_marked, vertical1, vertical2, vertical_line_color, 3, CV_AA);
				}
				else
				{
					if (abs(vision_commons::Geometry::angleWrtY(longest1, longest2) - 90.0) < hough_angle_tolerance_)
					{
						gate_point_message.point.x = pow((vision_commons::Geometry::distance(longest1, longest2) * 1.068) / 7526.5, -.92678);
						gate_point_message.point.y = (longest1.x + longest2.x) / 2 - image_.size().width / 2;
						gate_point_message.point.z = image_.size().height / 2 - (longest1.y + longest2.y) / 2 + 9 * vision_commons::Geometry::distance(longest1, longest2) / 48;
					}
					else
					{
						gate_point_message.point.x = pow((vision_commons::Geometry::distance(longest1, longest2) * 2.848) / 7526.5, -.92678);
						gate_point_message.point.y = (longest1.x + longest2.x) / 2 + 12 * vision_commons::Geometry::distance(longest1, longest2) / 9 - image_.size().width / 2;
						gate_point_message.point.z = image_.size().height / 2 - (longest1.y + longest2.y) / 2;
					}
					cv::line(image_marked, longest1, longest2, hough_line_color, 3, CV_AA);
					ROS_INFO("Couldn't find gate, estimated gate center (x, y, z) = (%.2f, %.2f, %.2f)", gate_point_message.point.x, gate_point_message.point.y, gate_point_message.point.z);
				}
				cv::circle(image_marked, cv::Point(gate_point_message.point.y + image_.size().width / 2, image_.size().height / 2 - gate_point_message.point.z), 1, gate_center_color, 8, 0);
				cv::circle(image_marked, cv::Point(image_.size().width / 2, image_.size().height / 2), 1, image_center_color, 8, 0);
			}
			blue_filtered_pub.publish(cv_bridge::CvImage(gate_point_message.header, "bgr8", blue_filtered).toImageMsg());
			thresholded_pub.publish(cv_bridge::CvImage(gate_point_message.header, "mono8", image_thresholded).toImageMsg());
			canny_pub.publish(cv_bridge::CvImage(gate_point_message.header, "mono8", image_canny).toImageMsg());
			lines_pub.publish(cv_bridge::CvImage(gate_point_message.header, "bgr8", image_lines).toImageMsg());
			coordinates_pub.publish(gate_point_message);
			marked_pub.publish(cv_bridge::CvImage(gate_point_message.header, "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty");
		ros::spinOnce();
	}
	return 0;
}
