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

image_transport::Publisher blue_filtered_pub;
image_transport::Publisher thresholded_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_pub;
std::string camera_frame = "auv-iitk";
geometry_msgs::PointStamped torpedo_point_message;
cv::Mat image;

void callback(vision_tasks::torpedoRangeConfig &config, double level){
    ROS_INFO("Reconfigure Request: (%d %d %d) - (%d %d %d): ", config.low_b, config.low_g, config.low_r, config.high_b, config.high_g, config.high_r);
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
    cv_bridge::CvImagePtr inputImagePtr;
    try {
	inputImagePtr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e) {
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    image = inputImagePtr->image;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "torpedo_task");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<vision_tasks::torpedoRangeConfig> server;
    dynamic_reconfigure::Server<vision_tasks::torpedoRangeConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher blue_filtered_pub = it.advertise("/torpedo_task/blue_filtered", 1);
    image_transport::Publisher thresholded_pub = it.advertise("/torpedo_task/thresholded", 1);
    image_transport::Publisher marked_pub = it.advertise("/torpedo_task/marked",1);
    ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/torpedo_task/torpedo_coordinates", 1000);

    image_transport::Subscriber image_raw_sub = it.subscribe("/bottom_camera/image_raw", 1, imageCallback);

    cv::Mat blue_filtered;
    cv::Mat image_thresholded;
    std::vector<std::vector<cv::Point> > contours;
    geometry_msgs::PointStamped torpedo_point_message;
    torpedo_point_message.header.frame_id = camera_frame.c_str();
    ros::Rate loop_rate(5);
    while(ros::ok()){
	cv::Mat image_marked = image;
	if(!image.empty()){
	    blue_filtered = vision_commons::Filter::blue_filter(image, clahe_clip, clahe_grid_size, clahe_bilateral_iter, balanced_bilateral_iter, denoise_h);
	    if(!(high_b<=low_b || high_g<=low_g || high_r<=low_r)) {
		image_thresholded = vision_commons::Threshold::threshold(blue_filtered, low_b, low_g, low_r, high_b, high_g, high_r);
		image_thresholded = vision_commons::Morph::open(image_thresholded, 2*opening_closing_mat_point+1, opening_closing_mat_point, opening_closing_mat_point, opening_iter);
		image_thresholded = vision_commons::Morph::close(image_thresholded, 2*opening_closing_mat_point+1, opening_closing_mat_point, opening_closing_mat_point, closing_iter);
		contours = vision_commons::Contour::getBestX(image_thresholded, 4);
		if (contours.size() != 0){
		    ROS_INFO("Contours size = %d", contours.size());
		    cv::Rect bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
		    if(contours.size() == 2) {
			bounding_rectangle = cv::boundingRect(cv::Mat(contours[1]));
		    }
		    else if(contours.size() == 3) {
			cv::Rect bounding_rectangle1 = boundingRect(cv::Mat(contours[0]));
			cv::Rect bounding_rectangle2 = boundingRect(cv::Mat(contours[1]));
			cv::Rect bounding_rectangle3 = boundingRect(cv::Mat(contours[2]));
			if(bounding_rectangle1.contains(bounding_rectangle2.br()) && bounding_rectangle2.contains(bounding_rectangle2.tl())) {
			    bounding_rectangle = bounding_rectangle2;
			}
			else if((bounding_rectangle2.contains(bounding_rectangle3.br()) && bounding_rectangle2.contains(bounding_rectangle3.tl()))
				|| (bounding_rectangle1.contains(bounding_rectangle3.br()) && bounding_rectangle1.contains(bounding_rectangle3.tl()))) {
			    bounding_rectangle = bounding_rectangle3;
			}
			else bounding_rectangle = bounding_rectangle2;
		    }
		    else if(contours.size() == 4) {
			cv::Rect bounding_rectangle1 = boundingRect(cv::Mat(contours[2]));
			cv::Rect bounding_rectangle2 = boundingRect(cv::Mat(contours[3]));
			if(bounding_rectangle1.br().y + bounding_rectangle1.tl().y > bounding_rectangle2.br().y + bounding_rectangle2.tl().y) bounding_rectangle = bounding_rectangle1;
			else bounding_rectangle = bounding_rectangle2;
		    }
		    cv::drawContours( image_marked, contours, -1, cv::Scalar(255,255,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
		    torpedo_point_message.header.stamp = ros::Time();
		    torpedo_point_message.point.x = pow((bounding_rectangle.br().x - bounding_rectangle.tl().x)/7526.5,-.92678);
		    torpedo_point_message.point.y = (bounding_rectangle.br().x + bounding_rectangle.tl().x)/2 - (image.size().width)/2;
		    torpedo_point_message.point.z = ((float)image.size().height)/2 - (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2;
		    cv::rectangle(image_marked, bounding_rectangle, cv::Scalar(255, 0, 0));
		    cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x)/2, (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2), 1, cv::Scalar(255,100,100), 8, 0);
		    cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(255,100, 50), 8, 0);
		}
	    }
	    blue_filtered_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
	    thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());
	    coordinates_pub.publish(torpedo_point_message);
	    ROS_INFO("Torpedo Centre Location (x, y, z) = (%.2f, %.2f, %.2f)", torpedo_point_message.point.x, torpedo_point_message.point.y, torpedo_point_message.point.z);
	    marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_marked).toImageMsg());
	}
	else ROS_INFO("Image empty");
	loop_rate.sleep();
	ros::spinOnce();
    }
}

