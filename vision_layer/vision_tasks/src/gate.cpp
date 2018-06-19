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

#include <vision_tasks/gateRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/blue_filter.h>

double clahe_clip = 4.0;
int clahe_grid_size = 8;
int clahe_bilateral_iter = 8;
int balanced_bilateral_iter = 4;
double denoise_h = 10.0;
int low_b = 10;
int low_g = 0;
int low_r = 0;
int high_b = 90;
int high_g= 255;
int high_r = 255;
int closing_mat_point = 1;
int closing_iter = 1;
image_transport::Publisher blue_filtered_pub;
image_transport::Publisher thresholded_HSV_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_front_pub;
ros::Publisher coordinates_bottom_pub;
std::string camera_frame = "auv-iitk";

void callback(vision_tasks::gateRangeConfig &config, double level){
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
    closing_mat_point = config.closing_mat_point;
    closing_iter = config.closing_iter;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	cv_bridge::CvImagePtr cv_img_ptr;
	try{
		cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_img_ptr->image;
		cv::Mat image_marked = cv_img_ptr->image;
		if(!image.empty()){
			cv::Mat blue_filtered = vision_commons::BlueFilter::filter(image, clahe_clip, clahe_grid_size, clahe_bilateral_iter, balanced_bilateral_iter, denoise_h);
  		blue_filtered_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
  		cv::Mat image_thresholded;
  		ROS_INFO("Thresholding Values: (%d %d %d) - (%d %d %d): ", low_b, low_g, low_r, high_b, high_g, high_r);
  		if(!(high_b<=low_b || high_g<=low_g || high_r<=low_r)) {
  			image_thresholded = vision_commons::Threshold::threshold(image, low_b, low_g, low_r, high_b, high_g, high_r);
  			image_thresholded = vision_commons::Morph::close(image_thresholded, 2*closing_mat_point+1, closing_mat_point, closing_mat_point, closing_iter);
    		cv_bridge::CvImage thresholded_ptr;
    		thresholded_ptr.header = msg->header;
    		thresholded_ptr.encoding = sensor_msgs::image_encodings::MONO8;
    		thresholded_ptr.image = image_thresholded;
    		thresholded_HSV_pub.publish(thresholded_ptr.toImageMsg());
    		std::vector<std::vector<cv::Point> > contours = vision_commons::Contour::getBestX(image_thresholded, 1);
    		ROS_INFO("contours size = %d", contours.size());
    		if(contours.size()!=0){
          cv::Rect bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
          geometry_msgs::PointStamped gate_point_message;
					gate_point_message.header.stamp = ros::Time();
					gate_point_message.header.frame_id = camera_frame.c_str();
					gate_point_message.point.x = 0.0;
					gate_point_message.point.y = (bounding_rectangle.br().x + bounding_rectangle.tl().x)/2 - (image.size().width)/2;
					gate_point_message.point.z = ((float)image.size().height)/2 - (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2;
					ROS_INFO("Gate Location (y, z) = (%.2f, %.2f)", gate_point_message.point.y, gate_point_message.point.z);
          coordinates_front_pub.publish(gate_point_message);
          cv::rectangle(image_marked, bounding_rectangle, cv::Scalar(255,0,255), 1, 8, 0);
          cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x)/2, (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2), 1, cv::Scalar(255,100,100), 8, 0);
					cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(255,100, 50), 8, 0);
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
    ros::init(argc, argv, "gate_task");
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<vision_tasks::gateRangeConfig> server;
    dynamic_reconfigure::Server<vision_tasks::gateRangeConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    image_transport::ImageTransport it(nh);
    blue_filtered_pub = it.advertise("/blue_filtered", 1);
    thresholded_HSV_pub = it.advertise("/thresholded", 1);
    marked_pub = it.advertise("/marked",1);
    coordinates_front_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/gate_front_coordinates", 1000);
    image_transport::Subscriber image_raw_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
}
