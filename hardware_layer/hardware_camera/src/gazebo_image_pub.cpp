#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

sensor_msgs::Image front_image;
sensor_msgs::Image bottom_image;

void bottomCB (const sensor_msgs::Image::ConstPtr &msg) {
	try
	{
		bottom_image = *msg;
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

void frontCB (const sensor_msgs::Image::ConstPtr &msg) {
	try
	{
		front_image = *msg;
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

int main (int argc, char** argv) {
    ros::init (argc, argv, "gazebo_camera_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it (nh);

    image_transport::Subscriber front_sub = it.subscribe("/rezrov/bottomCamera/image", 1, &bottomCB);
    image_transport::Subscriber bottom_sub = it.subscribe("/rezrov/frontCamera/image", 1, &bottomCB);

    image_transport::Publisher front_pub = it.advertise("/anahita/front_camera/image_raw", 1);
    image_transport::Publisher bottom_pub = it.advertise("/anahita/bottom_camera/image_raw", 1);

    ros::Rate loop_rate(20);

    while (ros::ok()) {

        front_pub.publish (front_image);
        bottom_pub.publish (bottom_image);
        loop_rate.sleep ();
        ros::spinOnce ();
    }

    return 0;

} 