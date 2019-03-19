#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/Image.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>

#include <std_msgs/Header.h>

sensor_msgs::Image disparity_image;
sensor_msgs::Image depth_image;

cv::Mat cv_disparity_image;
cv::Mat_<cv::Vec3f> cv_depth_image;

void imageCB (const stereo_msgs::DisparityImage msg) {
    disparity_image = msg.image;
    cv_disparity_image = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    // cv::imshow ("disparity_image", cv_disparity_image);
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "depth_image_pub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<stereo_msgs::DisparityImage>("/anahita/disparity", 1, &imageCB);
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/anahita/depth", 1);

    ros::Time::init();

    ros::Rate loop_rate(10);

    cv::Mat_<float> Q(4, 4);

    Q.at<float>(0, 0) = 1;
    Q.at<float>(0, 1) = 0;
    Q.at<float>(0, 2) = 0;
    Q.at<float>(0, 3) = -384.5;
    Q.at<float>(1, 0) = 0;
    Q.at<float>(1, 1) = 1;
    Q.at<float>(1, 2) = 0;
    Q.at<float>(1, 3) = -246.5;
    Q.at<float>(2, 0) = 0;
    Q.at<float>(2, 1) = 0;
    Q.at<float>(2, 2) = 0;
    Q.at<float>(2, 3) = 407.268;
    Q.at<float>(3, 0) = 0;
    Q.at<float>(3, 1) = 0;
    Q.at<float>(3, 2) = -33.33;
    Q.at<float>(3, 3) = 0;

    std_msgs::Header header;
    header.frame_id = "stereo_camera";

    int count = 0;

    while (ros::ok()) {
        header.stamp = ros::Time::now();
        header.seq = count++;
        // reprojectImageTo3D (cv_disparity_image, cv_depth_image, Q, false, CV_32F);
        pub.publish(cv_bridge::CvImage(header, "bgr8", cv_disparity_image).toImageMsg());
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}