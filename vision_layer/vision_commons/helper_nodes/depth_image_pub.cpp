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
cv::Mat_<float> Q(4, 4);

cv::Mat_<cv::Vec3f> computeDepth () {
    cv::Mat_<cv::Vec3f> cv_depth_image(cv_disparity_image.rows, cv_disparity_image.cols);   // Output point cloud

    cv::Mat_<float> vec_tmp(4, 1);
    for(int y = 0; y < cv_disparity_image.rows; ++y) {
        for(int x = 0; x < cv_disparity_image.cols; ++x) {
            vec_tmp(0) = x; vec_tmp(1) = y; vec_tmp(2) = cv_disparity_image.at<float>(y, x); vec_tmp(3) = 1;

            vec_tmp = Q*vec_tmp;
            vec_tmp /= vec_tmp(3);
            cv::Vec3f &point = cv_depth_image.at<cv::Vec3f>(y, x);

            point[2] = vec_tmp(2);
            point[1] = vec_tmp(1);
            point[0] = vec_tmp(0);
        }
    }
    return cv_depth_image;
}

void imageCB (const stereo_msgs::DisparityImage msg) {
    disparity_image = msg.image;
    cv_disparity_image = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    // cv_depth_image = computeDepth();
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "depth_image_pub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<stereo_msgs::DisparityImage>("/anahita/disparity", 1, &imageCB);
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/anahita/depth", 1);

    ros::Time::init();

    ros::Rate loop_rate(10);

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
        reprojectImageTo3D (cv_disparity_image, cv_depth_image, Q, false, CV_32F);
        pub.publish(cv_bridge::CvImage(header, "bgr8", cv_depth_image).toImageMsg());
        loop_rate.sleep();
        ros::spinOnce();
        // stereo algorithm : StereoSGBM (1)
        // prefilter_size : 5
        // prefilter_cap : 1
        // correlation_window_size : 21
        // min_disparity : 0
        // disparity_range : 32
        // uniqueness_theorem : 0
        // texture_threshold : 0
        // speckle_size : 307
        // speckle_range : 5
        // fullDP = true
        // P1 : 1000
        // P2 : 0
        // disp12MaxDiff : 36
    }
    return 0;
}