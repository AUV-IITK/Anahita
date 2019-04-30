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

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


sensor_msgs::Image disparity_image;
sensor_msgs::Image depth_image;

cv::Mat cv_disparity_image;
cv::Mat_<cv::Vec3f> cv_depth_image;
// cv::Mat_<float> Q(4, 4);

pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
cv::Mat xyz(4, 4, CV_64F);
cv::Mat Q(4, 4, CV_64F);
cv::Mat left;

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
    ROS_INFO("inside the callback");
    // cv_depth_image = computeDepth();
}

void change_to_pcl () {
    pc->height = left.rows;
    pc->width = left.cols;
    pc->is_dense = true;

    for (int u = 0; u < left.rows; ++u)
    {
        for (int v = 0; v < left.cols; ++v)
        {
            cv::Vec3f point = xyz.at<cv::Vec3f>(u,v);
            // cv::Vec3b bgr(left.at<cv::Vec3b>(u,v));

            // pcl::PointXYZRGB p(static_cast<int>(bgr.val[0]), static_cast<int>(bgr.val[1]), static_cast<int>(bgr.val[2]));
            pcl::PointXYZ p;

            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
        
            std::cout << "3d point: " << p.x << " " << p.y << " " << p.z << std::endl;
            pc->push_back(p);

        }
    }
}

void leftImageCB (const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr cv_img_ptr;
    ROS_INFO("left image callback");
	try
	{
		left = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
		// ROS_INFO("Found a new image and stored it in left!");
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
    ros::init (argc, argv, "depth_image_pub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<stereo_msgs::DisparityImage>("/anahita/disparity", 1, &imageCB);
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/anahita/depth", 1, true);
    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/anahita/pointcloud", 1, true);
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber left_image_sub = it.subscribe("/anahita/left/image_raw", 1, &leftImageCB);

    ros::Time::init();

    ros::Rate loop_rate(10);

    // Q.at<float>(0, 0) = 1;
    // Q.at<float>(0, 1) = 0;
    // Q.at<float>(0, 2) = 0;
    // Q.at<float>(0, 3) = -384.5;
    // Q.at<float>(1, 0) = 0;
    // Q.at<float>(1, 1) = 1;
    // Q.at<float>(1, 2) = 0;
    // Q.at<float>(1, 3) = -246.5;
    // Q.at<float>(2, 0) = 0;
    // Q.at<float>(2, 1) = 0;
    // Q.at<float>(2, 2) = 0;
    // Q.at<float>(2, 3) = 407.268;
    // Q.at<float>(3, 0) = 0;
    // Q.at<float>(3, 1) = 0;
    // Q.at<float>(3, 2) = -33.33;
    // Q.at<float>(3, 3) = 0;

    double cm_right[3][3] = {{407.19, 0.000000e+00, 384.5}, {0.000000e+00, 407.19, 246.5}, {0.000000e+00, 0.000000e+00, 1.000000e+00}};
    double cm_left[3][3] = {{407.19, 0.000000e+00, 384.5}, {0.000000e+00, 407.19, 246.5}, {0.000000e+00, 0.000000e+00, 1.000000e+00}};
    double d_right[1][5] = {{-0.389, 0.118, 0.0012, 0.002, 0.0}};
    double d_left[1][5] = {{-0.389, 0.118, 0.0012, 0.002, 0.0}};

    cv::Mat CM_RIGHT (3, 3, CV_64FC1, cm_right);
    cv::Mat CM_LEFT (3, 3, CV_64FC1, cm_left);
    cv::Mat D_RIGHT(1, 5, CV_64FC1, d_right);
    cv::Mat D_LEFT(1, 5, CV_64FC1, d_left);

    double r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double t[1][3] = {{0.03, 0, 0}};

    cv::Mat R (3, 3, CV_64FC1, r);
    cv::Mat T (3, 1, CV_64FC1, t);

    // Mat   R, T;
    cv::Mat R1, R2, T1, T2, Q, P1, P2;

    std_msgs::Header header;
    header.frame_id = "stereo_camera";

    int count = 0;

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    
    double q[] =
	{
		1, 0, 0, -CM_LEFT.at<float>(0,2),
		0, 1, 0, -CM_LEFT.at<float>(1,2),
		0, 0, 0, CM_LEFT.at<float>(0,0),
		0, 0, 1.0/90, 0.0
	};

    cv::Mat Q_(4,4,CV_64F, q);

    while (ros::ok()) {
        header.stamp = ros::Time::now();
        header.seq = count++;

        if (cv_disparity_image.empty() || left.empty()) {
            ROS_INFO("image empty");
            ros::spinOnce();
            continue;
        }

        std::cout << "Disparity image channels: " << cv_disparity_image.channels() << std::endl;

        float focal_length = 407.19;
        float base_line = 0.03;

        for (int u = 0; u < cv_disparity_image.rows; ++u)
        {
            for (int v = 0; v < cv_disparity_image.cols; ++v)
            {
                float disparity = std::abs(xyz.at<float>(u,v));
                pcl::PointXYZ p;

                p.z = (focal_length*base_line)/disparity;
                p.x = (p.z*(v - cv_disparity_image.cols/2))/focal_length;
                p.y = (p.z*(cv_disparity_image.rows/2 - u))/focal_length;
        
                // std::cout << "3d point: " << p.x << " " << p.y << " " << p.z << std::endl;
                pc->push_back(p);
            }
        }

        sensor_msgs::PointCloud2 object_msg;
        pcl::toROSMsg(*pc, object_msg);
        object_msg.header.frame_id = "world";
        pc_pub.publish(object_msg); 

        // break;

        // stereoRectify (CM_RIGHT, D_RIGHT, CM_LEFT, D_LEFT, cv_disparity_image.size(), R, T, R1, R2, P1, P2, Q);
        // reprojectImageTo3D (cv_disparity_image, xyz, Q_, false, CV_32F);
        // change_to_pcl ();
        // viewer.showCloud(pc);
        // pub.publish(cv_bridge::CvImage (header, "bgr8", xyz).toImageMsg());
        // while (!viewer.wasStopped())
        // {
            // ROS_INFO("viewer stropped");
        // }
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
